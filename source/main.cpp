#include <algorithm>
#include <chrono>
#include <cmath>
#include <fstream>
#include <iostream>
#include <numeric>
#include <vector>

using std::cout;
using std::string;
using std::vector;

const double RHO_AIR = 1.171774;
const double G = 9.81;

auto get_rho_air(double temp_c = 22.0, double elevation_m = 180.0) -> double;
auto get_rho_air(double temp_c, double elevation_m) -> double
{
  // actually implement this
  return RHO_AIR;
}

auto trapz_integration(vector<double> x, vector<double> y) -> vector<double>
{
  vector<double> result(x.size(), 0.0);
  for (size_t i = 1; i < x.size(); ++i) {
    result[i] = result[i - 1] + (x[i] - x[i - 1]) * (y[i] + y[i - 1]) / 2.0;
  }
  return result;
}

auto interpolate(const vector<double>& xData,
                 const vector<double>& yData,
                 double x,
                 bool extrapolate) -> double
{
  // from http://www.cplusplus.com/forum/general/216928/
  int size = xData.size();

  int i = 0;
  if (x >= xData[size - 2]) {
    i = size - 2;
  } else {
    while (x > xData[i + 1])
      i++;
  }
  double xL = xData[i], yL = yData[i], xR = xData[i + 1], yR = yData[i + 1];
  if (!extrapolate) {
    if (x < xL)
      yR = yL;
    if (x > xR)
      yL = yR;
  }

  double dydx = (yR - yL) / (xR - xL);

  return yL + dydx * (x - xL);
}

struct TimeTrace
{
  vector<double> time;
  vector<double> speed;
  vector<double> dt_s;
  auto distance() const -> vector<double>
  {
    return trapz_integration(time, speed);
  };
  TimeTrace(vector<double>& time, vector<double>& speed)
      : time(time)
      , speed(speed)
  {
    dt_s = vector<double>(time.size() - 1, 0.0);
    std::transform(time.begin(),
                   time.end() - 1,
                   time.begin() + 1,
                   dt_s.begin(),
                   [](double t1, double t2) { return t2 - t1; });
  }
};

struct BasicTrain
{
  double roll_resist;
  double drag_coeff;
  double frnt_area_m2;
  double m_kg;
};

struct LocoConsist
{
  double fc_pw_peak_w;
  vector<double> fc_eta_array;
  vector<double> fc_pwr_array;
};

struct SimulationState
{
  TimeTrace trace;
  const BasicTrain train;
  const LocoConsist loco_con;

  vector<double> rr_pwr_w;
  vector<double> drag_pw_w;
  vector<double> accel_pwr_w;
  vector<double> brake_pwr_w;
  vector<double> rho_air_kg__m3;
  vector<double> req_pwr_w;
  vector<double> fuel_pwr_w;
  vector<double> trace_miss_iters;
  vector<bool> trace_met;

  // constructor
  SimulationState(TimeTrace& trace,
                  const BasicTrain& train,
                  const LocoConsist& loco_con)
      : trace(trace)
      , train(train)
      , loco_con(loco_con)
  {
    size_t n = trace.time.size();
    rr_pwr_w = vector<double>(n, 0.0);
    drag_pw_w = vector<double>(n, 0.0);
    accel_pwr_w = vector<double>(n, 0.0);
    brake_pwr_w = vector<double>(n, 0.0);
    rho_air_kg__m3 = vector<double>(n, 0.0);
    req_pwr_w = vector<double>(n, 0.0);
    fuel_pwr_w = vector<double>(n, 0.0);
    trace_miss_iters = vector<double>(n, 0.0);
    trace_met = vector<bool>(n, false);
  }
};

void solve_required_power(SimulationState& state, int t)
{
  // TODO: get grade from trace
  auto grade = 0.0;
  const BasicTrain& train = state.train;
  const TimeTrace& trace = state.trace;

  double rho_air = get_rho_air();

  state.rho_air_kg__m3[t] = rho_air;
  state.rr_pwr_w[t] = train.roll_resist * std::cos(std::atan(grade))
      * train.m_kg * G * 0.5 * (trace.speed[t - 1] + trace.speed[t]);

  state.drag_pw_w[t] = 0.5 * rho_air * train.drag_coeff * train.frnt_area_m2
      * std::pow(0.5 * (trace.speed[t - 1] + trace.speed[t]), 3);

  state.accel_pwr_w[t] = train.m_kg / (2 * trace.dt_s[t])
      * (std::pow(trace.speed[t], 2) - std::pow(trace.speed[t - 1], 2));

  // TODO: add grade and curvature power

  state.req_pwr_w[t] =
      state.rr_pwr_w[t] + state.drag_pw_w[t] + state.accel_pwr_w[t];
}

void solve_missed_trace(SimulationState& state, int t)
{
  state.trace_met[t] = state.req_pwr_w[t] <= state.loco_con.fc_pw_peak_w;
  vector<double> speed_guess = {
      state.trace.speed[t],
      state.trace.speed[t]
          * std::pow(state.loco_con.fc_pw_peak_w / state.req_pwr_w[t], 3)};
  vector<double> pwr_shortage = {state.req_pwr_w[t]
                                 - state.loco_con.fc_pw_peak_w};

  while (!state.trace_met[t]) {
    if (state.trace_miss_iters[t] > 10.0) {
      break;
    }
    state.trace_miss_iters[t] += 1.0;
    state.trace.speed[t] = speed_guess[speed_guess.size() - 1];
    solve_required_power(state, t);
    pwr_shortage.push_back(state.req_pwr_w[t] - state.loco_con.fc_pw_peak_w);
    double new_speed_guess = speed_guess[speed_guess.size() - 1]
        - (speed_guess[speed_guess.size() - 1]
           - speed_guess[speed_guess.size() - 2])
            / (pwr_shortage[pwr_shortage.size() - 1]
               - pwr_shortage[pwr_shortage.size() - 2]);
    state.trace.speed[t] = new_speed_guess;
    speed_guess.push_back(new_speed_guess);
    state.trace_met[t] = state.req_pwr_w[t] <= state.loco_con.fc_pw_peak_w;
  }
}

void solve_braking(SimulationState& state, int t)
{
  state.brake_pwr_w[t] = std::max(0.0, -state.req_pwr_w[t]);
}

void solve_energy_consumption(SimulationState& state, int t)
{
  if (state.req_pwr_w[t] > 0.0) {
    state.fuel_pwr_w[t] = state.req_pwr_w[t]
        / interpolate(state.loco_con.fc_pwr_array,
                      state.loco_con.fc_eta_array,
                      state.req_pwr_w[t],
                      false);
  }
}

void step(SimulationState& state, int t)
{
  solve_required_power(state, t);
  solve_missed_trace(state, t);
  solve_braking(state, t);
  solve_energy_consumption(state, t);
}

auto main() -> int
{
  vector<double> speed {};

  vector<double> ramp_up(100);
  std::iota(ramp_up.begin(), ramp_up.end(), 0.0);
  for (auto& s : ramp_up) {
    s = s / 5;
  }
  speed.insert(speed.end(), ramp_up.begin(), ramp_up.end());

  vector<double> coast(3000, 20);
  speed.insert(speed.end(), coast.begin(), coast.end());

  vector<double> ramp_down = ramp_up;
  std::reverse(ramp_down.begin(), ramp_down.end());
  speed.insert(speed.end(), ramp_down.begin(), ramp_down.end());

  vector<double> time(speed.size());
  std::iota(time.begin(), time.end(), 0.0);

  TimeTrace trace {time, speed};
  BasicTrain train {0.005, 10, 5, 50000};

  double max_power = 500000;
  vector<double> power_frac {0.1, 0.325, 0.55, 0.775, 1.0};
  for (auto& p : power_frac) {
    p = p * max_power;
  }
  LocoConsist loco_con {max_power, {0.2, 0.32, 0.35, 0.4, 0.38}, power_frac};

  SimulationState state {trace, train, loco_con};

  const auto start = std::chrono::high_resolution_clock::now();
  for (int t = 1; t < speed.size(); ++t) {
    step(state, t);
  }
  const auto end = std::chrono::high_resolution_clock::now();

  cout << "Time taken: "
       << std::chrono::duration_cast<std::chrono::microseconds>(end - start)
              .count()
       << " microseconds" << '\n';

  // write to file
  std::ofstream outfile;
  outfile.open("results.csv");

  // write header
  outfile << "time,speed,req_pwr_w,rr_pwr_w,drag_pwr_w,accel_pwr_w,brake_pwr_w,"
             "fuel_pwr_w,trace_met,trace_miss_iters"
          << '\n';
  for (const int& t : time) {
    outfile << t << ',' << state.trace.speed[t] << ',' << state.req_pwr_w[t]
            << ',' << state.rr_pwr_w[t] << ',' << state.drag_pw_w[t] << ','
            << state.accel_pwr_w[t] << ',' << state.brake_pwr_w[t] << ','
            << state.fuel_pwr_w[t] << ',' << state.trace_met[t] << ','
            << state.trace_miss_iters[t] << '\n';
  }
  outfile.close();

  return 0;
}
