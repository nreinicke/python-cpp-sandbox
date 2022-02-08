#include "lib.hpp"

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

