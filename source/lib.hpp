#pragma once

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

const double R_AIR = 287;
const double G = 9.81;

auto get_rho_air(double temp_c = 22.0, double elevation_m = 180.0) -> double;

auto trapz_integration(vector<double> x, vector<double> y) -> vector<double>;

auto interpolate(const vector<double>& xData,
                 const vector<double>& yData,
                 double x,
                 bool extrapolate=false) -> double;

// string DEFAULT_TIME_TRACE_DIR = "../resources/";

struct TimeTrace
{
  vector<double> time_s;
  vector<double> speed_m__s;
  vector<double> dt_s;
  auto dist_m() const -> vector<double>
  {
    return trapz_integration(time_s, speed_m__s);
  };
  TimeTrace(vector<double>& time_s, vector<double>& speed_m__s)
      : time_s(time_s)
      , speed_m__s(speed_m__s)
  {
    dt_s = vector<double>(time_s.size() - 1, 0.0);
    std::transform(time_s.begin(),
                   time_s.end() - 1,
                   time_s.begin() + 1,
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
    size_t n = trace.time_s.size();
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

void solve_required_power(SimulationState& state, int t);
void solve_missed_trace(SimulationState& state, int t);
void solve_braking(SimulationState& state, int t);
void solve_energy_consumption(SimulationState& state, int t);
void step(SimulationState& state, int t);

