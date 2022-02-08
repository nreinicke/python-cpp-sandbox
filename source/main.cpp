#include <algorithm>
#include <chrono>
#include <cmath>
#include <fstream>
#include <iostream>
#include <numeric>
#include <vector>

#include "lib.hpp"

using std::cout;
using std::string;
using std::vector;

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
