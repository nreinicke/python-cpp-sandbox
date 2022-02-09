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
  vector<double> speed_m__s {};
  vector<double> time_s(speed_m__s.size());

  std::ifstream time_trace_infile;
  time_trace_infile.open("../resources/demo_time_trace.csv");
  for (){

  }

  TimeTrace trace {time_s, speed_m__s};
  BasicTrain train {0.005, 10, 5, 50000};

  double max_power = 500000;
  vector<double> power_frac {0.1, 0.325, 0.55, 0.775, 1.0};
  vector<double> fc_pwr_vec_w(power_frac.size());
  for (int i=0; i < power_frac.size(); ++i) {
    fc_pwr_vec_w[i] = power_frac[i] * max_power;
  }
  
  LocoConsist loco_con {max_power, {0.2, 0.32, 0.35, 0.4, 0.38}, fc_pwr_vec_w};

  SimulationState state {trace, train, loco_con};

  const auto start = std::chrono::high_resolution_clock::now();
  for (int t = 1; t < speed_m__s.size(); ++t) {
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
  outfile << "time_s,speed_m__s,req_pwr_w,rr_pwr_w,drag_pwr_w,accel_pwr_w,brake_pwr_w,"
             "fuel_pwr_w,trace_met,trace_miss_iters"
          << '\n';
  for (const int& t : time_s) {
    outfile << t << ',' << state.trace.speed_m__s[t] << ',' << state.req_pwr_w[t]
            << ',' << state.rr_pwr_w[t] << ',' << state.drag_pw_w[t] << ','
            << state.accel_pwr_w[t] << ',' << state.brake_pwr_w[t] << ','
            << state.fuel_pwr_w[t] << ',' << state.trace_met[t] << ','
            << state.trace_miss_iters[t] << '\n';
  }
  outfile.close();

  return 0;
}
