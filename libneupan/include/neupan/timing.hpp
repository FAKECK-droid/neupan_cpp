// SPDX-License-Identifier: GPL-3.0-or-later
#pragma once

#include <chrono>

namespace neupan {

// Wall time independent of ROS / simulation time and system-clock adjustments.
class Stopwatch {
 public:
  double elapsedMs() const {
    return std::chrono::duration<double, std::milli>(
        std::chrono::steady_clock::now() - start_).count();
  }
  void restart() { start_ = std::chrono::steady_clock::now(); }

 private:
  std::chrono::steady_clock::time_point start_ = std::chrono::steady_clock::now();
};

}  // namespace neupan
