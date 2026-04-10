#include "mjbotscpp_util.h"

#include <iomanip>
#include <sstream>

namespace mjbotscpp {

TimerMonitor::Slot::Slot(int w) : window(w, 0) {}

TimerMonitor::TimerMonitor(int window) : window_size_(window) {}

size_t TimerMonitor::Register(const std::string& name) {
  const size_t idx = slots_.size();
  name_to_idx_[name] = idx;
  names_.push_back(name);
  slots_.push_back(std::make_unique<Slot>(window_size_));
  return idx;
}

void TimerMonitor::Start(size_t idx) {
  slots_[idx]->t0 = std::chrono::steady_clock::now();
}

void TimerMonitor::Stop(size_t idx) {
  Slot& s = *slots_[idx];
  const long us = std::chrono::duration_cast<std::chrono::microseconds>(
      std::chrono::steady_clock::now() - s.t0).count();
  const long w = s.worst.load(std::memory_order_relaxed);
  if (us > w) s.worst.store(us, std::memory_order_relaxed);
  s.sum -= s.window[s.widx];
  s.window[s.widx] = us;
  s.sum += us;
  s.widx = (s.widx + 1) % window_size_;
  s.avg.store(s.sum / window_size_, std::memory_order_relaxed);
}

long TimerMonitor::Worst(size_t idx) const {
  return slots_[idx]->worst.load(std::memory_order_relaxed);
}

long TimerMonitor::Avg(size_t idx) const {
  return slots_[idx]->avg.load(std::memory_order_relaxed);
}

size_t TimerMonitor::Size() const {
  return slots_.size();
}

std::string TimerMonitor::Report() const {
  std::ostringstream oss;
  for (size_t i = 0; i < slots_.size(); ++i) {
    if (i) oss << " | ";
    oss << names_[i] << ": worst=" << std::setw(6) << Worst(i) << "us"
        << " avg=" << std::setw(6) << Avg(i) << "us";
  }
  return oss.str();
}

} // namespace mjbotscpp
