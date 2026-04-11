#pragma once

#include <atomic>
#include <chrono>
#include <memory>
#include <string>
#include <unordered_map>
#include <vector>

namespace mjbotscpp {

// Tracks worst-case and rolling-average execution time for named code blocks.
// Thread-safe for concurrent reads (e.g. from a print thread) via relaxed atomics.
class TimerMonitor {
public:
  explicit TimerMonitor(int window = 100);

  // Register a named slot before the hot loop; returns its index for fast access.
  size_t Register(const std::string& name);

  void Start(size_t idx);
  void Stop(size_t idx);

  long Worst(size_t idx) const;
  long Avg(size_t idx)   const;
  size_t Size() const;

  // Returns a one-line summary: "name: worst=Xus avg=Xus | ..."
  std::string Report() const;

private:
  struct Slot {
    std::chrono::steady_clock::time_point t0;
    std::vector<long> window;
    long sum = 0;
    int widx = 0;
    std::atomic<long> worst{0};
    std::atomic<long> avg{0};
    explicit Slot(int w);
  };

  int window_size_;
  std::vector<std::unique_ptr<Slot>> slots_;
  std::vector<std::string> names_;
  std::unordered_map<std::string, size_t> name_to_idx_;
};

struct ScopedTimer {
  TimerMonitor& mon;
  const size_t idx;
  ScopedTimer(TimerMonitor& m, size_t i) : mon(m), idx(i) { mon.Start(idx); }
  ~ScopedTimer() { mon.Stop(idx); }
};

// Tracks worst-case and rolling-average of a double value (e.g. loss rates).
// Thread-safe for concurrent reads via relaxed atomics.
class RateMonitor {
public:
  explicit RateMonitor(int window = 100);

  // Record a new sample.
  void Update(double value);

  double Worst() const;
  double Avg()   const;

private:
  int window_size_;
  std::vector<double> window_;
  double sum_ = 0.0;
  int widx_ = 0;
  std::atomic<double> worst_{0.0};
  std::atomic<double> avg_{0.0};
};

} // namespace mjbotscpp
