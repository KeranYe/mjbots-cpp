#pragma once

#include <vector>
#include <memory>
#include <string>
#include <map>
#include <atomic>
#include <unordered_map>

// #include <ostream>

#include "moteus.h"
#include "pi3hat_moteus_transport.h"
#include "mjbotscpp.h"
#include "mjbotscpp_util.h"

using namespace mjbots;

namespace mjbotscpp {

/**
 * @brief A thread-safe double-buffered object for one-writer one-reader scenarios.
 * 
 * @tparam T The type of the object to be double-buffered.
 */
template <typename T>
class DoubleBufferedObject {
 public:

  // constructor
  DoubleBufferedObject( const T &init_obj ) {
    // initialize to the same intput
    buffers_[0] = init_obj;
    buffers_[1] = init_obj;
  }

  // --- Writer side ---

  // Get the back buffer to write into.
  T& BackBuffer() {
    return buffers_[1 - front_idx_.load(std::memory_order_acquire)];
  }

  // Atomically publish the back buffer as the new front.
  void Publish() {
    int back = 1 - front_idx_.load(std::memory_order_acquire);
    front_idx_.store(back, std::memory_order_release);
  }

  // --- Reader side ---

  // Get a const reference to the last published buffer.
  const T& Read() const {
    return buffers_[front_idx_.load(std::memory_order_acquire)];
  }

 private:
  T buffers_[2]; // double buffer storage
  std::atomic<int> front_idx_{0}; // index of the front buffer to read from
};

/**
 * @brief A thread-safe double-buffered map for one-writer one-reader scenarios.
 * 
 * @tparam K The type of the key.
 * @tparam V The type of the value.
 */
template <typename K, typename V>
class DoubleBufferedMap {
 public:
  using Map = std::unordered_map<K, V>;

  /**
   * @brief Constructor for double-buffered Map, with pre-allocation of memory. 
   * 
   * @param max_count The maximum number of elements to reserve in each buffer.
   * @param init_map The initial map to populate each buffer with.
   */
  explicit DoubleBufferedMap(const size_t max_count, const Map& init_map) {
    _max_count = max_count > init_map.size() ? max_count : init_map.size();
    _buffers[0].reserve(_max_count);
    _buffers[1].reserve(_max_count);
    _buffers[0] = init_map;
    _buffers[1] = init_map;
  }

  // --- Writer side ---

  Map& BackBuffer() {
    return _buffers[1 - _front_idx.load(std::memory_order_acquire)];
  }

  void Publish() {
    int back = 1 - _front_idx.load(std::memory_order_acquire);
    _front_idx.store(back, std::memory_order_release);
  }

  // --- Reader side ---

  const Map& Read() const {
    return _buffers[_front_idx.load(std::memory_order_acquire)];
  }

 private:
  size_t _max_count; // maximum number of elements to reserve
  Map _buffers[2];
  std::atomic<int> _front_idx{0};
};

/**
 * @brief Mjbots Servo Command
 * 
 */
struct ServoCommand {
  moteus::Mode mode = moteus::Mode::kStopped;
  moteus::PositionMode::Command position;
  moteus::PositionMode::Format resolution;
};

/**
 * @brief Mjbots Servo Reply
 * 
 */
struct ServoReply {
  bool valid = false; // valid = recently updated, invalid = stale
  moteus::Query::Result result; // last updated result, or, previous result due to stale
  // moteus::Mode mode;  
  // moteus::PositionMode::Command result;
};

// using ServoCommandsMap = std::unordered_map<int, ServoCommand>;
// using ServoRepliesMap = std::unordered_map<int, ServoReply>;
// using DoubleBufferedServoCommands = DoubleBufferedObject< ServoCommandsMap >;
// using DoubleBufferedServoReplies = DoubleBufferedObject< ServoRepliesMap >;

using DoubleBufferedServoCommands = DoubleBufferedMap<int, ServoCommand>;
using DoubleBufferedServoReplies  = DoubleBufferedMap<int, ServoReply>;

struct Pi3HatMoteusData {

  // pi3hat::Span<ServoCommand> commands;
  DoubleBufferedServoCommands* commands = nullptr; // map from servo id to command

  // pi3hat::Span<ServoReply> replies;
  DoubleBufferedServoReplies* replies = nullptr; // map from servo id to reply

  std::shared_ptr<bool> timeout; // KY: copy from kodlab

 public:
  /** Major methods */
  explicit Pi3HatMoteusData( const size_t max_count ); 

  /**
   * @brief Convert servo commands to CAN FD frames.
   * 
   * @param moteus_controllers A vector of shared pointers to the moteus controllers.
   * @param frames A pointer to an external vector of CAN FD frames to store the command-can-frames by contents copying. If nullptr, the frames are not copied.
   * @return const size_t The number of CAN FD frames generated.
   */
  const size_t Commands2CanFdFrames ( 
    const std::vector< std::shared_ptr<moteus::Controller> > &moteus_controllers, 
    std::vector<moteus::CanFdFrame>* frames = nullptr 
  );

  /**
   * @brief Convert CAN FD frames to servo replies.
   * 
   * @param frames A pointer to an external vector of CAN FD frames to store the reply-can-frames by contents copying. If nullptr, the frames are not copied.
   * @return const size_t The number of CAN FD frames processed.
   */
  const size_t CanFdFrames2Replies ( std::vector<moteus::CanFdFrame>* frames = nullptr );

  const std::vector<moteus::CanFdFrame>& CanCommands() const { return _can_commands; }
  std::vector<moteus::CanFdFrame>& CanReplies() { return _can_replies; }


 private:
  size_t _max_count; // maximum number of servos to reserve
  std::vector<moteus::CanFdFrame> _can_commands; // CanFdFrames corresponding to the commands
  std::vector<moteus::CanFdFrame> _can_replies; // CanFdFrames corresponding to the replies
};

/**
 * @brief Pi3HatMoteusInterface is a wrapper class around pi3hat::Pi3HatMoteusTransport
 *        that provides an interface to communicate with Moteus servos over Pi3Hat CAN
 *        transport, with integration of mjbotscpp utilities.
 * 
 */
class Pi3HatMoteusInterface : public pi3hat::Pi3HatMoteusTransport {
 public: // data type
  using Options = pi3hat::Pi3HatMoteusTransport::Options;
 
 public:
  Pi3HatMoteusInterface( const Options &options);
  
  ~Pi3HatMoteusInterface();

  void Init( 
    Pi3HatMoteusData* data = nullptr, // if provided, also clear the data buffers
    const int clear_retries = 10, 
    const int retry_sleep_ms = 5 
  );

  void Cycle( 
    const std::vector<std::shared_ptr<moteus::Controller>> &moteus_controllers, 
    Pi3HatMoteusData &data, 
    moteus::CompletionCallback callback 
  ); 

  const double LostCommandRateAvg()   const { return _lost_command_rate.Avg(); }
  const double LostCommandRateWorst() const { return _lost_command_rate.Worst(); }
  const double LostReplyRateAvg()     const { return _lost_reply_rate.Avg(); }
  const double LostReplyRateWorst()   const { return _lost_reply_rate.Worst(); }

  const TimerMonitor& CycleTimer() const { return _cycle_timer; }
  
private:
  // diagnostics
  size_t _last_expect_count = 0;
  size_t _last_command_count = 0;
  size_t _last_reply_count = 0;
  RateMonitor _lost_command_rate;
  RateMonitor _lost_reply_rate;
  int _diag_throttle_cycles = 10; // update diagnostics every N cycles
  int _diag_cycle_counter = 0;

  // cycle timer
  TimerMonitor _cycle_timer;
  size_t _timer_cmd2frames = 0;
  size_t _timer_transport  = 0;
  size_t _timer_frames2replies = 0;
};
} // namespace mjbotscpp