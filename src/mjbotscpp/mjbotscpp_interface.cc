#include <vector>
#include <memory>
#include <string>
#include <ostream>

#include "moteus.h"
#include "pi3hat_moteus_transport.h"

#include "mjbotscpp.h"
#include "mjbotscpp_interface.h"

using namespace mjbots;
// using Transport = pi3hat::Pi3HatMoteusTransport;

namespace mjbotscpp {

Pi3HatMoteusData::Pi3HatMoteusData( const size_t max_count )
: _max_count(max_count)
{
  // reserve can frames to avoid dynamical allocation
  this->_can_commands.reserve(this->_max_count);
  this->_can_replies.reserve(this->_max_count);
}

const size_t Pi3HatMoteusData::Commands2CanFdFrames( 
  const std::vector< std::shared_ptr<moteus::Controller> > &moteus_controllers, std::vector<moteus::CanFdFrame>* frames 
) {
  if ( this->commands == nullptr ) return 0;

  this->_can_commands.clear(); 
  const auto &cmds = this->commands->Read();
  
  for (auto &controller : moteus_controllers) {
    const auto id = controller->options().id;
    auto it = cmds.find(id);
    if ( it != cmds.end() ) {
      const ServoCommand &cmd = it->second;
      if (cmd.mode == moteus::Mode::kPosition) {
        const auto frame = controller->MakePosition( cmd.position );
        this->_can_commands.push_back(frame);
      } else if (cmd.mode == moteus::Mode::kStopped) {
        const auto frame = controller->MakeStop();
        this->_can_commands.push_back(frame);
      }
    }
  }

  // optional copy to frames
  if (frames != nullptr) {
    frames->assign(this->_can_commands.begin(), this->_can_commands.end());
  }

  return this->_can_commands.size();
}

const size_t Pi3HatMoteusData::CanFdFrames2Replies ( std::vector<moteus::CanFdFrame>* frames ) {
  if ( this->replies == nullptr ) return 0;

  size_t update_count = 0; 
  auto &replies_to_write = this->replies->BackBuffer(); // get buffer to write

  // mark all replies as stale initially
  for (auto &kv : replies_to_write) {
    kv.second.valid = false;
  }


  for ( const auto & frame : this->_can_replies ) {
    const int servo_id = frame.source;
    
    // if no corresponding reply, skip
    if( replies_to_write.find(servo_id) == replies_to_write.end() ) {
      continue;
    }

    replies_to_write[servo_id].valid = true;
    replies_to_write[servo_id].result = moteus::Query::Parse(frame.data, frame.size);
    update_count++; 
  }

  this->replies->Publish(); // publish the back buffer as the new front

  if (frames != nullptr) {
    frames->assign(this->_can_replies.begin(), this->_can_replies.end());
  }

  return update_count;
}


Pi3HatMoteusInterface::Pi3HatMoteusInterface( const Options &options)
: pi3hat::Pi3HatMoteusTransport(options)
{

}

Pi3HatMoteusInterface::~Pi3HatMoteusInterface()
{

}

void Pi3HatMoteusInterface::Cycle( 
  const std::vector<std::shared_ptr<moteus::Controller>> &moteus_controllers, 
  Pi3HatMoteusData &data, 
  moteus::CompletionCallback callback 
)
{
  // convert commands to can-frames  
  this->_last_expect_count = moteus_controllers.size();
  this->_last_command_count = data.Commands2CanFdFrames( moteus_controllers );
  
  pi3hat::Pi3HatMoteusTransport::Cycle(
    data.CanCommands().data(), 
    data.CanCommands().size(), 
    &(data.CanReplies()), 
    nullptr, 
    nullptr, 
    nullptr, 
    callback
  );

  this->_last_reply_count = data.CanFdFrames2Replies();
  
  // diagnostics
  const double lost_cmd_rate = (_last_command_count < _last_expect_count)
      ? static_cast<double>(_last_expect_count - _last_command_count) / static_cast<double>(_last_expect_count)
      : 0.0;
  const double lost_reply_rate = (_last_reply_count < _last_command_count)
      ? static_cast<double>(_last_command_count - _last_reply_count) / static_cast<double>(_last_command_count)
      : 0.0;
  _lost_command_rate.Update(lost_cmd_rate);
  _lost_reply_rate.Update(lost_reply_rate);
}

void Pi3HatMoteusInterface::Init( Pi3HatMoteusData* data, const int clear_retries, const int retry_sleep_ms )
{
  // clear stale replies in bus 1 to 4
  std::vector<moteus::CanFdFrame> replies; 
  std::vector<moteus::CanFdFrame> *replies_ptr = nullptr; 
  if (data != nullptr) {
    data->CanReplies().clear();
    replies_ptr = &(data->CanReplies());
  } else {
    replies_ptr = &replies;
  }
  pi3hat::Pi3Hat::Input input_override;
  input_override.force_can_check = (1 << 1) | (1 << 2) | (1 << 3) | (1 << 4);

  for (int i = 0; i < clear_retries; i++) {
    replies.clear();
    moteus::BlockingCallback cbk;
    pi3hat::Pi3HatMoteusTransport::Cycle(
      nullptr, 
      0, 
      replies_ptr, 
      nullptr, 
      nullptr,
      &input_override, 
      cbk.callback()
    );
    cbk.Wait();
    std::this_thread::sleep_for(std::chrono::milliseconds(retry_sleep_ms));
  }
}


} // namespace mjbotscpp