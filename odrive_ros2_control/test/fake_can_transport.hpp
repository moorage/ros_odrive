#pragma once

#include <deque>
#include <functional>
#include <vector>

#include "odrive_ros2_control/odrive_system.hpp"

class FakeCanTransport final : public odrive_ros2_control::CanTransport {
public:
  bool init(const std::string &, std::function<void(const can_frame &)> cb) override {
    callback_ = std::move(cb);
    return true;
  }

  void shutdown() override { }

  bool send(const can_frame &frame) override {
    sent_frames.push_back(frame);
    return true;
  }

  void poll() override {
    while (!rx_frames.empty()) {
      const auto frame = rx_frames.front();
      rx_frames.pop_front();
      if (callback_) callback_(frame);
    }
  }

  void push_rx(const can_frame &frame) { rx_frames.push_back(frame); }

  std::vector<can_frame> sent_frames;

private:
  std::function<void(const can_frame &)> callback_;
  std::deque<can_frame> rx_frames;
};
