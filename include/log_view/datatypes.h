// Copyright 2020 Hatchbed L.L.C.
//
// Redistribution and use in source and binary forms, with or without
// modification, are permitted provided that the following conditions are met:
//
//    * Redistributions of source code must retain the above copyright
//      notice, this list of conditions and the following disclaimer.
//
//    * Redistributions in binary form must reproduce the above copyright
//      notice, this list of conditions and the following disclaimer in the
//      documentation and/or other materials provided with the distribution.
//
//    * Neither the name of the copyright holder nor the names of its
//      contributors may be used to endorse or promote products derived from
//      this software without specific prior written permission.
//
// THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
// AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
// IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
// ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE
// LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
// CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
// SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
// INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
// CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
// ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
// POSSIBILITY OF SUCH DAMAGE.

#pragma once

#include <chrono>
#include <ctime>
#include <string>
#include <vector>

#include <log_view/utils.h>
#include <rclcpp/rclcpp.hpp>
#include <rcl_interfaces/msg/log.hpp>

namespace log_view {

static constexpr const char* kMarkerNode = "__log_view_marker__";

struct LogLine {
  size_t index;
  size_t line;
};

struct LogEntry
{
  LogEntry() = default;
  LogEntry(const LogEntry& entry) = default;
  explicit LogEntry(const rcl_interfaces::msg::Log& log) :
    stamp(log.stamp),
    level(log.level),
    node(log.name),
    file(log.file),
    function(log.function),
    line(log.line),
    text(split(log.msg, '\n'))
  {}

  rclcpp::Time stamp;
  uint8_t level;
  std::string node;
  std::string file;
  std::string function;
  uint32_t line;
  std::vector<std::string> text;
};

struct NodeData {
  bool selected = false;  // true = in whitelist (show when node filter is active)
  size_t count = 0;
};

inline LogEntry makeMarkerEntry(const std::string& label) {
  auto now = std::chrono::system_clock::now();
  std::time_t t = std::chrono::system_clock::to_time_t(now);
  struct tm tm_info;
  localtime_r(&t, &tm_info);
  char buf[32];
  strftime(buf, sizeof(buf), "%Y-%m-%d %H:%M:%S", &tm_info);

  int64_t ns = std::chrono::duration_cast<std::chrono::nanoseconds>(
    now.time_since_epoch()).count();

  LogEntry entry;
  entry.stamp = rclcpp::Time(ns, RCL_ROS_TIME);
  entry.level = rcl_interfaces::msg::Log::INFO;
  entry.node = kMarkerNode;
  entry.file = "";
  entry.function = "";
  entry.line = 0;
  entry.text = {"-------- " + label + " " + buf + " --------"};
  return entry;
}

}  // namespace log_view
