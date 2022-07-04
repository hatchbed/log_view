/**
 * Copyright 2020 Hatchbed L.L.C.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are met:
 * 1. Redistributions of source code must retain the above copyright notice,
 *    this list of conditions and the following disclaimer.
 * 2. Redistributions in binary form must reproduce the above copyright notice,
 *    this list of conditions and the following disclaimer in the documentation
 *    and/or other materials provided with the distribution.
 * 3. Neither the name of the copyright holder nor the names of its contributors
 *    may be used to endorse or promote products derived from this software
 *    without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
 * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
 * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 * ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE
 * LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
 * CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
 * SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
 * INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
 * CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
 * ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 * POSSIBILITY OF SUCH DAMAGE.
 */

#ifndef LOG_VIEW_DATATYPES_H_
#define LOG_VIEW_DATATYPES_H_

#include <log_view/utils.h>
#include <rclcpp/rclcpp.hpp>
#include <rcl_interfaces/msg/log.hpp>

namespace log_view {

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
  bool exclude = true;
  size_t count = 0;
};

}  // namespace log_view

#endif  // LOG_VIEW_DATATYPES_H_
