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

#include <set>
#include <string>

namespace log_view {

struct Preferences {
  enum class TimestampFormat {
    SECONDS,      // raw ROS seconds since epoch (e.g. 1234567.8901)
    ELAPSED,      // seconds since first message
    TIME_OF_DAY,  // local wall clock HH:MM:SS.mmm
  };

  TimestampFormat timestamp_format = TimestampFormat::SECONDS;
  bool persist_filters = false;
  bool persist_logs = false;
  size_t log_rotate_size = 10 * 1024 * 1024;   // 10 MB
  size_t log_max_size    = 100 * 1024 * 1024;   // 100 MB

  struct FilterSettings {
    bool debug = true;
    bool info = true;
    bool warn = true;
    bool error = true;
    bool fatal = true;
    bool node_filter_enabled = false;
    std::string filter_pattern;
    std::string exclude_pattern;
    std::set<std::string> node_whitelist;  // nodes to show when node filter is enabled
  } filters;

  static std::string defaultPath();
  bool load();
  void save() const;
};

}  // namespace log_view
