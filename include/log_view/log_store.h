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

#include <deque>
#include <memory>
#include <mutex>

#include <log_view/datatypes.h>
#include <log_view/log_writer.h>
#include <rcl_interfaces/msg/log.hpp>

namespace log_view {

class LogStore {
public:
  LogStore() = default;

  const std::deque<LogEntry>& logs();
  size_t size() const;

  // Returns the nanosecond timestamp of the oldest log entry, or -1 if empty.
  // Does not flush new_logs_ and has no side effects.
  int64_t firstStampNs() const;

  void addEntry(const rcl_interfaces::msg::Log::SharedPtr msg);
  void addEntry(const LogEntry& entry);
  void clear();

  void setWriter(LogWriter* writer);

private:
  std::deque<LogEntry> logs_;
  std::deque<LogEntry> new_logs_;

  mutable std::mutex mutex_;
  LogWriter* writer_ = nullptr;
};
typedef std::shared_ptr<LogStore> LogStorePtr;

}  // namespace log_view
