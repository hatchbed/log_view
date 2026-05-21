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

#include <log_view/log_store.h>

namespace log_view {

const std::deque<LogEntry>& LogStore::logs() {
  std::lock_guard<std::mutex> lock(mutex_);
  for (const auto& entry : new_logs_) {
    logs_.push_back(entry);
  }
  new_logs_.clear();
  return logs_;
}

size_t LogStore::size() const {
  std::lock_guard<std::mutex> lock(mutex_);
  return logs_.size();
}

int64_t LogStore::firstStampNs() const {
  std::lock_guard<std::mutex> lock(mutex_);
  if (!logs_.empty()) {
    return static_cast<int64_t>(logs_.front().stamp.toNSec());
  }
  if (!new_logs_.empty()) {
    return static_cast<int64_t>(new_logs_.front().stamp.toNSec());
  }
  return -1;
}

void LogStore::addEntry(const rosgraph_msgs::LogConstPtr& msg) {
  std::lock_guard<std::mutex> lock(mutex_);
  new_logs_.push_back(LogEntry(*msg));
  if (writer_) {
    writer_->enqueue(new_logs_.back());
  }
}

void LogStore::addEntry(const LogEntry& entry) {
  std::lock_guard<std::mutex> lock(mutex_);
  new_logs_.push_back(entry);
}

void LogStore::clear() {
  std::lock_guard<std::mutex> lock(mutex_);
  logs_.clear();
  new_logs_.clear();
  if (writer_) {
    writer_->requestClear();
  }
}

void LogStore::setWriter(LogWriter* writer) {
  std::lock_guard<std::mutex> lock(mutex_);
  writer_ = writer;
}

}  // namespace log_view
