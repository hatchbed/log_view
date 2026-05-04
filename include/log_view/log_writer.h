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

#include <atomic>
#include <condition_variable>
#include <deque>
#include <fstream>
#include <mutex>
#include <string>
#include <thread>
#include <vector>

#include <log_view/datatypes.h>

namespace log_view {

// Persists log entries to disk in a background thread with file rotation.
// Files are stored under log_dir as log_view_YYYYMMDD_HHMMSS.log.
// A new file is opened when the current file exceeds rotate_bytes.
// Old files are pruned when total disk usage exceeds max_total_bytes.
class LogWriter {
public:
  LogWriter(const std::string& log_dir, size_t rotate_bytes, size_t max_total_bytes);
  ~LogWriter();

  // Start the background writer thread.
  void start();

  // Drain the queue, flush, and join the writer thread.
  void stop();

  // Thread-safe: enqueue an entry for writing. Called from the ROS callback thread.
  void enqueue(const LogEntry& entry);

  // Signal the writer to delete all log files and start a fresh file.
  // Called from the UI thread after the log store has been cleared.
  void requestClear();

  // Returns the default log directory: <cwd>/.log_view
  static std::string defaultDir();

  // Load all persisted log entries in chronological order. Called during startup.
  std::vector<LogEntry> loadAll() const;

private:
  void writerLoop();
  void openNewFile();
  void rotateIfNeeded();
  void writeEntry(const LogEntry& entry);
  void deleteAllFiles();
  void pruneToMaxSize();

  std::vector<std::string> findLogFiles() const;
  std::string generateFilePath() const;

  std::string log_dir_;
  size_t rotate_bytes_;
  size_t max_total_bytes_;

  std::ofstream file_;
  size_t current_file_size_ = 0;

  std::deque<LogEntry> queue_;
  std::mutex queue_mutex_;
  std::condition_variable cv_;
  std::atomic<bool> running_{false};
  std::atomic<bool> clear_requested_{false};
  std::thread thread_;
};

}  // namespace log_view
