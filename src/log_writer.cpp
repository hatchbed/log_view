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

#include <log_view/log_writer.h>

#include <algorithm>
#include <chrono>
#include <ctime>
#include <filesystem>
#include <fstream>
#include <stdexcept>
#include <string>

#include <log_view/preferences.h>
#include <log_view/utils.h>

namespace log_view {

// File format (two lines per record):
//   stamp_ns|level|node|file|func|source_line|msg_len\n
//   <msg_len bytes of raw message text>\n
//
// stamp_ns, level, source_line, msg_len are always numeric (no '|' possible).
// node and file never contain '|' by their respective naming rules.
// func is taken as everything between the second-from-right '|' and the
// right-most '|', so operator| in a function signature is preserved correctly.

LogWriter::LogWriter(const std::string& log_dir, size_t rotate_bytes, size_t max_total_bytes)
: log_dir_(log_dir), rotate_bytes_(rotate_bytes), max_total_bytes_(max_total_bytes) {}

LogWriter::~LogWriter() {
  stop();
}

void LogWriter::start() {
  if (log_dir_.empty()) return;
  running_ = true;
  thread_ = std::thread(&LogWriter::writerLoop, this);
}

void LogWriter::stop() {
  if (!running_ && !thread_.joinable()) return;
  running_ = false;
  cv_.notify_all();
  if (thread_.joinable()) {
    thread_.join();
  }
}

void LogWriter::enqueue(const LogEntry& entry) {
  if (!running_) return;
  {
    std::lock_guard<std::mutex> lock(queue_mutex_);
    queue_.push_back(entry);
  }
  cv_.notify_one();
}

void LogWriter::requestClear() {
  clear_requested_ = true;
  cv_.notify_one();
}

std::string LogWriter::defaultDir() {
  return Preferences::workspaceDataDir();
}

std::vector<LogEntry> LogWriter::loadAll() const {
  std::vector<LogEntry> result;
  if (log_dir_.empty() || !std::filesystem::is_directory(log_dir_)) return result;
  auto files = findLogFiles();

  for (const auto& path : files) {
    std::ifstream in(path);
    if (!in) continue;

    std::string header_line;
    while (std::getline(in, header_line)) {
      // Parse left anchor: first two '|' give stamp_ns and level.
      size_t p0 = header_line.find('|');
      if (p0 == std::string::npos) continue;
      size_t p1 = header_line.find('|', p0 + 1);
      if (p1 == std::string::npos) continue;

      // Parse right anchor: last two '|' give msg_len and source_line.
      size_t p5 = header_line.rfind('|');
      if (p5 == std::string::npos || p5 <= p1) continue;
      size_t p4 = header_line.rfind('|', p5 - 1);
      if (p4 == std::string::npos || p4 <= p1) continue;

      // Middle section between p1 and p4 is "node|file|func".
      // node and file never contain '|', so we take them left-to-right.
      // func is everything remaining (handles operator| correctly).
      std::string middle = header_line.substr(p1 + 1, p4 - p1 - 1);
      size_t m0 = middle.find('|');
      if (m0 == std::string::npos) continue;
      size_t m1 = middle.find('|', m0 + 1);
      if (m1 == std::string::npos) continue;

      int64_t stamp_ns = 0;
      uint8_t level = 0;
      uint32_t source_line = 0;
      size_t msg_len = 0;

      try {
        stamp_ns    = std::stoll(header_line.substr(0, p0));
        level       = static_cast<uint8_t>(std::stoul(header_line.substr(p0 + 1, p1 - p0 - 1)));
        source_line = static_cast<uint32_t>(std::stoul(header_line.substr(p4 + 1, p5 - p4 - 1)));
        msg_len     = static_cast<size_t>(std::stoull(header_line.substr(p5 + 1)));
      } catch (const std::exception&) {
        if (msg_len > 0) {
          in.ignore(static_cast<std::streamsize>(msg_len + 1));
        }
        continue;
      }

      std::string msg(msg_len, '\0');
      if (msg_len > 0 && !in.read(msg.data(), static_cast<std::streamsize>(msg_len))) break;
      in.ignore(1);  // skip the trailing newline after the message body

      LogEntry entry;
      entry.stamp    = rclcpp::Time(stamp_ns, RCL_ROS_TIME);
      entry.level    = level;
      entry.line     = source_line;
      entry.node     = middle.substr(0, m0);
      entry.file     = middle.substr(m0 + 1, m1 - m0 - 1);
      entry.function = middle.substr(m1 + 1);
      entry.text     = split(msg, '\n');
      if (entry.text.empty()) {
        entry.text.push_back("");
      }

      result.push_back(std::move(entry));
    }
  }

  return result;
}

void LogWriter::writerLoop() {
  openNewFile();

  while (true) {
    std::deque<LogEntry> batch;
    bool should_clear = false;
    bool should_exit = false;

    {
      std::unique_lock<std::mutex> lock(queue_mutex_);
      cv_.wait(lock, [this] {
        return !queue_.empty() || !running_ || clear_requested_;
      });

      should_clear = clear_requested_.exchange(false);
      if (should_clear) {
        queue_.clear();
      } else {
        std::swap(batch, queue_);
        should_exit = !running_;
      }
    }

    if (should_clear) {
      deleteAllFiles();
    } else {
      for (const auto& entry : batch) {
        writeEntry(entry);
        rotateIfNeeded();
      }
    }

    if (should_exit) break;
  }

  if (file_.is_open()) {
    file_.flush();
    file_.close();
  }
}

void LogWriter::openNewFile() {
  if (file_.is_open()) {
    file_.flush();
    file_.close();
  }
  current_file_size_ = 0;

  try {
    std::filesystem::create_directories(log_dir_);
  } catch (...) {
    return;
  }

  std::string path = generateFilePath();
  file_.open(path, std::ios::app);
  if (file_.is_open()) {
    try {
      current_file_size_ = static_cast<size_t>(std::filesystem::file_size(path));
    } catch (...) {
      current_file_size_ = 0;
    }
  }

  pruneToMaxSize();
}

void LogWriter::rotateIfNeeded() {
  if (current_file_size_ >= rotate_bytes_) {
    openNewFile();
  }
}

void LogWriter::writeEntry(const LogEntry& entry) {
  if (!file_.is_open()) return;

  std::string msg;
  for (size_t i = 0; i < entry.text.size(); ++i) {
    if (i > 0) msg += '\n';
    msg += entry.text[i];
  }

  // header: stamp_ns|level|node|file|func|source_line|msg_len
  std::string header =
    std::to_string(entry.stamp.nanoseconds()) + "|" +
    std::to_string(static_cast<unsigned>(entry.level)) + "|" +
    entry.node + "|" +
    entry.file + "|" +
    entry.function + "|" +
    std::to_string(entry.line) + "|" +
    std::to_string(msg.size()) + "\n";

  file_.write(header.data(), static_cast<std::streamsize>(header.size()));
  file_.write(msg.data(), static_cast<std::streamsize>(msg.size()));
  file_.put('\n');

  current_file_size_ += header.size() + msg.size() + 1;
}

void LogWriter::deleteAllFiles() {
  if (file_.is_open()) {
    file_.close();
  }
  current_file_size_ = 0;

  for (const auto& path : findLogFiles()) {
    try { std::filesystem::remove(path); } catch (...) {}
  }

  openNewFile();
}

void LogWriter::pruneToMaxSize() {
  auto files = findLogFiles();
  if (files.empty()) return;

  size_t total = 0;
  for (const auto& f : files) {
    try { total += static_cast<size_t>(std::filesystem::file_size(f)); } catch (...) {}
  }

  for (size_t i = 0; i < files.size() && total > max_total_bytes_; ++i) {
    size_t sz = 0;
    try { sz = static_cast<size_t>(std::filesystem::file_size(files[i])); } catch (...) {}
    try { std::filesystem::remove(files[i]); } catch (...) {}
    total -= sz;
  }
}

std::vector<std::string> LogWriter::findLogFiles() const {
  std::vector<std::string> files;
  if (log_dir_.empty() || !std::filesystem::is_directory(log_dir_)) return files;

  try {
  for (const auto& entry : std::filesystem::directory_iterator(log_dir_)) {
    const auto& p = entry.path();
    std::string fname = p.filename().string();
    if (p.extension() == ".log" && fname.substr(0, 9) == "log_view_") {
      files.push_back(p.string());
    }
  }
  } catch (...) {}
  std::sort(files.begin(), files.end());
  return files;
}

std::string LogWriter::generateFilePath() const {
  auto now = std::chrono::system_clock::now();
  std::time_t t = std::chrono::system_clock::to_time_t(now);
  struct tm tm_info;
  localtime_r(&t, &tm_info);
  char buf[64];
  strftime(buf, sizeof(buf), "log_view_%Y%m%d_%H%M%S", &tm_info);

  std::string base = log_dir_ + "/" + buf;
  std::string path = base + ".log";
  int counter = 1;
  while (std::filesystem::exists(path)) {
    path = base + "_" + std::to_string(counter++) + ".log";
  }
  return path;
}

}  // namespace log_view
