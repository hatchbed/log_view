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

#include <log_view/log_filter.h>

#include <log_view/utils.h>

namespace log_view {

LogFilter::LogFilter(LogStorePtr& logs) :  logs_(logs)
{

}

void LogFilter::setFilter(const std::string& filter) {
  filter_string_ = filter;
  auto filter_list = split(filter, ';');
  bool changed = filter_list.size() != filter_list_.size();

  for (size_t i = 0; i < filter_list_.size() && !changed; i++) {
    if (filter_list_[i] != filter_list[i]) {
      changed = true;
    }
  }

  filter_list_ = filter_list;

  if (changed) {
    reset();
  }
}

void LogFilter::setExclude(const std::string& exclude) {
  auto exclude_list = split(exclude, ';');
  bool changed = exclude_list.size() != exclude_list_.size();

  for (size_t i = 0; i < exclude_list_.size() && !changed; i++) {
    if (exclude_list_[i] != exclude_list[i]) {
      changed = true;
    }
  }

  exclude_list_ = exclude_list;

  if (changed) {
    reset();
  }
}

void LogFilter::setDebugLevel(bool enable) {
  if (debug_level_ != enable) {
    debug_level_ = enable;
    reset();
  }
}

void LogFilter::setInfoLevel(bool enable) {
  if (info_level_ != enable) {
    info_level_ = enable;
    reset();
  }
}

void LogFilter::setWarnLevel(bool enable) {
  if (warn_level_ != enable) {
    warn_level_ = enable;
    reset();
  }
}

void LogFilter::setErrorLevel(bool enable) {
  if (error_level_ != enable) {
    error_level_ = enable;
    reset();
  }
}

void LogFilter::setFatalLevel(bool enable) {
  if (fatal_level_ != enable) {
    fatal_level_ = enable;
    reset();
  }
}

void LogFilter::setEnableNodeFilter(bool enable) {
  if (filter_nodes_ != enable) {
    filter_nodes_ = enable;
    reset();
  }
}

void LogFilter::toggleNode(const std::string& node) {
  auto element = nodes_.find(node);
  if (element != nodes_.end()) {
    element->second.exclude = !element->second.exclude;

    filter_nodes_ = true;
    reset();
  }
}

void LogFilter::selectAllNodes() {
  bool filter_nodes = filter_nodes_;
  for (auto& elem: nodes_) {
    elem.second.exclude = false;
  }

  filter_nodes_ = true;
  reset();
}

void LogFilter::invertNodes() {
  filter_nodes_ = true;
  for (auto& elem: nodes_) {
    elem.second.exclude = !elem.second.exclude;
  }

  reset();
}


void LogFilter::reset() {
  log_indices_.clear();
  cursor_ = -1;
  clearSelect();
  latest_log_index_ = logs_->size();
  earliest_log_index_ = latest_log_index_;
  if (earliest_log_index_ > 0) {
    earliest_log_index_--;
  }
}

void LogFilter::update() {
  const auto& logs = logs_->logs();

  // TODO process for 50 ms to avoid hogging the screen if there is a backlog

  for (;latest_log_index_ < logs.size(); latest_log_index_++) {
    if (accepted(logs[latest_log_index_], true)) {
      for (size_t i = 0; i < logs[latest_log_index_].text.size(); i++) {
        log_indices_.push_back({latest_log_index_, i});
      }
    }
  }
}

void LogFilter::idleProcess() {
  const auto& logs = logs_->logs();

  // TODO process for 50 ms instead of fixed 1000

  for (size_t i = 0; earliest_log_index_ != 0 && i < 1000; earliest_log_index_--, i++)
  {
    if (accepted(logs[earliest_log_index_])) {
      size_t lines = logs[earliest_log_index_].text.size();
      for (size_t j = 1; j <= lines; j++) {
        log_indices_.push_front({earliest_log_index_, lines - j});
        if (cursor_ >= 0) {
          cursor_++;
        }
        if (select_start_ >= 0) {
          select_end_++;
          select_start_++;
        }
        if (search_cursor_ >= 0) {
          search_cursor_++;
        }
        if (search_cursor_fwd_ >= 0) {
          search_cursor_fwd_++;
          search_cursor_rev_++;
        }
      }
    }
  }

  if (search_cursor_ == -1 && !search_.empty() && !log_indices_.empty()) {
    if ((search_direction_ == SEARCH_BOTH || search_direction_ == SEARCH_FWD) && search_cursor_fwd_ >= 0) {
      size_t max_idx = search_cursor_fwd_ + 1000;
      for (size_t i = search_cursor_fwd_; i < max_idx && i < log_indices_.size(); i++) {
        auto& index = log_indices_[i];
        search_cursor_fwd_ = i + 1;
        if (contains(logs[index.index].text[index.line], search_, true)) {
          search_cursor_ = i;
          cursor_ = i + cursor_offset_;
          break;
        }
      }
    }

    if (search_cursor_ == -1 && (search_direction_ == SEARCH_BOTH || search_direction_ == SEARCH_REV) && search_cursor_rev_ >= 0) {
      int64_t min_idx = search_cursor_rev_ - 1000;
      for (int64_t i = search_cursor_rev_; i > min_idx && i >= 0; i--) {
        auto& index = log_indices_[i];
        search_cursor_rev_ = std::max(static_cast<int64_t>(0), i - 1);
        if (contains(logs[index.index].text[index.line], search_, true)) {
          search_cursor_ = i;
          cursor_ = i + 1;
          break;
        }
      }
    }
  }
}

void LogFilter::setCursor(int64_t index) {
  cursor_ = index;
}

size_t LogFilter::getCursor() {
  return cursor_;
}

void LogFilter::clearSelect() {
  select_start_ = -1;
  select_end_ = -1;
}

void LogFilter::setSelectStart(int64_t index) {
  select_start_ = index;
  select_end_ = index;
}

int64_t LogFilter::getSelectStart() {
  return select_start_;
}

void LogFilter::setSelectEnd(int64_t index) {
  select_end_ = index;
}

int64_t LogFilter::getSelectEnd() {
  return select_end_;
}

void LogFilter::search(const std::string& pattern) {
  search_ = pattern;

  search_direction_ = SEARCH_BOTH;
  search_cursor_ = -1;

  int64_t cursor = cursor_;
  if (cursor < 0) {
    cursor = log_indices_.size();
    cursor--;
  }

  search_cursor_fwd_ = cursor;
  search_cursor_rev_ = cursor;
}

void LogFilter::nextMatch() {
  search_direction_ = SEARCH_FWD;
  search_cursor_ = -1;

  int64_t cursor = cursor_;
  if (cursor < 0) {
    cursor = log_indices_.size();
    cursor--;
  }

  search_cursor_fwd_ = cursor + 1;
}

void LogFilter::prevMatch() {
  search_direction_ = SEARCH_REV;
  search_cursor_ = -1;

  int64_t cursor = cursor_;
  if (cursor < 0) {
    cursor = log_indices_.size();
    cursor--;
  }

  search_cursor_rev_ = std::max(static_cast<int64_t>(0), cursor - (cursor_offset_ + 1));
}

void LogFilter::clearSearch() {
  search_.clear();
  search_cursor_ = -1;
  search_cursor_fwd_ = -1;
  search_cursor_rev_ = -1;
}

bool LogFilter::accepted(const LogEntry& entry, bool new_entry) {
  bool include = filter_list_.empty();

  auto node = nodes_.find(entry.node);
  if (node == nodes_.end()) {
    nodes_[entry.node].exclude = true;
    nodes_[entry.node].count = 1;
  }
  else if (new_entry) {
    node->second.count++;
  }

  if (entry.level == rcl_interfaces::msg::Log::DEBUG) {
    if (!debug_level_) {
      return false;
    }
  }
  else if (entry.level == rcl_interfaces::msg::Log::INFO) {
    if (!info_level_) {
      return false;
    }
  }
  else if (entry.level == rcl_interfaces::msg::Log::WARN) {
    if (!warn_level_) {
      return false;
    }
  }
  else if (entry.level == rcl_interfaces::msg::Log::ERROR) {
    if (!error_level_) {
      return false;
    }
  }
  else if (entry.level == rcl_interfaces::msg::Log::FATAL) {
    if (!fatal_level_) {
      return false;
    }
  }

  if (filter_nodes_ && nodes_[entry.node].exclude) {
    return false;
  }

  for (const auto& filter: filter_list_) {
    for (const auto& line: entry.text) {
      if (contains(line, filter, true)) {
        include = true;
        break;
      }
    }
  }

  if (include) {
    for (const auto& exclude: exclude_list_) {
      for (const auto& line: entry.text) {
        if (contains(line, exclude, true)) {
          include = false;
          break;
        }
      }
    }
  }

  return include;
}


}  // namespace log_view
