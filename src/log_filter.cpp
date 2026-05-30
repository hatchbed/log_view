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

#include <log_view/log_filter.h>

#include <log_view/utils.h>

namespace log_view {

LogFilter::LogFilter(const LogStorePtr& logs) :
  logs_(logs)
{}

void LogFilter::updatePatternList(
    const std::string& raw,
    std::string& stored_string,
    std::vector<Pattern>& stored_patterns)
{
  auto tokens = split(raw, ';');
  bool changed = (tokens.size() != stored_patterns.size());
  if (!changed) {
    for (size_t i = 0; i < tokens.size(); i++) {
      if (tokens[i] != stored_patterns[i].raw) { changed = true; break; }
    }
  }
  stored_string = raw;
  stored_patterns.clear();
  for (const auto& token : tokens) {
    stored_patterns.push_back(Pattern::compile(token));
  }
  if (changed) {
    reset();
  }
}

void LogFilter::setFilter(const std::string& filter) {
  updatePatternList(filter, filter_string_, filter_patterns_);
}

void LogFilter::setExclude(const std::string& exclude) {
  updatePatternList(exclude, exclude_string_, exclude_patterns_);
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

void LogFilter::setShowSessionBoundaries(bool enable) {
  if (show_session_boundaries_ != enable) {
    show_session_boundaries_ = enable;
    reset();
  }
}

void LogFilter::toggleNode(const std::string& node) {
  auto element = nodes_.find(node);
  if (element != nodes_.end()) {
    element->second.selected = !element->second.selected;
    if (element->second.selected) {
      selected_node_count_++;
    } else if (selected_node_count_ > 0) {
      selected_node_count_--;
    }

    filter_nodes_ = true;
    reset();
  }
}

void LogFilter::selectAllNodes() {
  for (auto& elem : nodes_) {
    elem.second.selected = true;
  }
  selected_node_count_ = nodes_.size();

  filter_nodes_ = true;
  reset();
}

void LogFilter::invertNodes() {
  filter_nodes_ = true;
  selected_node_count_ = 0;
  for (auto& elem : nodes_) {
    elem.second.selected = !elem.second.selected;
    if (elem.second.selected) {
      selected_node_count_++;
    }
  }

  reset();
}

void LogFilter::setNodeWhitelist(const std::set<std::string>& whitelist) {
  for (const auto& name : whitelist) {
    if (!nodes_.count(name)) {
      nodes_[name] = {true, 0};
      selected_node_count_++;
    }
  }
}

void LogFilter::setBagSources(const std::vector<std::string>& paths) {
  bag_paths_ = paths;
  bag_sources_.clear();
  bag_sources_.resize(paths.size(), {true, 0});
  deselected_bag_count_ = 0;
}

void LogFilter::toggleBagSource(int idx) {
  if (idx < 0 || idx >= static_cast<int>(bag_sources_.size())) {
    return;
  }
  auto& src = bag_sources_[idx];
  src.selected = !src.selected;
  if (src.selected) {
    if (deselected_bag_count_ > 0) { deselected_bag_count_--; }
  } else {
    deselected_bag_count_++;
  }
  reset();
}

void LogFilter::selectAllBagSources() {
  for (auto& src : bag_sources_) {
    src.selected = true;
  }
  deselected_bag_count_ = 0;
  reset();
}

void LogFilter::invertBagSources() {
  deselected_bag_count_ = 0;
  for (auto& src : bag_sources_) {
    src.selected = !src.selected;
    if (!src.selected) { deselected_bag_count_++; }
  }
  reset();
}


void LogFilter::clearLogs() {
  for (auto it = nodes_.begin(); it != nodes_.end(); ) {
    if (it->second.selected) {
      it->second.count = 0;
      ++it;
    } else {
      it = nodes_.erase(it);
    }
  }
  for (auto& src : bag_sources_) {
    src.count = 0;
  }
  logs_->clear();
  clearSearch();
  log_indices_.clear();
  cursor_ = -1;
  clearSelect();
  latest_log_index_ = 0;
  earliest_log_index_ = 0;
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
  search_cursor_       = -1;
  search_cursor_fwd_   = 0;
  search_cursor_rev_   = -1;
  search_cursor_saved_ = -1;
  if (!search_pattern_.empty()) {
    search_direction_ = SearchDirection::SEARCH_BOTH;
  }
}

void LogFilter::update() {
  const auto& logs = logs_->logs();

  // TODO(malban): process for 50 ms to avoid hogging the screen if there is a backlog

  bool marker_added = false;
  for (;latest_log_index_ < logs.size(); latest_log_index_++) {
    if (accepted(logs[latest_log_index_], true)) {
      if (logs[latest_log_index_].node == kMarkerNode) {
        marker_added = true;
      }
      for (size_t i = 0; i < logs[latest_log_index_].text.size(); i++) {
        log_indices_.push_back({latest_log_index_, i});
      }
    }
  }
  if (marker_added) {
    cleanSessionBoundaries();
  }
}

void LogFilter::idleProcess() {
  const auto& logs = logs_->logs();

  // TODO(malban): process for 50 ms instead of fixed 1000

  bool marker_added = false;
  for (size_t i = 0; earliest_log_index_ != 0 && i < 1000; earliest_log_index_--, i++)
  {
    if (accepted(logs[earliest_log_index_])) {
      if (logs[earliest_log_index_].node == kMarkerNode) {
        marker_added = true;
      }
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
  if (marker_added) {
    cleanSessionBoundaries();
  }

  if (search_cursor_ == -1 && search_direction_ == SearchDirection::SEARCH_BOTH &&
      !search_pattern_.empty() && !log_indices_.empty()) {
    if (search_cursor_fwd_ >= 0) {
      size_t max_idx = search_cursor_fwd_ + 1000;
      for (size_t i = search_cursor_fwd_; i < max_idx && i < log_indices_.size(); i++) {
        auto& index = log_indices_[i];
        search_cursor_fwd_ = i + 1;
        if (search_pattern_.matches(logs[index.index].text[index.line])) {
          search_cursor_ = i;
          cursor_ = i + cursor_offset_;
          break;
        }
      }
    }

    if (search_cursor_ == -1 && search_cursor_rev_ >= 0) {
      int64_t min_idx = search_cursor_rev_ - 1000;
      for (int64_t i = search_cursor_rev_; i > min_idx && i >= 0; i--) {
        auto& index = log_indices_[i];
        search_cursor_rev_ = std::max(static_cast<int64_t>(0), i - 1);
        if (search_pattern_.matches(logs[index.index].text[index.line])) {
          search_cursor_ = i;
          cursor_ = i + cursor_offset_;
          break;
        }
      }
    }
  }
}

bool LogFilter::scanMatchForward(int64_t from_idx, bool minimal_scroll) {
  if (from_idx < 0) from_idx = 0;
  const auto& logs = logs_->logs();
  for (int64_t i = from_idx; i < static_cast<int64_t>(log_indices_.size()); i++) {
    const auto& ll = log_indices_[static_cast<size_t>(i)];
    if (ll.index >= logs.size()) continue;
    if (search_pattern_.matches(logs[ll.index].text[ll.line])) {
      search_cursor_ = i;
      if (minimal_scroll) {
        if (i >= cursor_) {
          cursor_ = i + 1;
        } else if (i < cursor_ - cursor_offset_) {
          cursor_ = i + cursor_offset_;
        }
      } else {
        cursor_ = i + cursor_offset_;
      }
      return true;
    }
  }
  return false;
}

bool LogFilter::scanMatchBackward(int64_t from_idx, bool minimal_scroll) {
  const auto& logs = logs_->logs();
  for (int64_t i = from_idx; i >= 0; i--) {
    const auto& ll = log_indices_[static_cast<size_t>(i)];
    if (ll.index >= logs.size()) continue;
    if (search_pattern_.matches(logs[ll.index].text[ll.line])) {
      search_cursor_ = i;
      if (minimal_scroll) {
        if (i >= cursor_) {
          cursor_ = i + 1;
        } else if (i < cursor_ - cursor_offset_) {
          cursor_ = i + cursor_offset_;
        }
      } else {
        cursor_ = i + 1;
      }
      return true;
    }
  }
  return false;
}

void LogFilter::nextMatchByMatch() {
  if (search_pattern_.empty() || log_indices_.empty()) return;
  int64_t start = (search_cursor_ >= 0) ? search_cursor_ + 1 : 0;
  scanMatchForward(start, true);
}

void LogFilter::prevMatchByMatch() {
  if (search_pattern_.empty() || log_indices_.empty()) return;
  int64_t start = (search_cursor_ >= 0)
                  ? search_cursor_ - 1
                  : static_cast<int64_t>(log_indices_.size()) - 1;
  scanMatchBackward(start, true);
}

void LogFilter::setCursor(int64_t index) {
  cursor_ = index;
}

int64_t LogFilter::getCursor() const {
  return cursor_;
}

const LogEntry& LogFilter::getEntry(int64_t index) const {
  if (index < 0 || index >= log_indices_.size()) {
    return dummy_entry_;
  }

  auto& entry = log_indices_[index];
  auto& all_logs = logs_->logs();
  if (entry.index >= all_logs.size()) {
    return dummy_entry_;
  }

  return all_logs[entry.index];
}

void LogFilter::clearSelect() {
  select_start_ = -1;
  select_end_ = -1;
}

void LogFilter::setSelectStart(int64_t index) {
  select_start_ = index;
  select_end_ = index;
}

int64_t LogFilter::getSelectStart() const {
  return select_start_;
}

void LogFilter::setSelectEnd(int64_t index) {
  select_end_ = index;
}

int64_t LogFilter::getSelectEnd() const {
  return select_end_;
}

void LogFilter::search(const std::string& pattern) {
  search_cursor_saved_ = -1;
  search_pattern_ = Pattern::compile(pattern);

  search_direction_ = SearchDirection::SEARCH_BOTH;
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
  if (search_pattern_.empty() || log_indices_.empty()) return;
  int64_t cursor = (cursor_ >= 0) ? cursor_ : static_cast<int64_t>(log_indices_.size());
  if (!scanMatchForward(cursor + 1, false)) {
    if (search_cursor_ >= 0) {
      search_cursor_saved_ = search_cursor_;
      if (!scanMatchForward(search_cursor_saved_ + 1, true)) {
        int64_t ii = search_cursor_;
        if (ii >= cursor_ || ii < cursor_ - cursor_offset_) {
          cursor_ = (ii >= cursor_) ? ii + 1 : ii + cursor_offset_;
        }
      }
    }
  }
}

void LogFilter::prevMatch() {
  if (search_pattern_.empty() || log_indices_.empty()) return;
  int64_t cursor = (cursor_ >= 0) ? cursor_ : static_cast<int64_t>(log_indices_.size());
  int64_t page_start = cursor - cursor_offset_ - 1;
  if (!scanMatchBackward(page_start, false)) {
    if (search_cursor_ >= 0) {
      search_cursor_saved_ = search_cursor_;
      if (!scanMatchBackward(search_cursor_saved_ - 1, true)) {
        int64_t ii = search_cursor_;
        if (ii >= cursor_ || ii < cursor_ - cursor_offset_) {
          cursor_ = (ii >= cursor_) ? ii + 1 : ii + cursor_offset_;
        }
      }
    }
  }
}

void LogFilter::clearSearch() {
  search_cursor_saved_ = -1;
  search_pattern_ = Pattern::compile("");
  search_cursor_ = -1;
  search_cursor_fwd_ = -1;
  search_cursor_rev_ = -1;
}

void LogFilter::removeAtIndex(size_t pos) {
  if (pos >= log_indices_.size()) return;
  log_indices_.erase(log_indices_.begin() + pos);

  int64_t p = static_cast<int64_t>(pos);
  auto shift = [p](int64_t& val) {
    if (val > p) {
      val--;
    } else if (val == p) {
      val = -1;
    }
  };

  shift(cursor_);
  shift(select_start_);
  shift(select_end_);
  if (select_start_ < 0 || select_end_ < 0) {
    select_start_ = -1;
    select_end_   = -1;
  }
  shift(search_cursor_);
  shift(search_cursor_fwd_);
  shift(search_cursor_rev_);
}

void LogFilter::cleanSessionBoundaries() {
  if (!show_session_boundaries_) return;
  const auto& logs = logs_->logs();

  size_t i = 0;
  while (i < log_indices_.size()) {
    const LogLine& ll = log_indices_[i];
    if (ll.index >= logs.size() || ll.line != 0) { i++; continue; }
    const LogEntry& entry = logs[ll.index];
    if (entry.node != kMarkerNode || entry.text.empty()) { i++; continue; }
    if (entry.text[0].find("Session Started") == std::string::npos) { i++; continue; }

    // Found a "Session Started" at i.  Scan forward to characterise the session.
    size_t end_pos = log_indices_.size();  // sentinel: no end marker found
    bool has_logs           = false;
    bool followed_by_start  = false;

    for (size_t j = i + 1; j < log_indices_.size(); j++) {
      const LogLine& jll = log_indices_[j];
      if (jll.index >= logs.size()) continue;
      const LogEntry& je = logs[jll.index];
      if (je.node == kMarkerNode) {
        if (jll.line != 0) continue;
        bool is_end = !je.text.empty() &&
                      je.text[0].find("Recording Ended") != std::string::npos;
        if (is_end) {
          end_pos = j;
        } else {
          followed_by_start = true;
        }
        break;
      }
      has_logs = true;
    }

    if (!has_logs) {
      bool is_current = (end_pos == log_indices_.size()) && !followed_by_start;
      if (!is_current) {
        if (end_pos < log_indices_.size()) {
          removeAtIndex(end_pos);  // remove end first (higher index)
        }
        removeAtIndex(i);
        continue;  // don't increment: next entry shifted into position i
      }
    }
    i++;
  }
}

bool LogFilter::accepted(const LogEntry& entry, bool new_entry) {
  if (entry.node == kMarkerNode) {
    return show_session_boundaries_;
  }

  bool include = filter_patterns_.empty();

  auto node = nodes_.find(entry.node);
  if (node == nodes_.end()) {
    nodes_[entry.node] = {false, 1};
  } else if (new_entry) {
    node->second.count++;
  }

  if (!bag_sources_.empty() && entry.bag_source_idx >= 0) {
    auto idx = entry.bag_source_idx;
    if (idx < static_cast<int>(bag_sources_.size())) {
      if (new_entry) { bag_sources_[idx].count++; }
      if (deselected_bag_count_ > 0 && !bag_sources_[idx].selected) {
        return false;
      }
    }
  }

  if (entry.level == rcl_interfaces::msg::Log::DEBUG) {
    if (!debug_level_) {
      return false;
    }
  } else if (entry.level == rcl_interfaces::msg::Log::INFO) {
    if (!info_level_) {
      return false;
    }
  } else if (entry.level == rcl_interfaces::msg::Log::WARN) {
    if (!warn_level_) {
      return false;
    }
  } else if (entry.level == rcl_interfaces::msg::Log::ERROR) {
    if (!error_level_) {
      return false;
    }
  } else if (entry.level == rcl_interfaces::msg::Log::FATAL) {
    if (!fatal_level_) {
      return false;
    }
  }

  // Whitelist mode: only filter when filter is enabled, whitelist is non-empty,
  // and this node is not in the whitelist.
  if (filter_nodes_ && selected_node_count_ > 0 && !nodes_[entry.node].selected) {
    return false;
  }

  for (const auto& filter : filter_patterns_) {
    for (const auto& line : entry.text) {
      if (filter.matches(line)) {
        include = true;
        break;
      }
    }
  }

  if (include) {
    for (const auto& exclude : exclude_patterns_) {
      for (const auto& line : entry.text) {
        if (exclude.matches(line)) {
          include = false;
          break;
        }
      }
    }
  }

  return include;
}


LogFilter::SearchStats LogFilter::getSearchStats() const {
  if (search_pattern_.empty()) {
    stats_indices_size_ = 0;
    stats_cursor_ = -2;
    stats_position_ = 0;
    stats_total_ = 0;
    return {};
  }
  bool cursor_valid = (search_cursor_ >= 0);
  bool need_recompute = (log_indices_.size() != stats_indices_size_) ||
                        (cursor_valid && search_cursor_ != stats_cursor_);
  if (!need_recompute) {
    return {stats_position_, stats_total_};
  }
  stats_indices_size_ = log_indices_.size();
  if (cursor_valid) {
    stats_cursor_ = search_cursor_;
    stats_position_ = 0;
  }
  stats_total_ = 0;
  const auto& logs = logs_->logs();
  for (size_t i = 0; i < log_indices_.size(); i++) {
    const auto& ll = log_indices_[i];
    if (ll.index >= logs.size()) continue;
    if (search_pattern_.matches(logs[ll.index].text[ll.line])) {
      stats_total_++;
      if (cursor_valid && static_cast<int64_t>(i) == search_cursor_) {
        stats_position_ = stats_total_;
      }
    }
  }
  return {stats_position_, stats_total_};
}

size_t LogFilter::filteredCount() const {
  const auto& logs = logs_->logs();
  size_t count = 0;
  for (const auto& ll : log_indices_) {
    if (ll.line == 0 && ll.index < logs.size() && logs[ll.index].node != kMarkerNode) {
      count++;
    }
  }
  return count;
}

}  // namespace log_view
