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

#include <log_view/panels/log_panel.h>

#include <log_view/utils.h>

namespace log_view {

void LogPanel::refresh() {
  int64_t cursor = getCursor();

  bool new_logs = last_content_size_ != getContentSize();

  if (following() && (cleared_ || new_logs)) {
    max_length_ = 0;

    if (!cleared_) {
      werase(window_);
    }
    cleared_ = false;

    const auto& logs = logs_->logs();
    if (getContentSize() < getContentHeight()) {
      for (size_t i = 0; i < getContentSize(); i++) {
        auto entry = filter_.indices()[i];
        printEntry(i, logs[entry.index], entry.line, i);
      }
    }
    else {
      size_t start_idx = getContentSize() - getContentHeight();
      for (size_t i = 0; i < height_; i++) {
        auto entry = filter_.indices()[i + start_idx];
        printEntry(i, logs[entry.index], entry.line, i + start_idx);
      }
    }
  }
  else if (!following() && (cleared_ || last_cursor_ != cursor || (!filled_ && new_logs))) {
    filled_ = false;
    max_length_ = 0;
    if (!cleared_) {
      werase(window_);
    }
    cleared_ = false;

    int64_t start_idx = cursor;
    if (cursor >= getContentHeight()) {
      start_idx -= getContentHeight();
    }
    else {
      start_idx = 0;
    }

    const auto& logs = logs_->logs();
    for (size_t i = 0; i < getContentHeight() && i + start_idx < getContentSize(); i++) {
      auto entry = filter_.indices()[i + start_idx];
      printEntry(i, logs[entry.index], entry.line, i + start_idx);
    }
    filled_ = start_idx + getContentHeight() < getContentSize();
  }

  last_content_size_ = getContentSize();
  last_cursor_ = cursor;

  drawScrollBar(getContentSize(), getContentHeight(), 0, width_ - 1);
}

bool LogPanel::handleKey(int key) {
  if (hidden()) {
    return false;
  }

  if (key == ctrl('a')) {
      selectAll();

      return true;
  }

  return false;
}


bool LogPanel::handleMouse(const MEVENT& event) {
  if (hidden() || !encloses(event.y, event.x)) {
    return false;
  }

  if (event.bstate & BUTTON1_PRESSED) {
    mouse_down_ = true;
    startSelect(event.y - y_);
    forceRefresh();
    return true;
  }
  else if (mouse_down_ && (event.bstate & REPORT_MOUSE_POSITION)) {
    endSelect(event.y - y_);
    forceRefresh();
    return true;
  }
  else if (event.bstate & BUTTON1_RELEASED) {
    mouse_down_ = false;
    copyToClipboard();
    return true;
  }
  else if (!mouse_down_ && (event.bstate & BUTTON3_PRESSED)) {
    filter_.clearSelect();
    forceRefresh();
    return true;
  }

  return false;
}

void LogPanel::resize(int height, int width, int y, int x) {
  PanelInterface::resize(height, width, y, x);
  filter_.setCursorOffset(height_);
}

void LogPanel::selectAll() {
  if (getContentSize() > 0) {
    filter_.setSelectStart(0);
    filter_.setSelectEnd(getContentSize() - 1);
    copyToClipboard();
    forceRefresh();
  }
}

void LogPanel::copyToClipboard() {
  int64_t select_start = filter_.getSelectStart();
  int64_t select_end = filter_.getSelectEnd();
  if (select_start >=0 && select_end >= 0) {
    int start = std::min(select_start, select_end);
    int end = std::max(select_start, select_end);
    const auto& logs = logs_->logs();
    std::string data;
    for (size_t i = start; i <= end && i < getContentSize(); i++) {
      auto entry = filter_.indices()[i];
      std::string prefix = getPrefix(logs[entry.index], entry.line);
      data += prefix + logs[entry.index].text[entry.line] + "\n";
    }
    toClipboard(data);
  }
}

void LogPanel::startSelect(int row) {
  follow(false);

  size_t start_idx = filter_.getCursor();
  if (start_idx >= getContentHeight()) {
    start_idx -= getContentHeight();
  }
  else {
    start_idx = 0;
  }

  filter_.setSelectStart(start_idx + row);
}

void LogPanel::endSelect(int row) {
  size_t start_idx = filter_.getCursor();
  if (start_idx >= getContentHeight()) {
    start_idx -= getContentHeight();
  }
  else {
    start_idx = 0;
  }

  filter_.setSelectEnd(start_idx + row);
}

int LogPanel::getContentWidth() const {
  int width = width_;
  if (getContentSize() >= getContentHeight()) {
    width--;
  }
  return width;
}

std::string LogPanel::getPrefix(const LogEntry& entry, size_t line) const {
  std::string text = toString(entry.stamp.seconds(), 4) + " [";
  if (entry.level == rcl_interfaces::msg::Log::DEBUG) {
    text += "DEBUG";
  }
  else if (entry.level == rcl_interfaces::msg::Log::INFO) {
    text += "INFO";
  }
  else if (entry.level == rcl_interfaces::msg::Log::WARN) {
    text += "WARN";
  }
  else if (entry.level == rcl_interfaces::msg::Log::ERROR) {
    text += "ERROR";
  }
  else if (entry.level == rcl_interfaces::msg::Log::FATAL) {
    text += "FATAL";
  }
  else {
    text += std::to_string(entry.level);
  }
  text += "] ";
  if (line > 0) {
    text = std::string(text.size(), ' ');
  }

  return text;
}

void LogPanel::printEntry(size_t row, const LogEntry& entry, size_t line, size_t idx) {
  bool selected = false;
  int64_t select_start = filter_.getSelectStart();
  int64_t select_end = filter_.getSelectEnd();
  if (select_start != -1) {
    int start = std::min(select_start, select_end);
    int end = std::max(select_start, select_end);
    selected = idx >= start && idx <= end;
  }

  if (selected) {
    wattron(window_, A_REVERSE);
  }

  if (entry.level == rcl_interfaces::msg::Log::DEBUG) {
    wattron(window_, A_DIM);
  }
  else if (entry.level == rcl_interfaces::msg::Log::ERROR) {
    wattron(window_, COLOR_PAIR(CP_RED));
  }
  else if (entry.level == rcl_interfaces::msg::Log::FATAL) {
    wattron(window_, A_BOLD);
    wattron(window_, COLOR_PAIR(CP_RED));
  }
  else if (entry.level == rcl_interfaces::msg::Log::WARN) {

    wattron(window_, COLOR_PAIR(CP_YELLOW));
  }

  std::string prefix = getPrefix(entry, line);
  std::string text = prefix + entry.text[line];
  max_length_ = std::max(max_length_, text.size());

  std::string match = filter_.getSearch();
  size_t match_size = match.size();
  std::vector<size_t> match_indices;
  bool matched = false;
  if (!match.empty()) {
    match_indices = find(entry.text[line], match, true);
    matched = !match_indices.empty();
  }

  if (shift_ >= text.size()) {
    text.clear();
  }
  else if (shift_ > 0) {
    text.erase(0, shift_);
  }
  if (text.size() > width_) {
    text.resize(width_);
  }

  mvwprintw(window_, row, 0, text.c_str());

  if (matched) {
    wattron(window_, COLOR_PAIR(CP_DEFAULT_CYAN));

    if (text.empty()) {
      mvwprintw(window_, row, 0, " ");
    }
    else {
      for (const auto& match_index: match_indices) {
        int64_t start_idx = match_index + prefix.length() - shift_;
        int64_t end_idx = start_idx + match_size;

        start_idx = std::min(static_cast<int64_t>(text.size()) - 2, std::max(static_cast<int64_t>(0), start_idx));
        end_idx = std::min(static_cast<int64_t>(text.size()) - 2, std::max(static_cast<int64_t>(0), end_idx));

        int64_t substr_len = std::max(static_cast<int64_t>(1), end_idx - start_idx);

        mvwprintw(window_, row, start_idx, text.substr(start_idx, substr_len).c_str());
      }
    }
    wattroff(window_, COLOR_PAIR(CP_DEFAULT_CYAN));
  }

  if (entry.level == rcl_interfaces::msg::Log::DEBUG) {
    wattroff(window_, A_DIM);
  }
  else if (entry.level == rcl_interfaces::msg::Log::ERROR) {
    wattroff(window_, COLOR_PAIR(CP_RED));
  }
  if (entry.level == rcl_interfaces::msg::Log::FATAL) {
    wattroff(window_, COLOR_PAIR(CP_RED));
    wattroff(window_, A_BOLD);
  }
  else if (entry.level == rcl_interfaces::msg::Log::WARN) {
    wattroff(window_, COLOR_PAIR(CP_YELLOW));
  }

  if (selected) {
    wattroff(window_, A_REVERSE);
  }
}

} // namespace log_view
