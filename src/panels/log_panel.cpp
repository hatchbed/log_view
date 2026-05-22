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

#include <log_view/panels/log_panel.h>

#include <ctime>

#include <log_view/utils.h>

namespace log_view {

void LogPanel::forceRefresh() {
  first_stamp_ns_ = -1;
  PanelInterface::forceRefresh();
}

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
    } else {
      size_t start_idx = getContentSize() - getContentHeight();
      for (size_t i = 0; i < height_; i++) {
        auto entry = filter_.indices()[i + start_idx];
        printEntry(i, logs[entry.index], entry.line, i + start_idx);
      }
    }
  } else if (!following() && (cleared_ || last_cursor_ != cursor || (!filled_ && new_logs))) {
    filled_ = false;
    max_length_ = 0;
    if (!cleared_) {
      werase(window_);
    }
    cleared_ = false;

    int64_t start_idx = cursor;
    if (cursor >= getContentHeight()) {
      start_idx -= getContentHeight();
    } else {
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

  if (key == 27 && filter_.getSelectStart() >= 0) {
    filter_.clearSelect();
    forceRefresh();
    return true;
  }

  if (key == ctrl('a')) {
    selectAll();
    return true;
  }

  if (key == KEY_SR || key == KEY_SF || key == KEY_SPREVIOUS ||
      key == KEY_SNEXT || key == KEY_SHOME || key == KEY_SEND) {
    extendSelect(key);
    return true;
  }

  return false;
}

void LogPanel::extendSelect(int key) {
  if (getContentSize() == 0) {
    return;
  }

  int64_t max_idx = static_cast<int64_t>(getContentSize()) - 1;

  int64_t cursor = filter_.getCursor();
  if (cursor < 0 || cursor > static_cast<int64_t>(getContentSize())) {
    cursor = static_cast<int64_t>(getContentSize());
  }
  int64_t view_height = static_cast<int64_t>(getContentHeight());
  int64_t view_top = std::max(static_cast<int64_t>(0), cursor - view_height);
  int64_t view_bottom = std::min(cursor - 1, max_idx);

  if (filter_.getSelectStart() < 0) {
    filter_.setSelectStart(view_bottom);
    filter_.setSelectEnd(view_bottom);
  }

  int64_t end = filter_.getSelectEnd();
  bool visible = (end >= view_top && end <= view_bottom);

  if (!visible) {
    follow(false);
    if (end < view_top) {
      filter_.setCursor(
        std::min(end + view_height, static_cast<int64_t>(getContentSize())));
    } else {
      moveTo(static_cast<size_t>(end) + 1);
    }
    forceRefresh();
    return;
  }

  if (key == KEY_SR) {
    end = std::max(static_cast<int64_t>(0), end - 1);
  } else if (key == KEY_SF) {
    end = std::min(max_idx, end + 1);
  } else if (key == KEY_SPREVIOUS) {
    if (end == view_top) {
      pageUp();
      cursor = filter_.getCursor();
      if (cursor < 0 || cursor > static_cast<int64_t>(getContentSize())) {
        cursor = static_cast<int64_t>(getContentSize());
      }
      view_top = std::max(static_cast<int64_t>(0), cursor - view_height);
    }
    end = view_top;
  } else if (key == KEY_SNEXT) {
    if (end == view_bottom) {
      pageDown();
      cursor = filter_.getCursor();
      if (cursor < 0 || cursor > static_cast<int64_t>(getContentSize())) {
        cursor = static_cast<int64_t>(getContentSize());
      }
      view_bottom = std::min(cursor - 1, max_idx);
    }
    end = view_bottom;
  } else if (key == KEY_SHOME) {
    end = 0;
  } else if (key == KEY_SEND) {
    end = max_idx;
  }

  filter_.setSelectEnd(end);

  cursor = filter_.getCursor();
  if (cursor < 0 || cursor > static_cast<int64_t>(getContentSize())) {
    cursor = static_cast<int64_t>(getContentSize());
  }
  int64_t new_view_top = std::max(static_cast<int64_t>(0), cursor - view_height);
  int64_t new_view_bottom = std::min(cursor - 1, max_idx);

  if (end < new_view_top) {
    follow(false);
    filter_.setCursor(
      std::min(end + view_height, static_cast<int64_t>(getContentSize())));
  } else if (end > new_view_bottom) {
    moveTo(static_cast<size_t>(end) + 1);
  }

  copyToClipboard();
  forceRefresh();
}


bool LogPanel::handleMouse(const MEVENT& event) {
  if (hidden() || !encloses(event.y, event.x)) {
    return false;
  }

  if (event.bstate & BUTTON1_PRESSED) {
    mouse_down_ = true;
    if ((event.bstate & BUTTON_CTRL) && filter_.getSelectStart() >= 0) {
      endSelect(event.y - y_);
    } else {
      startSelect(event.y - y_);
    }
    forceRefresh();
    return true;
  } else if (mouse_down_ && (event.bstate & REPORT_MOUSE_POSITION)) {
    endSelect(event.y - y_);
    forceRefresh();
    return true;
  } else if (event.bstate & BUTTON1_RELEASED) {
    mouse_down_ = false;
    copyToClipboard();
    return true;
  } else if (!mouse_down_ && (event.bstate & BUTTON3_PRESSED)) {
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
  } else {
    start_idx = 0;
  }

  filter_.setSelectStart(start_idx + row);
}

void LogPanel::endSelect(int row) {
  size_t start_idx = filter_.getCursor();
  if (start_idx >= getContentHeight()) {
    start_idx -= getContentHeight();
  } else {
    start_idx = 0;
  }

  filter_.setSelectEnd(start_idx + row);
}

int LogPanel::getContentWidth() const {
  if (right_edge_ > 0) {
    return right_edge_;
  }
  int width = width_;
  if (getContentSize() >= getContentHeight()) {
    width--;
  }
  return width;
}

std::string LogPanel::getPrefix(const LogEntry& entry, size_t line) const {
  if (entry.node == kMarkerNode) {
    return "";
  }

  std::string timestamp;
  switch (prefs_.timestamp_format) {
    case Preferences::TimestampFormat::ELAPSED: {
      if (first_stamp_ns_ < 0) {
        int64_t global_first = logs_->firstStampNs();
        first_stamp_ns_ = (global_first >= 0) ? global_first : entry.stamp.nanoseconds();
      }
      double elapsed = static_cast<double>(entry.stamp.nanoseconds() - first_stamp_ns_) * 1e-9;
      timestamp = toString(elapsed, 4);
      break;
    }
    case Preferences::TimestampFormat::TIME_OF_DAY: {
      int64_t ns = entry.stamp.nanoseconds();
      time_t secs = static_cast<time_t>(ns / 1000000000LL);
      int millis = static_cast<int>((ns % 1000000000LL) / 1000000LL);
      struct tm t;
      localtime_r(&secs, &t);
      char buf[16];
      snprintf(buf, sizeof(buf), "%02d:%02d:%02d.%03d", t.tm_hour, t.tm_min, t.tm_sec, millis);
      timestamp = buf;
      break;
    }
    default:
      timestamp = toString(entry.stamp.seconds(), 4);
      break;
  }
  std::string text = timestamp + " [" + levelName(entry.level) + "] ";
  if (line > 0) {
    text = std::string(text.size(), ' ');
  }

  return text;
}

void LogPanel::printEntry(size_t row, const LogEntry& entry, size_t line, size_t idx) {
  if (entry.node == kMarkerNode) {
    const std::string& label = entry.text[0];
    int w = getContentWidth();
    int pad = std::max(0, (w - static_cast<int>(label.size())) / 2);
    std::string text(pad, '-');
    text += label;
    text += std::string(std::max(0, w - static_cast<int>(text.size())), '-');
    if (static_cast<int>(text.size()) > w) {
      text.resize(w);
    }
    printStyledAt(window_, row, 0, kAttrGrey, "%s", text.c_str());
    return;
  }

  bool selected = false;
  int64_t select_start = filter_.getSelectStart();
  int64_t select_end = filter_.getSelectEnd();
  if (select_start != -1) {
    int start = std::min(select_start, select_end);
    int end = std::max(select_start, select_end);
    selected = idx >= start && idx <= end;
  }

  attr_t level_attr = selected ? A_REVERSE : 0;
  if (entry.level == rcl_interfaces::msg::Log::DEBUG) {
    level_attr |= A_DIM;
  } else if (entry.level == rcl_interfaces::msg::Log::ERROR) {
    level_attr |= COLOR_PAIR(CP_RED);
  } else if (entry.level == rcl_interfaces::msg::Log::FATAL) {
    level_attr |= A_BOLD | COLOR_PAIR(CP_RED);
  } else if (entry.level == rcl_interfaces::msg::Log::WARN) {
    level_attr |= COLOR_PAIR(CP_YELLOW);
  }
  std::string prefix = getPrefix(entry, line);
  const std::string& raw_line = entry.text[line];
  std::string stripped_line = raw_line.find('\033') != std::string::npos
    ? stripAnsi(raw_line) : raw_line;
  std::string text = prefix + stripped_line;
  max_length_ = std::max(max_length_, utf8DisplayWidth(text));

  std::string match = filter_.getSearch();
  size_t match_size = match.size();
  std::vector<size_t> match_indices;
  bool matched = false;
  if (!match.empty()) {
    match_indices = find(stripped_line, match, true);
    matched = !match_indices.empty();
  }

  if (shift_ > 0) {
    text = utf8EraseDisplayCols(text, static_cast<size_t>(shift_));
  }
  text = utf8TruncateDisplayCols(text, static_cast<size_t>(width_));

  printStyledAt(window_, row, 0, level_attr, "%s", text.c_str());

  // ANSI color/bold overlay pass
  if (raw_line.find('\033') != std::string::npos) {
    size_t vis_col = prefix.length();  // visible column of current segment start
    for (const auto& seg : parseAnsiSegments(raw_line)) {
      size_t seg_end = vis_col + seg.text.size();
      bool has_color = seg.ansi_fg >= 0 && seg.ansi_fg <= 7;
      bool has_attr  = has_color || seg.bold || seg.dim;
      if (has_attr && !seg.text.empty()) {
        int64_t scr_start = static_cast<int64_t>(vis_col) - static_cast<int64_t>(shift_);
        int64_t scr_end   = static_cast<int64_t>(seg_end) - static_cast<int64_t>(shift_);
        if (scr_start < static_cast<int64_t>(width_) && scr_end > 0) {
          int64_t clip_start = std::max(static_cast<int64_t>(0), scr_start);
          int64_t clip_end   = std::min(static_cast<int64_t>(width_), scr_end);
          size_t  txt_offset = static_cast<size_t>(clip_start - scr_start);
          size_t  txt_len    = static_cast<size_t>(clip_end - clip_start);
          attr_t attr = selected ? A_REVERSE : 0;
          if (has_color) attr |= COLOR_PAIR(kAnsiPairs[seg.ansi_fg]);
          if (seg.bold)  attr |= A_BOLD;
          if (seg.dim)   attr |= A_DIM;
          printStyledAt(window_, row, static_cast<int>(clip_start), attr,
            "%.*s", static_cast<int>(txt_len), seg.text.c_str() + txt_offset);
        }
      }
      vis_col = seg_end;
    }
  }

  if (matched) {
    bool off_left = false;
    bool off_right = false;
    int64_t visible_width = static_cast<int64_t>(getContentWidth());
    for (const auto& match_index : match_indices) {
      int64_t scr_start = static_cast<int64_t>(match_index + prefix.length())
                          - static_cast<int64_t>(shift_);
      int64_t scr_end   = scr_start + static_cast<int64_t>(match_size);

      if (scr_start >= visible_width) {
        off_right = true;
        continue;
      }
      if (scr_end <= 0) {
        off_left = true;
        continue;
      }

      int64_t clip_start = std::max(static_cast<int64_t>(0), scr_start);
      int64_t clip_end   = std::min({scr_end, visible_width,
                                     static_cast<int64_t>(text.size())});

      if (clip_end <= clip_start) {
        continue;
      }

      printStyledAt(window_, row, static_cast<int>(clip_start),
        (selected ? A_REVERSE : 0) | COLOR_PAIR(CP_DEFAULT_CYAN),
        "%.*s", static_cast<int>(clip_end - clip_start),
        text.c_str() + clip_start);
    }

    if (off_right) {
      int col = (right_edge_ > 0) ? right_edge_ - 1
                                   : width_ - 1 - (scrollbar() ? 1 : 0);
      printStyledAt(window_, row, col, COLOR_PAIR(CP_WHITE_CYAN), ">");
    }
    if (off_left) {
      printStyledAt(window_, row, 0, COLOR_PAIR(CP_WHITE_CYAN), "<");
    }
  }
}

}  // namespace log_view
