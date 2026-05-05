// Copyright 2026 Hatchbed L.L.C.
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

#include <log_view/panels/details_panel.h>

#include <string>

#include <rcl_interfaces/msg/log.hpp>
#include <log_view/datatypes.h>
#include <log_view/utils.h>

namespace log_view {

void DetailsPanel::refresh() {
  max_length_ = 0;
  if (!cleared_) {
    werase(window_);
  }
  cleared_ = false;

  box(window_, 0, 0);
  mvwprintw(window_, 0, width_ / 2 - 3, " details ");

  int64_t selected = filter_.getSelectEnd();

  int max_width = getContentWidth();
  auto printWrapped = [&](int row, const std::string& text) -> int {
    mvwaddnstr(window_, row++, 1, text.c_str(), max_width);
    size_t offset = max_width;
    while (offset < text.size()) {
      mvwaddnstr(window_, row++, 3, text.c_str() + offset, max_width - 2);
      offset += max_width - 2;
    }
    return row;
  };

  static const int kAnsiPairs[] = {
    CP_ANSI_BLACK, CP_ANSI_RED,   CP_ANSI_GREEN,   CP_ANSI_YELLOW,
    CP_ANSI_BLUE,  CP_ANSI_MAGENTA, CP_ANSI_CYAN,  CP_ANSI_WHITE
  };

  // Like printWrapped but strips ANSI before layout then repaints color attributes.
  auto printWrappedAnsi = [&](int row, const std::string& raw) -> int {
    bool has_ansi = raw.find('\033') != std::string::npos;
    const std::string& display = has_ansi ? stripAnsi(raw) : raw;

    // First pass: layout with stripped text
    int last_row = row;
    mvwaddnstr(window_, last_row, 1, display.c_str(), max_width);
    size_t offset = static_cast<size_t>(max_width);
    while (offset < display.size()) {
      last_row++;
      mvwaddnstr(window_, last_row, 3, display.c_str() + offset, max_width - 2);
      offset += static_cast<size_t>(max_width - 2);
    }

    // Second pass: ANSI color overlay
    if (has_ansi) {
      int cur_row = row;
      int col_off = 1;
      int row_cap = max_width;
      int vis_col = 0;
      for (const auto& seg : parseAnsiSegments(raw)) {
        bool has_color = seg.ansi_fg >= 0 && seg.ansi_fg <= 7;
        bool has_attr  = has_color || seg.bold || seg.dim;
        size_t remaining = seg.text.size();
        size_t seg_off   = 0;
        while (remaining > 0) {
          int space = row_cap - vis_col;
          if (space <= 0) {
            cur_row++;
            col_off = 3;
            row_cap = max_width - 2;
            vis_col = 0;
            space   = row_cap;
          }
          size_t chunk = std::min(remaining, static_cast<size_t>(space));
          if (has_attr) {
            if (has_color) { wattron(window_, COLOR_PAIR(kAnsiPairs[seg.ansi_fg])); }
            if (seg.bold)  { wattron(window_, A_BOLD); }
            if (seg.dim)   { wattron(window_, A_DIM); }
            mvwprintw(window_, cur_row, col_off + vis_col,
              "%.*s", static_cast<int>(chunk), seg.text.c_str() + seg_off);
            if (seg.dim)   { wattroff(window_, A_DIM); }
            if (seg.bold)  { wattroff(window_, A_BOLD); }
            if (has_color) { wattroff(window_, COLOR_PAIR(kAnsiPairs[seg.ansi_fg])); }
          }
          vis_col   += static_cast<int>(chunk);
          seg_off   += chunk;
          remaining -= chunk;
        }
      }
    }

    return last_row + 1;
  };

  const LogEntry* entry_ptr = nullptr;
  if (selected >= 0) {
    const auto& e = filter_.getEntry(selected);
    if (e.node != kMarkerNode) {
      entry_ptr = &e;
    }
  }

  if (!entry_ptr) {
    mvwprintw(window_, 1, 1, "stamp: ");
    mvwprintw(window_, 2, 1, "level: ");
    mvwprintw(window_, 3, 1, "file: ");
    mvwprintw(window_, 4, 1, "function: ");
    mvwprintw(window_, 5, 1, "line: ");
    mvwprintw(window_, 6, 1, "message: ");
  } else {
    const auto& entry = *entry_ptr;

    int row = 1;
    row = printWrapped(row, "stamp: " + toString(entry.stamp.seconds(), 4));

    std::string level_text = "level: ";
    if (entry.level == rcl_interfaces::msg::Log::DEBUG) {
      level_text += "DEBUG";
    } else if (entry.level == rcl_interfaces::msg::Log::INFO) {
      level_text += "INFO";
    } else if (entry.level == rcl_interfaces::msg::Log::WARN) {
      level_text += "WARN";
    } else if (entry.level == rcl_interfaces::msg::Log::ERROR) {
      level_text += "ERROR";
    } else if (entry.level == rcl_interfaces::msg::Log::FATAL) {
      level_text += "FATAL";
    } else {
      level_text += std::to_string(entry.level);
    }
    row = printWrapped(row, level_text);
    row = printWrapped(row, "file: " + entry.file);
    row = printWrapped(row, "function: " + entry.function);
    row = printWrapped(row, "line: " + std::to_string(entry.line));
    mvwprintw(window_, row++, 1, "message: ");
    for (const auto& line : entry.text) {
      row = printWrappedAnsi(row, line);
    }
  }

  drawScrollBar(getContentSize(), getContentHeight(), 1, width_ - 2);
}

int DetailsPanel::getContentWidth() const {
  int width = width_ - 2;
  if (getContentSize() > getContentHeight()) {
    width--;
  }
  return width;
}

}  // namespace log_view
