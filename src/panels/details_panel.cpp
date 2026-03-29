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

  if (selected < 0) {
    mvwprintw(window_, 1, 1, "stamp: ");
    mvwprintw(window_, 2, 1, "level: ");
    mvwprintw(window_, 3, 1, "file: ");
    mvwprintw(window_, 4, 1, "function: ");
    mvwprintw(window_, 5, 1, "line: ");
    mvwprintw(window_, 6, 1, "message: ");
  } else {
    const auto& entry = filter_.getEntry(selected);

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
      row = printWrapped(row, line);
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
