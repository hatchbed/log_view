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

#include <log_view/datatypes.h>
#include <log_view/utils.h>

namespace log_view {

static int countWrappedRows(const std::string& text, int max_width) {
  if (max_width <= 1) { return 1; }
  int rows = 1;
  int offset = max_width;
  while (offset < static_cast<int>(text.size())) {
    rows++;
    offset += max_width - 2;
  }
  return rows;
}

bool DetailsPanel::handleNavigation(int key) {
  if (hidden_) { return false; }

  int view_height  = getContentHeight();
  int content_size = static_cast<int>(getContentSize());

  if (content_size <= view_height) { return false; }

  bool scroll_up   = (key == KEY_UP   || key == KEY_PPAGE || key == KEY_HOME);
  bool scroll_down = (key == KEY_DOWN || key == KEY_NPAGE || key == KEY_END);

  if (scroll_up   && cursor_ >= 0 && cursor_ <= static_cast<int64_t>(view_height)) { return false; }
  if (scroll_down && cursor_ < 0) { return false; }

  return PanelInterface::handleNavigation(key);
}

void DetailsPanel::refresh() {
  max_length_ = 0;
  if (!cleared_) {
    werase(window_);
  }
  cleared_ = false;

  int64_t selected = filter_.getSelectEnd();

  const LogEntry* entry_ptr = nullptr;
  if (selected >= 0) {
    const auto& e = filter_.getEntry(selected);
    if (e.node != kMarkerNode) {
      entry_ptr = &e;
    }
  }

  // Build level text early - needed for both content-size counting and rendering.
  std::string level_text;
  if (entry_ptr) {
    level_text = "level: " + levelName(entry_ptr->level);
  }

  // Compute the total number of content rows for the current entry at a given width.
  int view_height = getContentHeight();
  auto computeSize = [&](int max_w) -> size_t {
    if (!entry_ptr) { return 6; }
    const auto& entry = *entry_ptr;
    int row = 1;
    row += countWrappedRows("stamp: " + toString(entry.stamp.toSec(), 4), max_w);
    row += countWrappedRows(level_text, max_w);
    row += countWrappedRows("file: " + entry.file, max_w);
    row += countWrappedRows("function: " + entry.function, max_w);
    row += countWrappedRows("line: " + std::to_string(entry.line), max_w);
    row++;  // "message: " label
    for (const auto& line : entry.text) {
      row += countWrappedRows(line, max_w);
    }
    return static_cast<size_t>(row - 1);
  };

  content_size_ = computeSize(width_ - 2);

  // Reset scroll to top when the selected entry changes.
  if (selected != last_selected_) {
    last_selected_ = selected;
    cursor_ = static_cast<int64_t>(view_height);
  }

  // Compute how many rows of content are above the visible area.
  int scroll_top = 0;
  if (static_cast<int>(content_size_) > view_height) {
    scroll_top = (cursor_ < 0)
      ? static_cast<int>(content_size_) - view_height
      : static_cast<int>(cursor_) - view_height;
    scroll_top = std::max(0, scroll_top);
  }

  box(window_, 0, 0);
  if (focus()) { wattron(window_, A_BOLD); }
  mvwprintw(window_, 0, width_ / 2 - 3, " details ");
  if (focus()) { wattroff(window_, A_BOLD); }

  int max_width = getContentWidth();

  // Renders `text` starting at logical row `row`, clipping rows outside the viewport.
  auto printWrapped = [&](int row, const std::string& text) -> int {
    {
      int dr = row - scroll_top;
      if (dr >= 1 && dr <= height_ - 2) {
        mvwaddnstr(window_, dr, 1, text.c_str(), max_width);
      }
    }
    row++;
    size_t offset = static_cast<size_t>(max_width);
    while (offset < text.size()) {
      int dr = row - scroll_top;
      if (dr >= 1 && dr <= height_ - 2) {
        mvwaddnstr(window_, dr, 3, text.c_str() + offset, max_width - 2);
      }
      row++;
      offset += static_cast<size_t>(max_width - 2);
    }
    return row;
  };

  // Like printWrapped but prints `key` in blue and `value` in default color.
  auto printWrappedWithKey =
      [&](int row, const std::string& key, const std::string& value) -> int {
    const std::string full = key + value;
    {
      int dr = row - scroll_top;
      if (dr >= 1 && dr <= height_ - 2) {
        wattron(window_, kAttrBoldBlue);
        mvwaddnstr(window_, dr, 1, key.c_str(), std::min(static_cast<int>(key.size()), max_width));
        wattroff(window_, kAttrBoldBlue);
        int val_col   = 1 + static_cast<int>(key.size());
        int val_width = max_width - static_cast<int>(key.size());
        if (val_width > 0) {
          mvwaddnstr(window_, dr, val_col, value.c_str(), val_width);
        }
      }
    }
    row++;
    size_t offset = static_cast<size_t>(max_width);
    while (offset < full.size()) {
      int dr = row - scroll_top;
      if (dr >= 1 && dr <= height_ - 2) {
        mvwaddnstr(window_, dr, 3, full.c_str() + offset, max_width - 2);
      }
      row++;
      offset += static_cast<size_t>(max_width - 2);
    }
    return row;
  };

  if (!entry_ptr) {
    const char* labels[] = {"stamp: ", "level: ", "file: ", "function: ", "line: ", "message: "};
    for (int i = 0; i < 6; i++) {
      int dr = (i + 1) - scroll_top;
      if (dr >= 1 && dr <= height_ - 2) {
        wattron(window_, kAttrBoldBlue);
        mvwprintw(window_, dr, 1, "%s", labels[i]);
        wattroff(window_, kAttrBoldBlue);
      }
    }
  } else {
    const auto& entry = *entry_ptr;
    int row = 1;
    row = printWrappedWithKey(row, "stamp: ", toString(entry.stamp.toSec(), 4));
    row = printWrappedWithKey(row, "level: ", level_text.substr(7));
    row = printWrappedWithKey(row, "file: ", entry.file);
    row = printWrappedWithKey(row, "function: ", entry.function);
    row = printWrappedWithKey(row, "line: ", std::to_string(entry.line));
    {
      int dr = row - scroll_top;
      if (dr >= 1 && dr <= height_ - 2) {
        wattron(window_, kAttrBoldBlue);
        mvwprintw(window_, dr, 1, "message: ");
        wattroff(window_, kAttrBoldBlue);
      }
    }
    row++;
    for (const auto& line : entry.text) {
      row = printWrapped(row, line);
    }
  }

  drawScrollBar(content_size_, view_height, 1, width_ - 1);
}

int DetailsPanel::getContentWidth() const {
  return width_ - 2;
}

}  // namespace log_view
