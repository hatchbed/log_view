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

#include <log_view/panels/bag_source_panel.h>

#include <log_view/utils.h>

namespace log_view {

void BagSourcePanel::refresh() {
  int64_t cursor = getCursor();

  max_length_ = 0;
  if (!cleared_) {
    werase(window_);
  }
  cleared_ = false;

  box(window_, 0, 0);
  printStyledAt(window_, 0, width_ / 2 - 3, focus() ? A_BOLD : 0, " bags ");

  size_t start_idx = cursor;
  if (start_idx >= getContentHeight()) {
    start_idx -= getContentHeight();
  } else {
    start_idx = 0;
  }

  const auto& paths = filter_.bagPaths();
  const auto& sources = filter_.bagSources();

  bool selection_visible = false;
  for (size_t i = 0; i < getContentHeight() && i + start_idx < getContentSize(); i++) {
    if (static_cast<int>(i + start_idx) == selected_idx_) {
      selection_visible = true;
      break;
    }
  }

  if (!selection_visible) {
    moveTo(cursor);
    cursor = getCursor();
    start_idx = static_cast<size_t>(cursor);
    if (start_idx >= static_cast<size_t>(getContentHeight())) {
      start_idx -= getContentHeight();
    } else {
      start_idx = 0;
    }
  }

  for (size_t i = 0; i < getContentHeight() && i + start_idx < getContentSize(); i++) {
    int idx = static_cast<int>(i + start_idx);
    const auto& src_data = sources[idx];
    const auto& name = paths[idx];
    bool hover = focus_ && (idx == selected_idx_);
    bool selected = src_data.selected;

    std::string text = name + ": " + std::to_string(src_data.count);

    attr_t attr = (selected ? A_REVERSE : 0) | (hover ? A_BOLD : 0);

    max_length_ = std::max(max_length_, text.size());

    if (shift_ >= text.size()) {
      text.clear();
    } else if (shift_ > 0) {
      text.erase(0, shift_ + 2);
    }

    bool cropped = false;
    if (text.size() > getContentWidth()) {
      text.resize(getContentWidth() - 2);
      cropped = true;
    }

    printStyledAt(window_, i + 1, 1, attr, "%s", text.c_str());

    if (shift_ > 0) {
      mvwprintw(window_, i + 1, 1, "  ");
      printStyledAt(window_, i + 1, 1, A_REVERSE, "<");
    }
    if (cropped) {
      mvwprintw(window_, i + 1, getContentWidth() - 1, "  ");
      printStyledAt(window_, i + 1, getContentWidth(), A_REVERSE, ">");
    }

    {
      int erased     = (shift_ > 0) ? static_cast<int>(shift_) + 2 : 0;
      int ind_hidden = (shift_ > 0) ? 2 : 0;
      int vis_col    = 1 + ind_hidden;
      int name_in_text = std::max(0, static_cast<int>(name.size()) - erased);
      int vis_name     = std::max(0, name_in_text - ind_hidden);
      int vis_text     = std::max(0, static_cast<int>(text.size()) - ind_hidden);
      int chgat_n      = std::min(vis_name, vis_text);
      if (chgat_n > 0) {
        attr_t name_attr = (selected ? A_REVERSE : 0) | (hover ? A_BOLD : 0);
        if (COLORS < 16) { name_attr |= A_BOLD; }
        mvwchgat(window_, static_cast<int>(i) + 1, vis_col, chgat_n, name_attr, CP_BRIGHT_BLUE,
                 nullptr);
      }
    }
  }

  last_cursor_ = cursor;

  drawScrollBar(getContentSize(), getContentHeight(), 1, width_ - 1);
}

bool BagSourcePanel::handleKey(int key) {
  if (hidden()) {
    return false;
  }

  if (key == ctrl('a')) {
    filter_.selectAllBagSources();
    return true;
  } else if (key == 'i' && focus()) {
    filter_.invertBagSources();
    return true;
  }

  return false;
}

bool BagSourcePanel::handleMouse(const MEVENT& event) {
  if (hidden() || !encloses(event.y, event.x)) {
    return false;
  }

  if (event.bstate & BUTTON1_PRESSED) {
    int row = event.y - (y_ + 1);
    size_t cursor = getCursor();
    size_t start_idx = cursor;
    if (start_idx >= getContentHeight()) {
      start_idx -= getContentHeight();
    } else {
      start_idx = 0;
    }

    int index = static_cast<int>(start_idx) + row;
    if (index < 0 || index >= static_cast<int>(filter_.bagSources().size())) {
      return true;
    }

    selected_idx_ = index;
    filter_.toggleBagSource(selected_idx_);
    refresh();
  }
  return true;
}

void BagSourcePanel::follow(bool enable) {
  if (getContentSize() == 0) {
    return;
  }
  selected_idx_ = static_cast<int>(getContentSize()) - 1;
  moveTo(getCursor());
}

void BagSourcePanel::moveTo(size_t index) {
  if (getContentSize() == 0) {
    return;
  }

  size_t cursor = getCursor();
  int64_t offset = static_cast<int64_t>(index) - cursor;

  if (cursor == 0) {
    cursor = getContentHeight();
    setCursor(cursor);
  }

  int64_t selection = selected_idx_ + offset;
  selection = std::max(
    static_cast<int64_t>(0),
    std::min(static_cast<int64_t>(getContentSize()) - 1, selection));
  selected_idx_ = static_cast<int>(selection);

  size_t start_idx = cursor;
  if (start_idx >= getContentHeight()) {
    start_idx -= getContentHeight();
  } else {
    start_idx = 0;
  }

  if (selection < static_cast<int64_t>(start_idx)) {
    setCursor(selection + getContentHeight());
  } else if (selection >= static_cast<int64_t>(start_idx + getContentHeight())) {
    setCursor(selection + 1);
  }
}

bool BagSourcePanel::handleNavigation(int key) {
  if (hidden_ || !focus()) { return false; }

  int content_size = static_cast<int>(getContentSize());

  bool scroll_up   = (key == KEY_UP   || key == KEY_PPAGE || key == KEY_HOME);
  bool scroll_down = (key == KEY_DOWN || key == KEY_NPAGE || key == KEY_END);

  if (scroll_up   && selected_idx_ <= 0)                { return false; }
  if (scroll_down && selected_idx_ >= content_size - 1) { return false; }

  return PanelInterface::handleNavigation(key);
}

int BagSourcePanel::getContentWidth() const {
  return width_ - 2;
}

void BagSourcePanel::select() {
  filter_.toggleBagSource(selected_idx_);
}

}  // namespace log_view
