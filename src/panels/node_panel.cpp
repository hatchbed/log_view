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

#include <log_view/panels/node_panel.h>

#include <log_view/utils.h>

namespace log_view {

void NodePanel::refresh() {
  int64_t cursor = getCursor();

  max_length_ = 0;
  if (!cleared_) {
    werase(window_);
  }
  cleared_ = false;

  box(window_, 0, 0);
  printStyledAt(window_, 0, width_ / 2 - 3, focus() ? A_BOLD : 0, " nodes ");

  size_t start_idx = cursor;
  if (start_idx >= getContentHeight()) {
    start_idx -= getContentHeight();
  } else {
    start_idx = 0;
  }

  std::vector<std::pair<std::string, NodeData>> nodes;
  for (const auto& node : filter_.nodes()) {
    nodes.push_back(node);
  }

  bool selection_visible = false;
  for (size_t i = 0; i < getContentHeight() && i + start_idx < getContentSize(); i++) {
    auto name = nodes[i + start_idx].first;
    if (name == selected_) {
      selection_visible = true;
      break;
    }
  }

  // force in focus element to be visible
  if (!selection_visible) {
    moveTo(cursor);
  }

  for (size_t i = 0; i < getContentHeight() && i + start_idx < getContentSize(); i++) {
    const auto& node_data = nodes[i + start_idx].second;
    auto name = nodes[i + start_idx].first;
    bool hover = focus_ && (name == selected_);
    bool selected = node_data.selected;

    std::string text = name + ": " + std::to_string(node_data.count);

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
        if (COLORS < 16) { name_attr |= A_BOLD; }  // bold required for brightness on 8-color
        mvwchgat(window_, static_cast<int>(i) + 1, vis_col, chgat_n, name_attr, CP_BRIGHT_BLUE,
                 nullptr);
      }
    }
  }

  last_cursor_ = cursor;

  drawScrollBar(getContentSize(), getContentHeight(), 1, width_ - 1);
}

bool NodePanel::handleKey(int key) {
  if (hidden()) {
    return false;
  }

  if (key == ctrl('a')) {
      filter_.selectAllNodes();

      return true;
  } else if (key == 'i' && focus()) {
      filter_.invertNodes();

      return true;
  }

  return false;
}


bool NodePanel::handleMouse(const MEVENT& event) {
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

    size_t index = start_idx + row;
    if (index >= filter_.nodes().size()) {
      return true;
    }

    std::vector<std::pair<std::string, NodeData>> nodes;
    for (const auto& node : filter_.nodes()) {
      nodes.push_back(node);
    }

    selected_ = nodes[index].first;
    filter_.toggleNode(selected_);
    refresh();
  }
  return true;
}


void NodePanel::follow(bool enable) {
  if (getContentSize() == 0) {
    return;
  }
  selected_ = filter_.nodes().rbegin()->first;
  moveTo(getCursor());
}

void NodePanel::moveTo(size_t index) {
  if (getContentSize() == 0) {
    return;
  }

  size_t cursor = getCursor();
  int64_t offset = static_cast<int64_t>(index) - cursor;

  if (cursor == 0) {
    cursor = getContentHeight();
    setCursor(cursor);
  }

  std::vector<std::pair<std::string, NodeData>> nodes;
  int64_t selection = -1;
  size_t idx = 0;
  for (const auto& node : filter_.nodes()) {
    nodes.push_back(node);
    if (node.first == selected_) {
      selection = idx;
    }
    idx++;
  }

  if (selection < 0 || index == 0) {
    selection = 0;
  } else {
    selection += offset;
    selection = std::max(
      static_cast<int64_t>(0),
      std::min(static_cast<int64_t>(getContentSize()) - 1, selection));
  }
  selected_ = nodes[selection].first;

  size_t start_idx = cursor;
  if (start_idx >= getContentHeight()) {
    start_idx -= getContentHeight();
  } else {
    start_idx = 0;
  }

  if (selection < start_idx) {
    setCursor(selection + getContentHeight());
  } else if (selection >= start_idx + getContentHeight()) {
    setCursor(selection + 1);
  }
}

bool NodePanel::handleNavigation(int key) {
  if (hidden_ || !focus()) { return false; }

  int view_height  = getContentHeight();
  int content_size = static_cast<int>(getContentSize());

  if (content_size <= view_height) { return false; }

  int64_t selection = 0;
  int64_t idx = 0;
  for (const auto& node : filter_.nodes()) {
    if (node.first == selected_) {
      selection = idx;
    }
    idx++;
  }

  bool scroll_up   = (key == KEY_UP   || key == KEY_PPAGE || key == KEY_HOME);
  bool scroll_down = (key == KEY_DOWN || key == KEY_NPAGE || key == KEY_END);

  if (scroll_up   && selection <= 0)                { return false; }
  if (scroll_down && selection >= content_size - 1) { return false; }

  return PanelInterface::handleNavigation(key);
}

int NodePanel::getContentWidth() const {
  return width_ - 2;
}

void NodePanel::select() {
  filter_.toggleNode(selected_);
}

}  // namespace log_view
