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

#include <log_view/panels/node_panel.h>

namespace log_view {

void NodePanel::refresh() {
  size_t cursor = getCursor();

  max_length_ = 0;
  if (!cleared_) {
    werase(window_);
  }
  cleared_ = false;

  box(window_, 0, 0);
  mvwprintw(window_, 0, width_ / 2 - 3, " nodes ");

  size_t start_idx = cursor;
  if (start_idx >= getContentHeight()) {
    start_idx -= getContentHeight();
  }
  else {
    start_idx = 0;
  }

  std::vector<std::pair<std::string, NodeData>> nodes;
  for (const auto& node: filter_.nodes()) {
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
    bool selected = !node_data.exclude;

    std::string text = name + ": " + std::to_string(node_data.count);

    if (selected) {
      wattron(window_, A_REVERSE);
    }

    if (hover) {
      wattron(window_, A_BOLD);
    }

    max_length_ = std::max(max_length_, text.size());

    if (shift_ >= text.size()) {
      text.clear();
    }
    else if (shift_ > 0) {
      text.erase(0, shift_ + 2);
    }

    bool cropped = false;
    if (text.size() > getContentWidth()) {
      text.resize(getContentWidth() - 2);
      cropped = true;
    }

    mvwprintw(window_, i + 1, 1, text.c_str());

    if (hover) {
      wattroff(window_, A_BOLD);
    }

    if (selected) {
      wattroff(window_, A_REVERSE);
    }

    if (shift_  > 0) {
      mvwprintw(window_, i + 1, 1, "  ");
      wattron(window_, A_REVERSE);
      mvwprintw(window_, i + 1, 1, "<");
      wattroff(window_, A_REVERSE);
    }
    if (cropped) {
      mvwprintw(window_, i + 1, getContentWidth() - 1, "  ");
      wattron(window_, A_REVERSE);
      mvwprintw(window_, i + 1, getContentWidth(), ">");
      wattroff(window_, A_REVERSE);
    }
  }

  last_cursor_ = cursor;

  drawScrollBar(getContentSize(), getContentHeight(), 1, width_ - 2);
}

bool NodePanel::handleKey(int key) {
  if (hidden()) {
    return false;
  }

  if (key == ctrl('a')) {
      filter_.selectAllNodes();

      return true;
  }
  else if (key == ctrl('i')) {
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
    }
    else {
      start_idx = 0;
    }

    size_t index = start_idx + row;
    if (index >= filter_.nodes().size()) {
      return true;
    }

    std::vector<std::pair<std::string, NodeData>> nodes;
    for (const auto& node: filter_.nodes()) {
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
  for (const auto& node: filter_.nodes()) {
    nodes.push_back(node);
    if (node.first == selected_) {
      selection = idx;
    }
    idx++;
  }

  if (selection < 0 || index == 0) {
    selection = 0;
  }
  else {
    selection += offset;
    selection = std::max(static_cast<int64_t>(0), std::min(static_cast<int64_t>(getContentSize()) - 1, selection));
  }
  selected_ = nodes[selection].first;

  size_t start_idx = cursor;
  if (start_idx >= getContentHeight()) {
    start_idx -= getContentHeight();
  }
  else {
    start_idx = 0;
  }

  if (selection < start_idx) {
    setCursor(selection + getContentHeight());
  }
  else if (selection >= start_idx + getContentHeight())
  {
    setCursor(selection + 1);
  }
}

int NodePanel::getContentWidth() const {
  int width = width_ - 2;
  if (getContentSize() > getContentHeight()) {
    width--;
  }
  return width;
}

void NodePanel::select() {
  filter_.toggleNode(selected_);
}

} // namespace log_view