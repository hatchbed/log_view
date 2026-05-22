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

#include <log_view/panels/help_panel.h>

#include <cmath>

#include <log_view/utils.h>

namespace log_view {

// TODO(malban): handle case where terminal is too small

// TODO(malban): support show/hide timestamps

HelpPanel::HelpPanel(int height, int width, int y, int x) :
  PanelInterface(height, width, y, x),
  keys_({
    {2, "CTRL-c", "Exit log viewer"},
    {3, "CTRL-h", "Show/hide this help screen"},
    {5, "F1", "Show/hide debug level"},
    {6, "F2", "Show/hide info level"},
    {7, "F3", "Show/hide warning level"},
    {8, "F4", "Show/hide error level"},
    {9, "F5", "Show/hide fatal level"},
    {10, "F7", "Enable/disable node filter"},
    {11, "CTRL-a", "Select all"},
    {12, "CTRL-d", "Show/hide message details"},
    {13, "CTRL-n", "Show/hide node selection"},
    {14, "CTRL-s", "Search for matching string"},
    {15, "CTRL-e", "Enable/disable text exclude filter"},
    {16, "CTRL-f", "Enable/disable text include filter"},
    {17, "CTRL-k", "Show/hide preferences"},
    {18, "CTRL-r", "Clear message history"}})
{
  for (const auto& key : keys_) {
    longest_key_ = std::max(longest_key_, key.key.length());
  }

  for (const auto& key : keys_) {
    longest_line_ = std::max(longest_line_, longest_key_ + 11 + key.description.length());
  }
}

size_t HelpPanel::getContentSize() const {
  if (keys_.empty()) { return 0; }
  return keys_.back().line - 1;
}

int HelpPanel::getContentHeight() const {
  return std::max(1, height_ - 3);
}

void HelpPanel::refresh() {
  box(window_, 0, 0);
  mvwprintw(window_, 0, width_ / 2 - 3, " help ");

  int content_size = static_cast<int>(getContentSize());
  int view_height  = getContentHeight();
  int scroll_top   = 0;

  if (content_size > view_height) {
    int64_t cursor = getCursor();
    scroll_top = (cursor < 0)
      ? content_size - view_height
      : static_cast<int>(cursor) - view_height;
    scroll_top = std::max(0, scroll_top);
    drawScrollBar(content_size, view_height, 2, width_ - 1);
  }

  for (const auto& key : keys_) {
    printKeybinding(key, scroll_top);
  }
}

void HelpPanel::resize(int height, int width, int y, int x) {
  int w = width;
  if (w > longest_line_) {
    w = longest_line_;
    x += (width - w) / 2;
  }
  PanelInterface::resize(height, w, y, x);
}

bool HelpPanel::handleKey(int key) {
  if (hidden()) {
    return false;
  }

  if (key == ctrl('h') || key == 27) {
    hide(visible());
  } else if (key == KEY_RESIZE || key == ctrl('q') || key == ctrl('c')
             || key == KEY_UP || key == KEY_DOWN
             || key == KEY_PPAGE || key == KEY_NPAGE
             || key == KEY_HOME || key == KEY_END) {
    return false;
  }

  return true;
}

void HelpPanel::printKeybinding(const HelpText& help_text, int scroll_top) {
  int row = help_text.line - scroll_top;
  if (row < 2 || row > height_ - 2) {
    return;
  }

  mvwprintw(window_, row, 3, "%s", help_text.key.c_str());

  int max_size = std::max(0, width() - (static_cast<int>(longest_key_) + 10));
  auto desc = help_text.description;
  if (static_cast<int>(desc.size()) > max_size) {
    desc.resize(max_size);
  }
  mvwprintw(window_, row, longest_key_ + 8, "%s", desc.c_str());

  int line_start = help_text.key.length() + 4;
  int line_end = longest_key_ + 7;

  wattron(window_, kAttrGrey);
  mvwhline(window_, row, line_start, 0, line_end - line_start);
  wattroff(window_, kAttrGrey);
}

}  // namespace log_view
