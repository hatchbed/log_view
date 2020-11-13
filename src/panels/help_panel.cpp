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

#include <log_view/panels/help_panel.h>

#include <log_view/utils.h>

namespace log_view {

// TODO handle case where terminal is too small

// TODO support show/hide timestamps

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
    {11, "CTRL-a", "Select all log lines and copy to clipboard"},
    {12, "CTRL-n", "Show/hide node selection"},
    {13, "CTRL-s", "Search for matching string"},
    {14, "CTRL-x", "Clear search"},
    {15, "Backspace", "Prev match"},
    {16, "Enter", "Next match"},
    {17, "CTRL-e", "Enable/disable text exclude filter"},
    {18, "CTRL-f", "Enable/disable text include filter"}})
{
  for (const auto& key: keys_) {
    longest_key_ = std::max(longest_key_, key.key.length());
  }

  for (const auto& key: keys_) {
    longest_line_ = std::max(longest_line_, longest_key_ + 11 + key.description.length());
  }
}

void HelpPanel::refresh() {
  box(window_, 0, 0);
  mvwprintw(window_,  0, width_ / 2 - 3, " help ");
  for (const auto& key: keys_) {
    printKeybinding(key);
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

void HelpPanel::printKeybinding(const HelpText& help_text) {
    mvwprintw(window_,  help_text.line, 3, help_text.key.c_str());
    mvwprintw(window_,  help_text.line, longest_key_ + 8, help_text.description.c_str());

    int line_start = help_text.key.length() + 4;
    int line_end = longest_key_ + 7;

    wattron(window_, COLOR_PAIR(CP_GREY));
    mvwhline(window_, help_text.line, help_text.key.length() + 4, 0, line_end - line_start);
    wattroff(window_, COLOR_PAIR(CP_GREY));
}

} // namespace log_view