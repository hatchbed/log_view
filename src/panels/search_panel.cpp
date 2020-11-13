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

#include <log_view/panels/search_panel.h>

namespace log_view {

void SearchPanel::refresh() {
  if (show_results_) {
    wattron(window_, COLOR_PAIR(CP_DEFAULT_GREY));
    std::string background(width_, ' ');
    mvwprintw(window_, 0, 0, background.c_str());
    std::string text = "match: " + filter_.getSearch();
    mvwprintw(window_, 0, 0, text.c_str());

    std::string help = "  Press Enter/Backspace to move forward/backward through search results";
    if (help.length() + text.length() <= width_) {
        mvwprintw(window_, 0, width_ - help.length(), help.c_str());
    }

    wattroff(window_, COLOR_PAIR(CP_DEFAULT_GREY));
  }
  else {
    mvwprintw(window_, 0, 0, "search: %s", input_text_.c_str());
  }
}

bool SearchPanel::handleInput(int val) {
  if (!canInput() || !focus_) {
    return false;
  }

  if (val == KEY_ENTER_VAL) {
    if (input_text_.empty()) {
      hide(true);
      setFocus(false);
      return true;
    }

    filter_.search(input_text_);
    show_results_ = true;
    input_text_.clear();
    input_loc_ = -1;
    setFocus(false);
    refresh();
    return true;
  }

  return PanelInterface::handleInput(val);
}

void SearchPanel::clearSearch() {
  if (show_results_) {
    show_results_ = false;
    filter_.clearSearch();
    input_text_.clear();
    setFocus(false);
    hide(true);
  }
}

void SearchPanel::toggle() {
  if (!hidden_ && show_results_) {
    input_text_.clear();
    input_loc_ = -1;
    show_results_ = false;
    setFocus(true);
  }
  else if (!hidden_) {
    hide(true);
  }
  else {
    show_results_ = false;
    setFocus(true);
    hide(false);
  }
}

} // namespace log_view