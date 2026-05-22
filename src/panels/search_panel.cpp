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

#include <log_view/panels/search_panel.h>

#include <log_view/utils.h>

namespace log_view {

void SearchPanel::refresh() {
  if (show_results_ && !edit_mode_) {
    wattron(window_, kAttrGreyBg);
    std::string background(width_, ' ');
    mvwprintw(window_, 0, 0, "%s", background.c_str());
    wattroff(window_, kAttrGreyBg);

    std::string label = "match: ";
    std::string search = filter_.getSearch();
    printStyledAt(window_, 0, 0, kAttrGreyBg | (focus_ ? A_BOLD : 0), "%s", label.c_str());
    wattron(window_, kAttrGreyBg);
    mvwprintw(window_, 0, static_cast<int>(label.length()), "%s", search.c_str());

    if (focus_) {
      std::string help = "  Enter: next  Backspace: prev  CTRL-x: clear";
      size_t text_len = label.length() + search.length();
      if (help.length() + text_len <= static_cast<size_t>(width_)) {
        mvwprintw(window_, 0, width_ - static_cast<int>(help.length()), "%s", help.c_str());
      }
    }

    wattroff(window_, kAttrGreyBg);
  } else if (edit_mode_) {
    wattron(window_, kAttrGreyBg);
    std::string background(width_, ' ');
    mvwprintw(window_, 0, 0, "%s", background.c_str());
    wattroff(window_, kAttrGreyBg);

    printStyledAt(window_, 0, 0, kAttrGreyBg | A_BOLD, "search: ");
    wattron(window_, kAttrGreyBg);
    mvwprintw(window_, 0, inputOffset(), "%s", input_text_.c_str());

    if (focus_) {
      std::string help = "  Enter: search  CTRL-s: cancel  CTRL-x: clear";
      size_t text_len = static_cast<size_t>(inputOffset()) + input_text_.length();
      if (help.length() + text_len <= static_cast<size_t>(width_)) {
        mvwprintw(window_, 0, width_ - static_cast<int>(help.length()), "%s", help.c_str());
      }
    }

    wattroff(window_, kAttrGreyBg);
  } else {
    printStyledAt(window_, 0, 0, focus() ? A_BOLD : 0, "search: ");
    mvwprintw(window_, 0, inputOffset(), "%s", input_text_.c_str());
  }
}

bool SearchPanel::handleInput(int val) {
  if (!focus_) {
    return false;
  }

  if (edit_mode_) {
    if (val == KEY_ENTER_VAL) {
      if (!input_text_.empty()) {
        filter_.search(input_text_);
        if (on_search_) {
          on_search_();
        }
      }
      edit_mode_ = false;
      input_text_.clear();
      input_loc_ = -1;
      forceRefresh();
      return true;
    } else if (val == ctrl('s')) {
      edit_mode_ = false;
      input_text_.clear();
      input_loc_ = -1;
      forceRefresh();
      return true;
    }
    return PanelInterface::handleInput(val);
  }

  if (show_results_) {
    if (val == KEY_ENTER_VAL) {
      filter_.nextMatch();
      return true;
    } else if (val == KEY_BACKSPACE) {
      filter_.prevMatch();
      return true;
    }
    return false;
  }

  if (val == KEY_ENTER_VAL) {
    if (input_text_.empty()) {
      hide(true);
      setFocus(false);
      return true;
    }

    filter_.search(input_text_);
    if (on_search_) {
      on_search_();
    }
    show_results_ = true;
    input_text_.clear();
    input_loc_ = -1;
    forceRefresh();
    return true;
  }

  return PanelInterface::handleInput(val);
}

void SearchPanel::clearSearch() {
  if (show_results_) {
    filter_.clearSearch();
    input_text_.clear();
    edit_mode_ = false;
    setFocus(false);
    show_results_ = false;
    hide(true);
  }
}

void SearchPanel::toggle() {
  if (!hidden_ && show_results_ && !edit_mode_) {
    edit_mode_ = true;
    input_text_ = filter_.getSearch();
    input_loc_ = -1;
    setFocus(true);
    forceRefresh();
  } else if (!hidden_ && show_results_ && edit_mode_) {
    edit_mode_ = false;
    input_text_.clear();
    input_loc_ = -1;
    forceRefresh();
  } else if (!hidden_) {
    hide(true);
  } else {
    show_results_ = false;
    edit_mode_ = false;
    setFocus(true);
    hide(false);
  }
}

}  // namespace log_view
