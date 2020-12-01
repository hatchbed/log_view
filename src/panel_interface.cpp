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

#include <log_view/panel_interface.h>

#include <cmath>

namespace log_view {

PanelInterface::PanelInterface(int height, int width, int y, int x) :
  x_(x),
  y_(y),
  width_(width),
  height_(height)
{
  window_ = newwin(height_, width_, y_, x_);
  panel_ = new_panel(window_);
}

PanelInterface::~PanelInterface() {
  delwin(window_);
}

void PanelInterface::forceRefresh() {
  werase(window_);
  cleared_ = true;
  refresh();
}

void PanelInterface::resize(int height, int width, int y, int x) {
  x_ = x;
  y_ = y;
  width_ = width;
  height_ = height;
  werase(window_);
  mvwin(window_, y_, x_);
  wresize(window_, height_, width_);
  cleared_ = true;
  refresh();
}

bool PanelInterface::handleInput(int val) {
  if (!canInput() || !focus_) {
    return false;
  }

  bool key_used = false;
  if (val < 256 && isprint(val)) {
    if (input_loc_ == -1 || input_loc_ >= input_text_.size()) {
      input_text_ += val;
    }
    else {
      input_loc_ = std::max(0, input_loc_);
      input_text_.insert(input_loc_, 1, static_cast<char>(val));
      input_loc_++;
    }
    key_used = true;
  }
  else if (!input_text_.empty() && val == KEY_BACKSPACE && (input_loc_ == -1 || input_loc_ > 0)) {
    if (input_loc_ == -1 || input_loc_ >= input_text_.size()) {
      input_text_.pop_back();
    }
    else {
      input_text_.erase(input_loc_ - 1, 1);
      input_loc_--;
    }
    key_used = true;
  }
  else if (!input_text_.empty() && val == KEY_DC && input_loc_ != -1) {
    input_text_.erase(input_loc_, 1);
    if (input_loc_ >= input_text_.size()) {
      input_loc_ = -1;
    }
    key_used = true;
  }
  else if (input_loc_ != 0 && val == KEY_LEFT) {
    if (input_loc_ == -1) {
      input_loc_ = input_text_.size();
    }
    input_loc_--;
    key_used = true;
  }
  else if (input_loc_ != -1 && val == KEY_RIGHT) {
    input_loc_++;
    if (input_loc_ >= input_text_.size()) {
      input_loc_ = -1;
    }
    key_used = true;
  }

  if (key_used) {
    // trigger underlying action of input (filtering, etc.)
    activate(true);

    // refresh display
    werase(window_);
    cleared_ = true;
    refresh();
  }

  return key_used;
}

bool PanelInterface::handleNavigation(int key) {
  if (!canNavigate() || hidden_ || (canFocus() && !focus())) {
    return false;
  }

  bool key_used = false;
  if (key == KEY_NPAGE) {
    pageDown();
    key_used = true;
  }
  else if (key == KEY_PPAGE) {
    pageUp();
    key_used = true;
  }
  else if (key == KEY_UP) {
    move(-1);
    key_used = true;
  }
  else if (key == KEY_DOWN) {
    move(1);
    key_used = true;
  }
  else if (key == KEY_END) {
    follow(true);
    key_used = true;
  }
  else if (key == KEY_HOME) {
    moveTo(0);
    key_used = true;
  }
  else if (key == KEY_LEFT) {
    shift(-5);
    key_used = true;
  }
  else if (key == KEY_RIGHT) {
    shift(5);
    key_used = true;
  }
  else if (canSelect() && key == ' ') {
    select();
    key_used = true;
  }

  if (key_used) {
    // refresh display
    werase(window_);
    cleared_ = true;
    refresh();
  }

  return key_used;
}

bool PanelInterface::encloses(int y, int x) {
  return y >= y_ && y < y_ + height_ && x >= x_ && x < x_ + width_;
}

void PanelInterface::hide(bool enable) {
  if (enable == hidden_) {
    return;
  }

  hidden_ = enable;
  if (hidden_) {
    hide_panel(panel_);
    activate(false);
    focus_ = false;
  }
  else {
    show_panel(panel_);
    activate(true);
    input_loc_ = -1;
    if (canFocus()) {
      focus_ = true;
    }
  }
}

bool PanelInterface::setFocus(bool enable) {
  focus_ = false;
  if (enable && !hidden_ && canFocus()) {
    focus_ = true;
  }

  return focus_;
}

bool PanelInterface::setCursor() {
  if (focus_ && canInput()) {
    int loc = input_loc_;
    if (loc == -1) {
      loc = input_text_.size();
    }

    wmove(window_, 0, inputOffset() + loc);
    show_panel(panel_);
    curs_set(1);
    return true;
  }
  return false;
}

void PanelInterface::drawScrollBar(size_t count, int height, int y, int x) {
  if (count <= height) {
    return;
  }

  mvwvline(window_, y, x, 0, height);

  int64_t cursor = getCursor();
  if (cursor < 0) {
    cursor = count;
  }

  if (count >= 2 * height) {
    float percent = std::max(0.0f, static_cast<float>((cursor - height)) / (count - height));
    int scroll_loc = std::round(percent * (height - 1));

    wattron(window_, A_REVERSE);
    mvwprintw(window_, y + scroll_loc, x, " ");
    wattroff(window_, A_REVERSE);
  }
  else {
    int size = 2 * height - count;
    cursor = std::max(static_cast<int64_t>(0), cursor - height);

    wattron(window_, A_REVERSE);
    for (size_t i = cursor; i < cursor + size; i++) {
      mvwprintw(window_, y + i, x, " ");
    }
    wattroff(window_, A_REVERSE);
  }
}

void PanelInterface::toTop() {
  top_panel(panel_);
}

int PanelInterface::x() const {
  return x_;
}

int PanelInterface::y() const {
  return y_;
}

int PanelInterface::width() const {
  return width_;
}

int PanelInterface::height() const {
  return height_;
}

bool PanelInterface::hidden() const {
  return hidden_;
}

bool PanelInterface::visible() const {
  return !hidden_;
}

bool PanelInterface::focus() const {
  return canFocus() && focus_;
}

bool PanelInterface::scrollbar() const {
  return getContentSize() > getContentHeight();
}

void PanelInterface::follow(bool enable) {
  if (enable && !following()) {
    last_content_size_ = 0;
    last_cursor_ = -1;
    setCursor(-1);
  }
  else if (!enable && following()) {
    setCursor(getContentSize());
  }
}

void PanelInterface::pageUp() {
  move(-(getContentHeight()));
}

void PanelInterface::pageDown() {
  move(getContentHeight());
}

void PanelInterface::move(int step) {
  if (step == 0 || getContentSize() == 0) {
    return;
  }

  if (following() && step > 0) {
    return;
  }

  if (following() && step < 0) {
    follow(false);
  }

  int64_t cursor = getCursor();
  int64_t dst = std::max(static_cast<int64_t>(0), cursor + step);

  moveTo(dst);
}

void PanelInterface::moveTo(size_t index) {
  size_t view_size = getContentHeight();
  if (getContentSize() <= view_size || index > getContentSize() - 1) {
    follow(true);
    return;
  }

  if (index < view_size) {
    index = view_size;
  }
  follow(false);
  setCursor(index);
}

void PanelInterface::shift(int cols) {
  int shift = shift_;
  if (cols < 0) {
    shift += cols;
    shift = std::max(0, shift);
  }
  else if (shift_ + getContentWidth() < max_length_) {
    shift += cols;
  }

  shift_ = shift;
}

}  // namespace log_view