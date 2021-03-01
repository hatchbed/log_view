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

#ifndef LOG_VIEW_PANEL_INTERFACE_H_
#define LOG_VIEW_PANEL_INTERFACE_H_

#include <memory>
#include <string>

#include <curses.h>
#include <panel.h>

#define KEY_ENTER_VAL 10

namespace log_view {

class PanelInterface {
  public:
  PanelInterface(int height, int width, int y, int x);
  virtual ~PanelInterface();
  virtual void refresh() = 0;
  virtual void forceRefresh();
  virtual void resize(int height, int width, int y, int x);
  virtual bool handleInput(int key);
  virtual bool handleNavigation(int key);
  virtual bool handleMouse(const MEVENT& event) { return false; }
  virtual bool handleKey(int key) { return false; }
  virtual bool encloses(int y, int x);

  virtual void hide(bool enable);
  virtual bool setFocus(bool enable);
  virtual void toTop();
  virtual bool setCursor();

  virtual int x() const;
  virtual int y() const;
  virtual int width() const;
  virtual int height() const;
  virtual bool hidden() const;
  virtual bool visible() const;
  virtual bool focus() const;
  virtual bool scrollbar() const;

  protected:
  virtual bool canFocus() const { return false; }
  virtual void drawScrollBar(size_t count, int height, int y, int x);

  // text input
  virtual bool canInput() const { return false; }
  virtual void activate(bool enable) {}
  virtual int inputOffset() const { return 0; }

  // navigation
  virtual bool canNavigate() const { return false; }
  virtual bool canSelect() const { return false; }
  virtual size_t getContentSize() const { return 0; }
  virtual int getContentHeight() const { return height_; }
  virtual int getContentWidth() const { return width_; }
  virtual void setCursor(int64_t cursor) {}
  virtual int64_t getCursor() const { return 0; }
  virtual void follow(bool enable);
  virtual void pageUp();
  virtual void pageDown();
  virtual void move(int step);
  virtual void moveTo(size_t index);
  virtual void shift(int cols);
  virtual void select() {};
  virtual bool following() { return getCursor() < 0; }

  WINDOW* window_ = nullptr;
  PANEL* panel_ = nullptr;
  int x_;
  int y_;
  int width_;
  int height_;
  bool cleared_ = false;
  bool hidden_ = false;

  // text input
  bool focus_ = false;
  std::string input_text_;
  int input_loc_ = -1;

  // navigation
  bool follow_ = true;
  size_t last_content_size_ = 0;
  int64_t last_cursor_ = 0;
  size_t max_length_ = 0;
  int shift_ = 0;
};
typedef std::shared_ptr<PanelInterface> PanelInterfacePtr;

}  // namespace log_view

#endif  // LOG_VIEW_PANEL_INTERFACE_H_