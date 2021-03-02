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

#ifndef LOG_VIEW_NODE_PANEL_H_
#define LOG_VIEW_NODE_PANEL_H_

#include <log_view/log_filter.h>
#include <log_view/panel_interface.h>

namespace log_view {

class NodePanel : public PanelInterface {
  public:
  NodePanel(int height, int width, int y, int x, LogFilter& filter) : PanelInterface(height, width, y, x), filter_(filter) {}
  virtual ~NodePanel() {}
  virtual void refresh();
  virtual bool handleMouse(const MEVENT& event);
  virtual bool handleKey(int key);

  protected:
  virtual bool canFocus() const { return true; }
  virtual bool canNavigate() const { return true; }
  virtual bool canSelect() const { return true; }
  virtual size_t getContentSize() const { return filter_.nodes().size(); }
  virtual int getContentHeight() const { return height_ - 2; }
  virtual int getContentWidth() const;
  virtual void follow(bool enable);
  virtual void moveTo(size_t index);
  virtual void setCursor(int64_t cursor) { cursor_ = cursor; }
  virtual int64_t getCursor() const { return cursor_; }
  virtual void select();

  size_t cursor_ = 0;
  std::string selected_;

  LogFilter& filter_;
};
typedef std::shared_ptr<NodePanel> NodePanelPtr;

}  // namespace log_view

#endif  // LOG_VIEW_NODE_PANEL_H_