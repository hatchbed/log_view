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

#pragma once

#include <algorithm>
#include <memory>

#include <log_view/log_filter.h>
#include <log_view/panel_interface.h>

namespace log_view {

class DetailsPanel : public PanelInterface {
  public:
  DetailsPanel(int height, int width, int y, int x, const LogFilter& filter)
  : PanelInterface(height, width, y, x), filter_(filter) {}
  virtual ~DetailsPanel() {}
  virtual void refresh();
  virtual bool handleNavigation(int key);

  protected:
  virtual bool canNavigate() const { return !hidden(); }
  virtual size_t getContentSize() const { return content_size_; }
  virtual int getContentHeight() const { return std::max(1, height_ - 2); }
  virtual int getContentWidth() const;
  virtual void setCursor(int64_t cursor) { cursor_ = cursor; }
  virtual int64_t getCursor() const { return cursor_; }

  const LogFilter& filter_;
  mutable size_t content_size_ = 6;
  int64_t cursor_ = -1;
  int64_t last_selected_ = -2;
};
typedef std::shared_ptr<DetailsPanel> DetailsPanelPtr;

}  // namespace log_view
