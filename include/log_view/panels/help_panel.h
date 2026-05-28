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

#pragma once

#include <memory>
#include <string>
#include <vector>

#include <log_view/panel_interface.h>

namespace log_view {

struct HelpText {
  int line;
  std::string key;
  std::string description;
};

class HelpPanel : public PanelInterface {
  public:
  HelpPanel(int height, int width, int y, int x);
  virtual ~HelpPanel() {}
  void refresh() override;
  void resize(int height, int width, int y, int x) override;
  bool handleMouse(const MEVENT& event) override { return !hidden(); }
  bool handleKey(int key) override;

  protected:
  bool canNavigate() const override { return !hidden(); }
  size_t getContentSize() const override;
  int getContentHeight() const override;
  void setCursor(int64_t cursor) override { cursor_ = cursor; }
  int64_t getCursor() const override { return cursor_; }

  void printKeybinding(const HelpText& text, int scroll_top);

  std::vector<HelpText> keys_;
  size_t longest_key_ = 0;
  size_t longest_line_ = 0;
  int64_t cursor_ = -1;
};
using HelpPanelPtr = std::shared_ptr<HelpPanel>;

}  // namespace log_view
