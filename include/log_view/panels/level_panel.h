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

#include <functional>
#include <memory>
#include <utility>

#include <log_view/log_filter.h>
#include <log_view/panel_interface.h>

namespace log_view {

class LevelPanel : public PanelInterface {
  public:
  LevelPanel(int height, int width, int y, int x, LogFilter& filter)
  : PanelInterface(height, width, y, x), filter_(filter) {}
  virtual ~LevelPanel() {}
  virtual void refresh();

  virtual void toggleDebug();
  virtual void toggleInfo();
  virtual void toggleWarn() ;
  virtual void toggleError();
  virtual void toggleFatal();
  virtual void toggleAllNodes();
  virtual bool handleMouse(const MEVENT& event);

  void setShowInvertHintCallback(std::function<bool()> cb) {
    show_invert_hint_ = std::move(cb);
  }

  void setHelpOpenCallback(std::function<bool()> cb) {
    help_open_ = std::move(cb);
  }

  void setBagMode(bool bag_mode) { bag_mode_ = bag_mode; }

  void setBagPanelOpenCallback(std::function<bool()> cb) {
    bag_panel_open_ = std::move(cb);
  }

  void setShowBagInvertHintCallback(std::function<bool()> cb) {
    show_bag_invert_hint_ = std::move(cb);
  }

  protected:
  LogFilter& filter_;
  std::function<bool()> show_invert_hint_;
  std::function<bool()> show_bag_invert_hint_;
  std::function<bool()> help_open_;
  std::function<bool()> bag_panel_open_;
  bool bag_mode_ = false;
};
using LevelPanelPtr = std::shared_ptr<LevelPanel>;

}  // namespace log_view
