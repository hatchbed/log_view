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
#include <string>

#include <log_view/panel_interface.h>
#include <log_view/preferences.h>

namespace log_view {

class PrefsPanel : public PanelInterface {
public:
  PrefsPanel(int height, int width, int y, int x, Preferences& prefs);
  virtual ~PrefsPanel() {}
  virtual void refresh();
  virtual bool handleKey(int key);
  bool handleMouse(const MEVENT& event) override { return !hidden(); }

  void setOnSave(std::function<void()> cb) { on_save_ = cb; }

protected:
  virtual bool canFocus() const { return false; }
  bool canNavigate() const override { return !hidden(); }
  void activate(bool enable) override;

private:
  void cycleTimestampFormat(int direction);
  void cycleRotateSize(int direction);
  void cycleMaxSize(int direction);
  bool isFieldEnabled(int field) const;
  static std::string formatSize(size_t bytes);

  Preferences& prefs_;
  Preferences pending_;
  int selected_ = 0;
  std::function<void()> on_save_;

  static constexpr int kNumFields       = 5;
  static constexpr int kFieldTimestamp  = 0;
  static constexpr int kFieldPersist    = 1;
  static constexpr int kFieldPersistLogs = 2;
  static constexpr int kFieldRotateSize = 3;
  static constexpr int kFieldMaxSize    = 4;

  static constexpr size_t kRotateSizePresets[] = {
    1ul * 1024 * 1024,
    5ul * 1024 * 1024,
    10ul * 1024 * 1024,
    25ul * 1024 * 1024,
    50ul * 1024 * 1024,
    100ul * 1024 * 1024
  };
  static constexpr size_t kMaxSizePresets[] = {
    10ul * 1024 * 1024,
    50ul * 1024 * 1024,
    100ul * 1024 * 1024,
    250ul * 1024 * 1024,
    500ul * 1024 * 1024,
    1024ul * 1024 * 1024
  };
  static constexpr int kRotateSizeCount = 6;
  static constexpr int kMaxSizeCount    = 6;
};
typedef std::shared_ptr<PrefsPanel> PrefsPanelPtr;

}  // namespace log_view
