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

#ifndef LOG_VIEW_HELP_PANEL_H_
#define LOG_VIEW_HELP_PANEL_H_

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
  virtual void refresh();
  virtual void resize(int height, int width, int y, int x);
  virtual bool handleMouse(const MEVENT& event) { return !hidden(); }
  virtual bool handleKey(int key);

  protected:
  virtual bool canNavigate() const { return !hidden(); }

  void printKeybinding(const HelpText& text);

  std::vector<HelpText> keys_;
  size_t longest_key_ = 0;
  size_t longest_line_ = 0;
};
typedef std::shared_ptr<HelpPanel> HelpPanelPtr;

}  // namespace log_view

#endif  // LOG_VIEW_HELP_PANEL_H_