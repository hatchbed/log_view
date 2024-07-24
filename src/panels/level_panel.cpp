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

#include <log_view/panels/level_panel.h>

namespace log_view {

void LevelPanel::refresh() {
  wattron(window_, A_REVERSE);
  wattron(window_, A_BOLD);
  std::string clear(width_, ' ');
  mvwprintw(window_, 0, 0, "%s", clear.c_str());
  mvwprintw(window_, 0, 0, " debug  info  warn  error  fatal      all nodes");
  wattroff(window_, A_BOLD);
  mvwprintw(window_, 0, width_ - 17, "CTRL+h: view help");
  wattroff(window_, A_REVERSE);

  wattron(window_, COLOR_PAIR(CP_DEFAULT_GREY));
  if (!filter_.getDebugLevel()) {
    mvwprintw(window_, 0, 0, " debug ");
  }
  if (!filter_.getInfoLevel()) {
    mvwprintw(window_, 0, 7, " info ");
  }
  if (!filter_.getWarnLevel()) {
    mvwprintw(window_, 0, 13, " warn ");
  }
  if (!filter_.getErrorLevel()) {
    mvwprintw(window_, 0, 19, " error ");
  }
  if (!filter_.getFatalLevel()) {
    mvwprintw(window_, 0, 26, " fatal ");
  }
  if (filter_.getEnableNodeFilter()) {
    mvwprintw(window_, 0, 37, " all nodes ");
  }
  wattroff(window_, COLOR_PAIR(CP_DEFAULT_GREY));
}

bool LevelPanel::handleMouse(const MEVENT& event) {
  if (hidden() || !encloses(event.y, event.x)) {
    return false;
  }

  if (event.bstate & BUTTON1_PRESSED) {
    if (event.x < 7) {
      toggleDebug();
    }
    else if (event.x < 13) {
      toggleInfo();
    }
    else if (event.x < 19) {
      toggleWarn();
    }
    else if (event.x < 26) {
      toggleError();
    }
    else if (event.x < 33) {
      toggleFatal();
    }
    else if (event.x > 36 && event.x < 48) {
      toggleAllNodes();
    }
  }
  return true;
}

void LevelPanel::toggleDebug() {
  filter_.setDebugLevel(!filter_.getDebugLevel());
  refresh();
}

void LevelPanel::toggleInfo() {
  filter_.setInfoLevel(!filter_.getInfoLevel());
  refresh();
}

void LevelPanel::toggleWarn() {
  filter_.setWarnLevel(!filter_.getWarnLevel());
  refresh();
}

void LevelPanel::toggleError() {
  filter_.setErrorLevel(!filter_.getErrorLevel());
  refresh();
}

void LevelPanel::toggleFatal() {
  filter_.setFatalLevel(!filter_.getFatalLevel());
  refresh();
}

void LevelPanel::toggleAllNodes() {
  filter_.setEnableNodeFilter(!filter_.getEnableNodeFilter());
  refresh();
}

} // namespace log_view
