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

#include <log_view/panels/prefs_panel.h>

#include <log_view/utils.h>

namespace log_view {

PrefsPanel::PrefsPanel(int height, int width, int y, int x, Preferences& prefs)
: PanelInterface(height, width, y, x), prefs_(prefs), pending_(prefs) {}

void PrefsPanel::activate(bool enable) {
  if (enable) {
    pending_ = prefs_;
    selected_ = 0;
    werase(window_);
    refresh();
  }
}

void PrefsPanel::refresh() {
  box(window_, 0, 0);

  int title_x = std::max(1, width_ / 2 - 7);
  mvwprintw(window_, 0, title_x, " preferences ");

  // --- Timestamp Format field ---
  mvwprintw(window_, 2, 3, "Timestamp Format");

  const char* fmt_str = "seconds";
  if (pending_.timestamp_format == Preferences::TimestampFormat::ELAPSED) {
    fmt_str = "elapsed";
  } else if (pending_.timestamp_format == Preferences::TimestampFormat::TIME_OF_DAY) {
    fmt_str = "time of day";
  }

  if (selected_ == kFieldTimestamp) {
    wattron(window_, A_REVERSE);
  }
  mvwprintw(window_, 3, 5, "%-14s", fmt_str);
  if (selected_ == kFieldTimestamp) {
    wattroff(window_, A_REVERSE);
  }
  wattron(window_, COLOR_PAIR(CP_GREY));
  mvwprintw(window_, 3, 20, "< >");
  wattroff(window_, COLOR_PAIR(CP_GREY));

  // --- Persist Filter Settings field ---
  mvwprintw(window_, 5, 3, "Persist Filter Settings");

  const char* persist_str = pending_.persist_filters ? "yes" : "no";
  if (selected_ == kFieldPersist) {
    wattron(window_, A_REVERSE);
  }
  mvwprintw(window_, 6, 5, "%-14s", persist_str);
  if (selected_ == kFieldPersist) {
    wattroff(window_, A_REVERSE);
  }
  wattron(window_, COLOR_PAIR(CP_GREY));
  mvwprintw(window_, 6, 20, "< >");
  wattroff(window_, COLOR_PAIR(CP_GREY));

  // --- Controls hint ---
  wattron(window_, COLOR_PAIR(CP_GREY));
  mvwprintw(window_, height_ - 3, 3, "Up/Dn: select   Space/</>: change");
  mvwprintw(window_, height_ - 2, 3, "Enter: save          Esc: cancel");
  wattroff(window_, COLOR_PAIR(CP_GREY));
}

bool PrefsPanel::handleKey(int key) {
  if (hidden()) {
    return false;
  }

  if (key == KEY_RESIZE || key == ctrl('q') || key == ctrl('c')) {
    return false;
  }

  if (key == 27 /* ESC */) {
    pending_ = prefs_;
    hide(true);
  } else if (key == KEY_ENTER_VAL) {
    prefs_ = pending_;
    prefs_.save();
    if (on_save_) {
      on_save_();
    }
    hide(true);
  } else if (key == KEY_UP) {
    if (selected_ > 0) {
      selected_--;
    }
  } else if (key == KEY_DOWN) {
    if (selected_ < kNumFields - 1) {
      selected_++;
    }
  } else if (key == KEY_LEFT) {
    if (selected_ == kFieldTimestamp) {
      cycleTimestampFormat(-1);
    } else if (selected_ == kFieldPersist) {
      pending_.persist_filters = !pending_.persist_filters;
    }
  } else if (key == KEY_RIGHT || key == ' ') {
    if (selected_ == kFieldTimestamp) {
      cycleTimestampFormat(1);
    } else if (selected_ == kFieldPersist) {
      pending_.persist_filters = !pending_.persist_filters;
    }
  }

  if (!hidden()) {
    refresh();
  }
  return true;
}

void PrefsPanel::cycleTimestampFormat(int direction) {
  int current = static_cast<int>(pending_.timestamp_format);
  int count = 3;
  current = (current + direction + count) % count;
  pending_.timestamp_format = static_cast<Preferences::TimestampFormat>(current);
}

}  // namespace log_view
