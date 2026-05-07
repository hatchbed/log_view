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

#include <algorithm>
#include <string>

#include <log_view/utils.h>

namespace log_view {

constexpr size_t PrefsPanel::kRotateSizePresets[];
constexpr size_t PrefsPanel::kMaxSizePresets[];

PrefsPanel::PrefsPanel(int height, int width, int y, int x, Preferences& prefs)
: PanelInterface(height, width, y, x), prefs_(prefs), pending_(prefs) {}

void PrefsPanel::activate(bool enable) {
  if (enable) {
    pending_ = prefs_;
    if (prefs_.workspace_dir.empty()) {
      pending_.persist_logs    = false;
      pending_.persist_filters = false;
    }
    selected_ = 0;
    scroll_top_ = 0;
    ensureSelectedVisible();
    werase(window_);
    refresh();
  }
}

void PrefsPanel::refresh() {
  werase(window_);
  box(window_, 0, 0);

  int title_x = std::max(1, width_ / 2 - 7);
  mvwprintw(window_, 0, title_x, " preferences ");

  int max_scroll = std::max(0, static_cast<int>(getContentSize()) - getContentHeight());
  scroll_top_ = std::max(0, std::min(max_scroll, scroll_top_));

  bool workspace_ok = !prefs_.workspace_dir.empty();
  int path_max = width_ - 6;
  int bot = height_ - 4;

  // Returns the display row for logical content row r, or -1 if clipped.
  auto vis = [&](int r) -> int {
    int d = r - scroll_top_;
    return (d >= 2 && d <= bot) ? d : -1;
  };

  int dr;

  // --- Timestamp Format ---
  if ((dr = vis(2)) >= 0) { mvwprintw(window_, dr, 3, "Timestamp Format"); }

  const char* fmt_str = "seconds";
  if (pending_.timestamp_format == Preferences::TimestampFormat::ELAPSED) {
    fmt_str = "elapsed";
  } else if (pending_.timestamp_format == Preferences::TimestampFormat::TIME_OF_DAY) {
    fmt_str = "time of day";
  }

  bool ts_focused = (selected_ == kFieldTimestamp);
  if ((dr = vis(3)) >= 0) {
    if (ts_focused) { wattron(window_, A_REVERSE); }
    mvwprintw(window_, dr, 5, "%-14s", fmt_str);
    if (ts_focused) { wattroff(window_, A_REVERSE); }
    if (!ts_focused) { wattron(window_, kAttrGrey); }
    mvwprintw(window_, dr, 20, "< >");
    if (!ts_focused) { wattroff(window_, kAttrGrey); }
  }

  // --- Persist Filter Settings ---
  if ((dr = vis(5)) >= 0) {
    if (!workspace_ok) { wattron(window_, kAttrGrey); }
    mvwprintw(window_, dr, 3, "Persist Filter Settings");
    if (!workspace_ok) { wattroff(window_, kAttrGrey); }
  }

  if ((dr = vis(6)) >= 0) {
    if (workspace_ok) {
      std::string path = prefs_.workspace_dir + "/preferences.yaml";
      if (static_cast<int>(path.size()) > path_max) {
        path = "..." + path.substr(path.size() - (path_max - 3));
      }
      wattron(window_, kAttrGrey);
      mvwprintw(window_, dr, 3, "%-*s", path_max, path.c_str());
      wattroff(window_, kAttrGrey);
    } else {
      wattron(window_, COLOR_PAIR(CP_YELLOW));
      std::string msg = prefs_.workspace_error;
      if (static_cast<int>(msg.size()) > path_max) { msg = msg.substr(0, path_max); }
      mvwprintw(window_, dr, 3, "%-*s", path_max, msg.c_str());
      wattroff(window_, COLOR_PAIR(CP_YELLOW));
    }
  }

  bool pf_focused = workspace_ok && (selected_ == kFieldPersist);
  if ((dr = vis(7)) >= 0) {
    if (!workspace_ok) { wattron(window_, kAttrGrey); }
    if (pf_focused) { wattron(window_, A_REVERSE); }
    mvwprintw(window_, dr, 5, "%-14s", pending_.persist_filters ? "yes" : "no");
    if (pf_focused) { wattroff(window_, A_REVERSE); }
    if (!pf_focused) { wattron(window_, kAttrGrey); }
    mvwprintw(window_, dr, 20, "< >");
    if (!workspace_ok || !pf_focused) { wattroff(window_, kAttrGrey); }
  }

  // --- Persist Logs to Disk ---
  if ((dr = vis(9)) >= 0) {
    if (!workspace_ok) { wattron(window_, kAttrGrey); }
    mvwprintw(window_, dr, 3, "Persist Logs to Disk");
    if (!workspace_ok) { wattroff(window_, kAttrGrey); }
  }

  if ((dr = vis(10)) >= 0) {
    if (workspace_ok) {
      std::string path = prefs_.workspace_dir + "/";
      if (static_cast<int>(path.size()) > path_max) {
        path = "..." + path.substr(path.size() - (path_max - 3));
      }
      wattron(window_, kAttrGrey);
      mvwprintw(window_, dr, 3, "%-*s", path_max, path.c_str());
      wattroff(window_, kAttrGrey);
    } else {
      wattron(window_, COLOR_PAIR(CP_YELLOW));
      std::string msg = prefs_.workspace_error;
      if (static_cast<int>(msg.size()) > path_max) { msg = msg.substr(0, path_max); }
      mvwprintw(window_, dr, 3, "%-*s", path_max, msg.c_str());
      wattroff(window_, COLOR_PAIR(CP_YELLOW));
    }
  }

  bool pl_focused = workspace_ok && (selected_ == kFieldPersistLogs);
  if ((dr = vis(11)) >= 0) {
    if (!workspace_ok) { wattron(window_, kAttrGrey); }
    if (pl_focused) { wattron(window_, A_REVERSE); }
    mvwprintw(window_, dr, 5, "%-14s", pending_.persist_logs ? "yes" : "no");
    if (pl_focused) { wattroff(window_, A_REVERSE); }
    if (!pl_focused) { wattron(window_, kAttrGrey); }
    mvwprintw(window_, dr, 20, "< >");
    if (!workspace_ok || !pl_focused) { wattroff(window_, kAttrGrey); }
  }

  // --- Log Rotate Size ---
  bool size_enabled = workspace_ok && pending_.persist_logs;
  bool rs_focused = size_enabled && (selected_ == kFieldRotateSize);
  std::string rotate_str = formatSize(pending_.log_rotate_size);

  if ((dr = vis(13)) >= 0) {
    if (!size_enabled) { wattron(window_, kAttrGrey); }
    mvwprintw(window_, dr, 3, "Log Rotate Size");
    if (!size_enabled) { wattroff(window_, kAttrGrey); }
  }

  if ((dr = vis(14)) >= 0) {
    if (rs_focused) {
      wattron(window_, A_REVERSE);
    } else if (!size_enabled) {
      wattron(window_, kAttrGrey);
    }
    mvwprintw(window_, dr, 5, "%-14s", rotate_str.c_str());
    if (rs_focused) { wattroff(window_, A_REVERSE); }
    if (!rs_focused) { wattron(window_, kAttrGrey); }
    mvwprintw(window_, dr, 20, "< >");
    if (!rs_focused) { wattroff(window_, kAttrGrey); }
  }

  // --- Max Total Log Size ---
  bool ms_focused = size_enabled && (selected_ == kFieldMaxSize);
  std::string max_str = formatSize(pending_.log_max_size);

  if ((dr = vis(16)) >= 0) {
    if (!size_enabled) { wattron(window_, kAttrGrey); }
    mvwprintw(window_, dr, 3, "Max Total Log Size");
    if (!size_enabled) { wattroff(window_, kAttrGrey); }
  }

  if ((dr = vis(17)) >= 0) {
    if (ms_focused) {
      wattron(window_, A_REVERSE);
    } else if (!size_enabled) {
      wattron(window_, kAttrGrey);
    }
    mvwprintw(window_, dr, 5, "%-14s", max_str.c_str());
    if (ms_focused) { wattroff(window_, A_REVERSE); }
    if (!ms_focused) { wattron(window_, kAttrGrey); }
    mvwprintw(window_, dr, 20, "< >");
    if (!ms_focused) { wattroff(window_, kAttrGrey); }
  }

  // --- Show Session Boundaries ---
  bool sb_focused = (selected_ == kFieldSessionBound);
  if ((dr = vis(19)) >= 0) { mvwprintw(window_, dr, 3, "Show Session Boundaries"); }
  if ((dr = vis(20)) >= 0) {
    if (sb_focused) { wattron(window_, A_REVERSE); }
    mvwprintw(window_, dr, 5, "%-14s", pending_.show_session_boundaries ? "yes" : "no");
    if (sb_focused) { wattroff(window_, A_REVERSE); }
    if (!sb_focused) { wattron(window_, kAttrGrey); }
    mvwprintw(window_, dr, 20, "< >");
    if (!sb_focused) { wattroff(window_, kAttrGrey); }
  }

  drawScrollBar(getContentSize(), getContentHeight(), 2, width_ - 1);

  // --- Controls hint (fixed position) ---
  wattron(window_, kAttrGrey);
  mvwprintw(window_, height_ - 3, 3, "Up/Dn: select   Space/</>: change");
  mvwprintw(window_, height_ - 2, 3, "Enter: save          Esc: cancel");
  wattroff(window_, kAttrGrey);
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
    int next = selected_ - 1;
    while (next > 0 && !isFieldEnabled(next)) next--;
    if (next >= 0) selected_ = next;
    ensureSelectedVisible();
  } else if (key == KEY_DOWN) {
    int next = selected_ + 1;
    while (next < kNumFields && !isFieldEnabled(next)) next++;
    if (next < kNumFields) selected_ = next;
    ensureSelectedVisible();
  } else if (key == KEY_HOME) {
    selected_ = 0;
    scroll_top_ = 0;
  } else if (key == KEY_END) {
    selected_ = kNumFields - 1;
    while (selected_ > 0 && !isFieldEnabled(selected_)) selected_--;
    int max_s = std::max(0, static_cast<int>(getContentSize()) - getContentHeight());
    scroll_top_ = max_s;
  } else if (key == KEY_PPAGE) {
    scroll_top_ = std::max(0, scroll_top_ - getContentHeight());
    snapSelectionToViewport(true);
  } else if (key == KEY_NPAGE) {
    int max_s = std::max(0, static_cast<int>(getContentSize()) - getContentHeight());
    scroll_top_ = std::min(max_s, scroll_top_ + getContentHeight());
    snapSelectionToViewport(false);
  } else if (key == KEY_LEFT) {
    if (selected_ == kFieldTimestamp) {
      cycleTimestampFormat(-1);
    } else if (selected_ == kFieldPersist) {
      pending_.persist_filters = !pending_.persist_filters;
    } else if (selected_ == kFieldPersistLogs) {
      pending_.persist_logs = !pending_.persist_logs;
      if (!pending_.persist_logs && (selected_ == kFieldRotateSize || selected_ == kFieldMaxSize)) {
        selected_ = kFieldPersistLogs;
      }
    } else if (selected_ == kFieldRotateSize) {
      cycleRotateSize(-1);
    } else if (selected_ == kFieldMaxSize) {
      cycleMaxSize(-1);
    } else if (selected_ == kFieldSessionBound) {
      pending_.show_session_boundaries = !pending_.show_session_boundaries;
    }
  } else if (key == KEY_RIGHT || key == ' ') {
    if (selected_ == kFieldTimestamp) {
      cycleTimestampFormat(1);
    } else if (selected_ == kFieldPersist) {
      pending_.persist_filters = !pending_.persist_filters;
    } else if (selected_ == kFieldPersistLogs) {
      pending_.persist_logs = !pending_.persist_logs;
      if (!pending_.persist_logs && (selected_ == kFieldRotateSize || selected_ == kFieldMaxSize)) {
        selected_ = kFieldPersistLogs;
      }
    } else if (selected_ == kFieldRotateSize) {
      cycleRotateSize(1);
    } else if (selected_ == kFieldMaxSize) {
      cycleMaxSize(1);
    } else if (selected_ == kFieldSessionBound) {
      pending_.show_session_boundaries = !pending_.show_session_boundaries;
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

void PrefsPanel::cycleRotateSize(int direction) {
  int idx = 0;
  for (int i = 0; i < kRotateSizeCount; ++i) {
    if (kRotateSizePresets[i] == pending_.log_rotate_size) {
      idx = i;
      break;
    }
  }
  idx = (idx + direction + kRotateSizeCount) % kRotateSizeCount;
  pending_.log_rotate_size = kRotateSizePresets[idx];
}

void PrefsPanel::cycleMaxSize(int direction) {
  int idx = 0;
  for (int i = 0; i < kMaxSizeCount; ++i) {
    if (kMaxSizePresets[i] == pending_.log_max_size) {
      idx = i;
      break;
    }
  }
  idx = (idx + direction + kMaxSizeCount) % kMaxSizeCount;
  pending_.log_max_size = kMaxSizePresets[idx];
}

bool PrefsPanel::isFieldEnabled(int field) const {
  bool workspace_ok = !prefs_.workspace_dir.empty();
  if (field == kFieldPersist || field == kFieldPersistLogs) {
    return workspace_ok;
  }
  if (field == kFieldRotateSize || field == kFieldMaxSize) {
    return workspace_ok && pending_.persist_logs;
  }
  return true;
}

std::string PrefsPanel::formatSize(size_t bytes) {
  if (bytes >= 1024ul * 1024 * 1024) {
    return std::to_string(bytes / (1024ul * 1024 * 1024)) + " GB";
  } else if (bytes >= 1024 * 1024) {
    return std::to_string(bytes / (1024 * 1024)) + " MB";
  } else {
    return std::to_string(bytes / 1024) + " KB";
  }
}

void PrefsPanel::snapSelectionToViewport(bool prefer_top) {
  static constexpr int kHRows[] = {2, 5, 9, 13, 16, 19};
  static constexpr int kVRows[] = {3, 7, 11, 14, 17, 20};

  int lo = scroll_top_ + 2;
  int hi = scroll_top_ + (height_ - 4);

  if (prefer_top) {
    for (int i = 0; i < kNumFields; i++) {
      if (isFieldEnabled(i) && kHRows[i] >= lo && kVRows[i] <= hi) {
        selected_ = i;
        return;
      }
    }
  } else {
    for (int i = kNumFields - 1; i >= 0; i--) {
      if (isFieldEnabled(i) && kHRows[i] >= lo && kVRows[i] <= hi) {
        selected_ = i;
        return;
      }
    }
  }
  ensureSelectedVisible();
}

void PrefsPanel::ensureSelectedVisible() {
  static constexpr int kHeaderRows[] = {2, 5, 9, 13, 16, 19};
  static constexpr int kValueRows[]  = {3, 7, 11, 14, 17, 20};

  int view_height = getContentHeight();
  int header_row  = kHeaderRows[selected_];
  int value_row   = kValueRows[selected_];

  // Scroll up:
  // header_row - scroll_top_ >= 2  =>  scroll_top_ <= header_row - 2
  if (scroll_top_ > header_row - 2) {
    scroll_top_ = header_row - 2;
  }

  // Scroll down:
  // value_row - scroll_top_ <= height_-4  =>  scroll_top_ >= value_row - view_height - 1
  int min_scroll = std::max(0, value_row - view_height - 1);
  if (scroll_top_ < min_scroll) {
    scroll_top_ = min_scroll;
  }

  int max_scroll = std::max(0, static_cast<int>(getContentSize()) - view_height);
  scroll_top_ = std::max(0, std::min(max_scroll, scroll_top_));
}

}  // namespace log_view
