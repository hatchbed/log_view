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
    werase(window_);
    refresh();
  }
}

void PrefsPanel::refresh() {
  box(window_, 0, 0);

  int title_x = std::max(1, width_ / 2 - 7);
  mvwprintw(window_, 0, title_x, " preferences ");

  bool workspace_ok = !prefs_.workspace_dir.empty();
  int path_max = width_ - 6;

  // --- Timestamp Format ---
  mvwprintw(window_, 2, 3, "Timestamp Format");

  const char* fmt_str = "seconds";
  if (pending_.timestamp_format == Preferences::TimestampFormat::ELAPSED) {
    fmt_str = "elapsed";
  } else if (pending_.timestamp_format == Preferences::TimestampFormat::TIME_OF_DAY) {
    fmt_str = "time of day";
  }

  bool ts_focused = (selected_ == kFieldTimestamp);
  if (ts_focused) { wattron(window_, A_REVERSE); }
  mvwprintw(window_, 3, 5, "%-14s", fmt_str);
  if (ts_focused) { wattroff(window_, A_REVERSE); }
  if (!ts_focused) { wattron(window_, COLOR_PAIR(CP_GREY)); }
  mvwprintw(window_, 3, 20, "< >");
  if (!ts_focused) { wattroff(window_, COLOR_PAIR(CP_GREY)); }

  // --- Persist Filter Settings ---
  if (!workspace_ok) { wattron(window_, COLOR_PAIR(CP_GREY)); }
  mvwprintw(window_, 5, 3, "Persist Filter Settings");
  if (!workspace_ok) { wattroff(window_, COLOR_PAIR(CP_GREY)); }

  if (workspace_ok) {
    std::string path = prefs_.workspace_dir + "/preferences.yaml";
    if (static_cast<int>(path.size()) > path_max) {
      path = "..." + path.substr(path.size() - (path_max - 3));
    }
    wattron(window_, COLOR_PAIR(CP_GREY));
    mvwprintw(window_, 6, 3, "%-*s", path_max, path.c_str());
    wattroff(window_, COLOR_PAIR(CP_GREY));
  } else {
    wattron(window_, COLOR_PAIR(CP_YELLOW));
    std::string msg = prefs_.workspace_error;
    if (static_cast<int>(msg.size()) > path_max) { msg = msg.substr(0, path_max); }
    mvwprintw(window_, 6, 3, "%-*s", path_max, msg.c_str());
    wattroff(window_, COLOR_PAIR(CP_YELLOW));
  }

  bool pf_focused = workspace_ok && (selected_ == kFieldPersist);
  if (!workspace_ok) { wattron(window_, COLOR_PAIR(CP_GREY)); }
  if (pf_focused) { wattron(window_, A_REVERSE); }
  mvwprintw(window_, 7, 5, "%-14s", pending_.persist_filters ? "yes" : "no");
  if (pf_focused) { wattroff(window_, A_REVERSE); }
  if (!pf_focused) { wattron(window_, COLOR_PAIR(CP_GREY)); }
  mvwprintw(window_, 7, 20, "< >");
  if (!workspace_ok || !pf_focused) { wattroff(window_, COLOR_PAIR(CP_GREY)); }

  // --- Persist Logs to Disk ---
  if (!workspace_ok) { wattron(window_, COLOR_PAIR(CP_GREY)); }
  mvwprintw(window_, 9, 3, "Persist Logs to Disk");
  if (!workspace_ok) { wattroff(window_, COLOR_PAIR(CP_GREY)); }

  if (workspace_ok) {
    std::string path = prefs_.workspace_dir + "/";
    if (static_cast<int>(path.size()) > path_max) {
      path = "..." + path.substr(path.size() - (path_max - 3));
    }
    wattron(window_, COLOR_PAIR(CP_GREY));
    mvwprintw(window_, 10, 3, "%-*s", path_max, path.c_str());
    wattroff(window_, COLOR_PAIR(CP_GREY));
  } else {
    wattron(window_, COLOR_PAIR(CP_YELLOW));
    std::string msg = prefs_.workspace_error;
    if (static_cast<int>(msg.size()) > path_max) { msg = msg.substr(0, path_max); }
    mvwprintw(window_, 10, 3, "%-*s", path_max, msg.c_str());
    wattroff(window_, COLOR_PAIR(CP_YELLOW));
  }

  bool pl_focused = workspace_ok && (selected_ == kFieldPersistLogs);
  if (!workspace_ok) { wattron(window_, COLOR_PAIR(CP_GREY)); }
  if (pl_focused) { wattron(window_, A_REVERSE); }
  mvwprintw(window_, 11, 5, "%-14s", pending_.persist_logs ? "yes" : "no");
  if (pl_focused) { wattroff(window_, A_REVERSE); }
  if (!pl_focused) { wattron(window_, COLOR_PAIR(CP_GREY)); }
  mvwprintw(window_, 11, 20, "< >");
  if (!workspace_ok || !pl_focused) { wattroff(window_, COLOR_PAIR(CP_GREY)); }

  // --- Log Rotate Size ---
  bool size_enabled = workspace_ok && pending_.persist_logs;
  bool rs_focused = size_enabled && (selected_ == kFieldRotateSize);
  std::string rotate_str = formatSize(pending_.log_rotate_size);

  if (!size_enabled) { wattron(window_, COLOR_PAIR(CP_GREY)); }
  mvwprintw(window_, 13, 3, "Log Rotate Size");
  if (!size_enabled) { wattroff(window_, COLOR_PAIR(CP_GREY)); }

  if (rs_focused) { wattron(window_, A_REVERSE); }
  else if (!size_enabled) { wattron(window_, COLOR_PAIR(CP_GREY)); }
  mvwprintw(window_, 14, 5, "%-14s", rotate_str.c_str());
  if (rs_focused) { wattroff(window_, A_REVERSE); }
  if (!rs_focused) { wattron(window_, COLOR_PAIR(CP_GREY)); }
  mvwprintw(window_, 14, 20, "< >");
  if (!rs_focused) { wattroff(window_, COLOR_PAIR(CP_GREY)); }

  // --- Max Total Log Size ---
  bool ms_focused = size_enabled && (selected_ == kFieldMaxSize);
  std::string max_str = formatSize(pending_.log_max_size);

  if (!size_enabled) { wattron(window_, COLOR_PAIR(CP_GREY)); }
  mvwprintw(window_, 16, 3, "Max Total Log Size");
  if (!size_enabled) { wattroff(window_, COLOR_PAIR(CP_GREY)); }

  if (ms_focused) { wattron(window_, A_REVERSE); }
  else if (!size_enabled) { wattron(window_, COLOR_PAIR(CP_GREY)); }
  mvwprintw(window_, 17, 5, "%-14s", max_str.c_str());
  if (ms_focused) { wattroff(window_, A_REVERSE); }
  if (!ms_focused) { wattron(window_, COLOR_PAIR(CP_GREY)); }
  mvwprintw(window_, 17, 20, "< >");
  if (!ms_focused) { wattroff(window_, COLOR_PAIR(CP_GREY)); }

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
    int next = selected_ - 1;
    while (next > 0 && !isFieldEnabled(next)) next--;
    if (next >= 0) selected_ = next;
  } else if (key == KEY_DOWN) {
    int next = selected_ + 1;
    while (next < kNumFields && !isFieldEnabled(next)) next++;
    if (next < kNumFields) selected_ = next;
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

}  // namespace log_view
