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

#include <log_view/log_view.h>

#include <cctype>
#include <clocale>
#include <string>

#include <log_view/utils.h>

namespace log_view {

LogView::LogView(LogStorePtr& logs) :
  logs_(logs),
  log_filter_(logs_)
{}

void LogView::setOfflineMode(bool offline) {
  offline_mode_ = offline;
}

void LogView::setBagFiles(const std::vector<std::string>& bags) {
  bag_files_ = bags;
  log_filter_.setBagSources(bags);
}

LogView::~LogView() {
  close();
}

void LogView::init() {
  prefs_.load();

  if (!offline_mode_) {
    if (prefs_.persist_logs) {
      log_writer_ = std::make_unique<LogWriter>(
        prefs_.workspace_dir, prefs_.log_rotate_size, prefs_.log_max_size);
      for (const auto& entry : log_writer_->loadAll()) {
        logs_->addEntry(entry);
      }
      logs_->setWriter(log_writer_.get());
      log_writer_->start();
    }

    auto marker = makeMarkerEntry("Session Started At");
    logs_->addEntry(marker);
    if (log_writer_) {
      log_writer_->enqueue(marker);
    }
  }

  setlocale(LC_ALL, "");
  initscr();
  use_default_colors();
  start_color();
  init_pair(CP_DEFAULT, -1, -1);
  init_pair(CP_RED, COLOR_RED, -1);
  init_pair(CP_YELLOW, COLOR_YELLOW, -1);
  init_pair(CP_GREY, 8, -1);
  init_pair(CP_DEFAULT_CYAN, -1, COLOR_CYAN);
  init_pair(CP_DEFAULT_GREY, -1, 8);
  init_pair(CP_ANSI_BLACK,   COLOR_BLACK,   -1);
  init_pair(CP_ANSI_RED,     COLOR_RED,     -1);
  init_pair(CP_ANSI_GREEN,   COLOR_GREEN,   -1);
  init_pair(CP_ANSI_YELLOW,  COLOR_YELLOW,  -1);
  init_pair(CP_ANSI_BLUE,    COLOR_BLUE,    -1);
  init_pair(CP_ANSI_MAGENTA, COLOR_MAGENTA, -1);
  init_pair(CP_ANSI_CYAN,    COLOR_CYAN,    -1);
  init_pair(CP_ANSI_WHITE,   COLOR_WHITE,   -1);
  init_pair(CP_BRIGHT_BLUE,  (COLORS >= 16) ? 12 : COLOR_BLUE, -1);
  init_pair(CP_WHITE_CYAN,   COLOR_WHITE, COLOR_CYAN);
  kAttrGrey     = (COLORS >= 16) ? COLOR_PAIR(CP_GREY)         : static_cast<attr_t>(A_DIM);
  kAttrGreyBg   = (COLORS >= 16) ? COLOR_PAIR(CP_DEFAULT_GREY) : static_cast<attr_t>(A_REVERSE);
  kAttrBoldBlue = A_BOLD | COLOR_PAIR(CP_ANSI_BLUE);
  noecho();
  curs_set(0);
  raw();
  keypad(stdscr, true);
  mouseinterval(0);
  mousemask(ALL_MOUSE_EVENTS | REPORT_MOUSE_POSITION, NULL);
  printf("\033[?1003h\n");  // Enable mouse move events
  define_key("\033[1;5A", KEY_SR);
  define_key("\033[1;5B", KEY_SF);
  define_key("\033[5;5~", KEY_SPREVIOUS);
  define_key("\033[6;5~", KEY_SNEXT);
  define_key("\033[1;5H", KEY_SHOME);
  define_key("\033[1;5F", KEY_SEND);

  refresh();

  log_panel_ = std::make_shared<LogPanel>(LINES - 2, COLS, 1, 0, logs_, log_filter_, prefs_);
  panels_.push_back(log_panel_);

  status_panel_ = std::make_shared<StatusPanel>(1, COLS, 0, 0, logs_, log_filter_);
  if (!bag_files_.empty()) {
    status_panel_->setBagFiles(bag_files_);
  }
  panels_.push_back(status_panel_);

  level_panel_ = std::make_shared<LevelPanel>(1, COLS, LINES - 1, 0, log_filter_);
  panels_.push_back(level_panel_);

  search_panel_ = std::make_shared<SearchPanel>(1, COLS, LINES - 1, 0, log_filter_);
  search_panel_->hide(true);
  search_panel_->setOnSearch([this]() {
    log_panel_->forceRefresh();
  });
  panels_.push_back(search_panel_);

  filter_panel_ = std::make_shared<FilterPanel>(1, COLS, LINES - 1, 0, log_filter_);
  filter_panel_->hide(true);
  panels_.push_back(filter_panel_);

  exclude_panel_ = std::make_shared<ExcludePanel>(1, COLS, LINES - 1, 0, log_filter_);
  exclude_panel_->hide(true);
  panels_.push_back(exclude_panel_);

  node_panel_ = std::make_shared<NodePanel>(
    LINES - 2, COLS / 2, 1, COLS / 2 - (COLS + 1) % 2, log_filter_);
  node_panel_->hide(true);
  panels_.push_back(node_panel_);
  level_panel_->setShowInvertHintCallback([this]() {
    return node_panel_->visible() && node_panel_->focus() &&
           !help_panel_->visible() && !prefs_panel_->visible();
  });

  if (!bag_files_.empty()) {
    bag_source_panel_ = std::make_shared<BagSourcePanel>(
      LINES - 2, COLS / 2, 1, COLS / 2 - (COLS + 1) % 2, log_filter_);
    bag_source_panel_->hide(true);
    panels_.push_back(bag_source_panel_);
    level_panel_->setBagMode(true);
    level_panel_->setBagPanelOpenCallback([this]() {
      return bag_source_panel_->visible();
    });
    level_panel_->setShowBagInvertHintCallback([this]() {
      return bag_source_panel_->visible() && bag_source_panel_->focus() &&
             !help_panel_->visible() && !prefs_panel_->visible();
    });
  }

  details_panel_ = std::make_shared<DetailsPanel>(
    LINES - 2, COLS / 2, 1, COLS / 2 - (COLS + 1) % 2, log_filter_);
  details_panel_->hide(true);
  panels_.push_back(details_panel_);

  help_panel_ = std::make_shared<HelpPanel>(21, COLS - 8, 2, 4);
  help_panel_->hide(true);
  panels_.push_back(help_panel_);
  level_panel_->setHelpOpenCallback([this]() {
    return help_panel_->visible();
  });

  {
    int pw = prefsPanelWidth();
    prefs_panel_ = std::make_shared<PrefsPanel>(
      25, pw, std::max(0, LINES / 2 - 12), std::max(0, COLS / 2 - pw / 2), prefs_);
  }
  prefs_panel_->hide(true);
  prefs_panel_->setOnSave([this]() {
    if (prefs_.persist_logs && !log_writer_) {
      log_writer_ = std::make_unique<LogWriter>(
        prefs_.workspace_dir, prefs_.log_rotate_size, prefs_.log_max_size);
      logs_->setWriter(log_writer_.get());
      log_writer_->start();
    } else if (!prefs_.persist_logs && log_writer_) {
      log_writer_->stop();
      logs_->setWriter(nullptr);
      log_writer_.reset();
    } else if (log_writer_) {
      log_writer_->stop();
      logs_->setWriter(nullptr);
      log_writer_ = std::make_unique<LogWriter>(
        prefs_.workspace_dir, prefs_.log_rotate_size, prefs_.log_max_size);
      logs_->setWriter(log_writer_.get());
      log_writer_->start();
    }
    log_filter_.setShowSessionBoundaries(prefs_.show_session_boundaries);
    log_panel_->forceRefresh();
  });
  prefs_panel_->setOnPreview([this]() {
    log_panel_->forceRefresh();
  });
  panels_.push_back(prefs_panel_);

  log_filter_.setShowSessionBoundaries(prefs_.show_session_boundaries);

  if (prefs_.persist_filters) {
    log_filter_.setDebugLevel(prefs_.filters.debug);
    log_filter_.setInfoLevel(prefs_.filters.info);
    log_filter_.setWarnLevel(prefs_.filters.warn);
    log_filter_.setErrorLevel(prefs_.filters.error);
    log_filter_.setFatalLevel(prefs_.filters.fatal);
    log_filter_.setEnableNodeFilter(prefs_.filters.node_filter_enabled);
    log_filter_.setNodeWhitelist(prefs_.filters.node_whitelist);
    if (!prefs_.filters.filter_pattern.empty()) {
      filter_panel_->setInputText(prefs_.filters.filter_pattern);
      filter_panel_->hide(false);
      filter_panel_->setFocus(false);
    }
    if (!prefs_.filters.exclude_pattern.empty()) {
      exclude_panel_->setInputText(prefs_.filters.exclude_pattern);
      exclude_panel_->hide(false);
      exclude_panel_->setFocus(false);
    }
  }

  refreshLayout();

  update_panels();
  doupdate();

  refreshLayout();
}

void LogView::close() {
  if (log_writer_) {
    auto marker = makeMarkerEntry("Recording Ended At");
    logs_->addEntry(marker);
    log_writer_->enqueue(marker);
    log_writer_->stop();
    logs_->setWriter(nullptr);
  }

  if (prefs_.persist_filters) {
    prefs_.filters.debug  = log_filter_.getDebugLevel();
    prefs_.filters.info   = log_filter_.getInfoLevel();
    prefs_.filters.warn   = log_filter_.getWarnLevel();
    prefs_.filters.error  = log_filter_.getErrorLevel();
    prefs_.filters.fatal  = log_filter_.getFatalLevel();
    prefs_.filters.node_filter_enabled = log_filter_.getEnableNodeFilter();
    prefs_.filters.filter_pattern  = log_filter_.getFilterString();
    prefs_.filters.exclude_pattern = log_filter_.getExcludeString();
    prefs_.filters.node_whitelist.clear();
    for (const auto& [name, data] : log_filter_.nodes()) {
      if (data.selected) {
        prefs_.filters.node_whitelist.insert(name);
      }
    }
    prefs_.save();
  }
  printf("\033[?1003l\n");  // Disable mouse movement events
  endwin();
}

bool LogView::exited() const {
  return exited_;
}

void LogView::setSimTime(const rclcpp::Time& time) {
  status_panel_->setSimTime(time);
}

void LogView::setSystemTime(const rclcpp::Time& time) {
  status_panel_->setSystemTime(time);
}

void LogView::update() {
  log_filter_.idleProcess();
  log_filter_.update();
  timeout(50);
  int ch = getch();

  bool key_used = false;

  if (confirm_clear_) {
    if (ch != ERR && ch != KEY_MOUSE) {
      if (ch == 'y' || ch == 'Y') {
        log_filter_.clearLogs();
        for (auto& p : panels_) {
          p->forceRefresh();
        }
      }
      closeConfirmClear();
    }
    key_used = true;
  }

  while (!key_used && ch == KEY_MOUSE) {
    MEVENT event;
    if (getmouse(&event) == OK) {
      if (event.bstate & BUTTON4_PRESSED) {
        ch = KEY_UP;
        break;
      } else {
        key_used = true;

        bool pressed = event.bstate & (BUTTON1_PRESSED | BUTTON2_PRESSED | BUTTON3_PRESSED);
        if (pressed) {
          for (size_t i = 0; i < panels_.size(); i++) {
            size_t idx = panels_.size() - (i + 1);
            auto& panel = panels_[idx];
            if (panel->handleMouse(event)) {
              break;
            }
          }
        } else {
          for (auto& panel : panels_) {
            panel->handleMouse(event);
          }
        }
      }
    }
    timeout(0);
    ch = getch();
  }
  timeout(50);

  if (!key_used) {
    std::for_each(panels_.rbegin(), panels_.rend(), [&](PanelInterfacePtr& panel) {
      if (!key_used) {
        key_used = panel->handleKey(ch);
      }
    });
  }

  if (!key_used) {
    std::for_each(panels_.rbegin(), panels_.rend(), [&](PanelInterfacePtr& panel) {
      if (!key_used && panel->focus()) {
        key_used = panel->handleInput(ch);
        if (key_used) {
          if (!panel->focus()) {
            focusNext(panel);
            refreshLayout();
          }
          if (!panel->visible()) {
            refreshLayout();
          }
        }
      }
    });
  }

  if (!key_used && !mouse_down_) {
    for (size_t i = 1; i <= panels_.size(); i++) {
      key_used = panels_[panels_.size() - i]->handleNavigation(ch);
      if (key_used) {
        break;
      }
    }
  }

  if (!key_used && !mouse_down_) {
    if (ch == KEY_RESIZE) {
      refreshLayout();
    } else if (ch == ctrl('q') || ch == ctrl('c')) {
      exited_ = true;
    } else if (ch == '\t') {
      tab();
    } else if (ch == ctrl('s')) {
      search_panel_->toggle();
      if (search_panel_->focus()) {
        unfocusOthers(search_panel_);
      } else {
        focusNext(search_panel_);
      }
      refreshLayout();
    } else if (ch == ctrl('x')) {
      search_panel_->clearSearch();
      refreshLayout();
    } else if (ch == ctrl('e')) {
      exclude_panel_->hide(exclude_panel_->visible());
      if (exclude_panel_->focus()) {
        unfocusOthers(exclude_panel_);
      } else {
        focusNext(exclude_panel_);
      }
      refreshLayout();
    } else if (ch == ctrl('f')) {
      filter_panel_->hide(filter_panel_->visible());
      if (filter_panel_->focus()) {
        unfocusOthers(filter_panel_);
      } else {
        focusNext(filter_panel_);
      }
      refreshLayout();
    } else if (ch == ctrl('h')) {
      help_panel_->hide(help_panel_->visible());
    } else if (ch == ctrl('k')) {
      prefs_panel_->hide(prefs_panel_->visible());
    } else if (ch == ctrl('n')) {
      details_panel_->hide(true);
      if (bag_source_panel_) { bag_source_panel_->hide(true); }
      node_panel_->hide(node_panel_->visible());
      if (node_panel_->focus()) {
        unfocusOthers(node_panel_);
      } else {
        focusNext(node_panel_);
      }
      refreshLayout();
    } else if (ch == ctrl('b') && bag_source_panel_) {
      node_panel_->hide(true);
      details_panel_->hide(true);
      bag_source_panel_->hide(bag_source_panel_->visible());
      if (bag_source_panel_->focus()) {
        unfocusOthers(bag_source_panel_);
      } else {
        focusNext(bag_source_panel_);
      }
      refreshLayout();
    } else if (ch == ctrl('d')) {
      node_panel_->hide(true);
      if (bag_source_panel_) { bag_source_panel_->hide(true); }
      details_panel_->hide(details_panel_->visible());
      if (details_panel_->focus()) {
        unfocusOthers(details_panel_);
      } else {
        focusNext(details_panel_);
      }
      refreshLayout();
    } else if (ch == KEY_F(1)) {
      level_panel_->toggleDebug();
    } else if (ch == KEY_F(2)) {
      level_panel_->toggleInfo();
    } else if (ch == KEY_F(3)) {
      level_panel_->toggleWarn();
    } else if (ch == KEY_F(4)) {
      level_panel_->toggleError();
    } else if (ch == KEY_F(5)) {
      level_panel_->toggleFatal();
    } else if (ch == KEY_F(7)) {
      level_panel_->toggleAllNodes();
    } else if (ch == ctrl('r')) {
      openConfirmClear();
    }
  }

  log_panel_->refresh();
  if (log_scroll_ != log_panel_->scrollbar()) {
    refreshLayout();
    log_scroll_ = log_panel_->scrollbar();
  }

  if (node_panel_->visible()) {
    node_panel_->refresh();
  }

  if (bag_source_panel_ && bag_source_panel_->visible()) {
    bag_source_panel_->refresh();
  }

  if (details_panel_->visible()) {
    details_panel_->refresh();
  }

  level_panel_->refresh();
  status_panel_->refresh();

  if (help_panel_->visible()) {
    help_panel_->toTop();
  }

  if (prefs_panel_->visible()) {
    prefs_panel_->toTop();
  }

  if (confirm_clear_) {
    top_panel(confirm_panel_);
  }

  curs_set(0);
  for (auto& panel : panels_) {
    panel->setCursor();
  }

  update_panels();
  doupdate();
}

void LogView::refreshLayout() {
  status_panel_->resize(1, COLS, 0, 0);
  int side_start = COLS / 2 - (COLS + 1) % 2 + !log_panel_->scrollbar();
  bool side_visible = node_panel_->visible() || details_panel_->visible() ||
                      (bag_source_panel_ && bag_source_panel_->visible());
  log_panel_->setRightEdge(side_visible ? side_start : 0);
  log_panel_->resize(
    LINES - (2 + filter_panel_->visible() + exclude_panel_->visible() + search_panel_->visible()),
    COLS, 1, 0);
  level_panel_->resize(
    1, COLS,
    LINES - (1 + filter_panel_->visible() + exclude_panel_->visible() + search_panel_->visible()),
    0);
  search_panel_->resize(
    1, COLS, LINES - (1 + exclude_panel_->visible() + filter_panel_->visible()), 0);
  filter_panel_->resize(1, COLS, LINES - (1 + exclude_panel_->visible()), 0);
  exclude_panel_->resize(1, COLS, LINES - 1, 0);
  node_panel_->resize(
    LINES - (2 + filter_panel_->visible() + exclude_panel_->visible() + search_panel_->visible()),
    COLS / 2, 1, side_start);
  if (bag_source_panel_) {
    bag_source_panel_->resize(
      LINES -
        (2 + filter_panel_->visible() + exclude_panel_->visible() + search_panel_->visible()),
      COLS / 2, 1, side_start);
  }
  details_panel_->resize(
    LINES - (2 + filter_panel_->visible() + exclude_panel_->visible() + search_panel_->visible()),
    COLS / 2, 1, side_start);
  int help_height = std::min(21, std::max(5, LINES - 4));
  help_panel_->resize(help_height, COLS - 8, 2, 4);
  int pw = prefsPanelWidth();
  int prefs_height = std::min(25, std::max(6, LINES - 4));
  int prefs_y = std::max(0, LINES / 2 - prefs_height / 2);
  prefs_panel_->resize(prefs_height, pw, prefs_y, std::max(0, COLS / 2 - pw / 2));
}

int LogView::prefsPanelWidth() const {
  // Minimum width to fit all fixed content (hint line at col 3 is 33 chars -> needs 38).
  int w = 38;
  if (!prefs_.workspace_dir.empty()) {
    int s1 = static_cast<int>((prefs_.workspace_dir + "/preferences.yaml").size()) + 6;
    int s2 = static_cast<int>((prefs_.workspace_dir + "/").size()) + 6;
    w = std::max(w, std::max(s1, s2));
  } else if (!prefs_.workspace_error.empty()) {
    w = std::max(w, static_cast<int>(prefs_.workspace_error.size()) + 6);
  }
  return std::min(w, COLS - 4);
}

void LogView::tab() {
  int idx = -1;
  for (size_t i = 0; i < panels_.size(); i++) {
    if (panels_[i]->focus()) {
      idx = i;
      break;
    }
  }
  if (idx == -1) {
    return;
  }

  panels_[idx]->setFocus(false);
  for (size_t i = 1; i < panels_.size() + 1; i++) {
    int next = (idx + i) % panels_.size();
    if (panels_[next]->setFocus(true)) {
      break;
    }
  }
}

void LogView::unfocusOthers(const PanelInterfacePtr& focused) {
  for (auto& panel : panels_) {
    if (panel != focused) {
      panel->setFocus(false);
    }
  }
}

void LogView::focusNext(const PanelInterfacePtr& panel) {
  int idx = -1;
  for (size_t i = 0; i < panels_.size(); i++) {
    if (panels_[i] == panel) {
      idx = i;
      break;
    }
  }

  if (idx < 0) {
    return;
  }

  for (size_t i = 1; i < panels_.size(); i++) {
    int next = (idx + i) % panels_.size();
    if (panels_[next]->setFocus(true)) {
      break;
    }
  }
}

void LogView::openConfirmClear() {
  static const std::string msg = "Clear all messages? (y/N)";
  int width = static_cast<int>(msg.length()) + 4;
  int height = 3;
  int y = LINES / 2 - 1;
  int x = COLS / 2 - width / 2;

  confirm_win_ = newwin(height, width, y, x);
  confirm_panel_ = new_panel(confirm_win_);
  box(confirm_win_, 0, 0);
  mvwprintw(confirm_win_, 1, 2, "%s", msg.c_str());
  confirm_clear_ = true;
}

void LogView::closeConfirmClear() {
  del_panel(confirm_panel_);
  delwin(confirm_win_);
  confirm_panel_ = nullptr;
  confirm_win_ = nullptr;
  confirm_clear_ = false;
  refreshLayout();
}

}  // namespace log_view
