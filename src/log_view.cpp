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

#include <log_view/log_view.h>

#include <clocale>
#include <cctype>
#include <clocale>
#include <string>

#include <log_view/utils.h>

namespace log_view {

LogView::LogView(LogStorePtr& logs) :
  logs_(logs),
  log_filter_(logs_)
{

}

LogView::~LogView() {
  close();
}

void LogView::init() {
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
  noecho();
  curs_set(0);
  raw();
  keypad(stdscr, true);
  mouseinterval(0);
  mousemask(ALL_MOUSE_EVENTS | REPORT_MOUSE_POSITION, NULL);
  printf("\033[?1003h\n");

  refresh();

  log_panel_ = std::make_shared<LogPanel>(LINES - 2, COLS, 1, 0, logs_, log_filter_);
  panels_.push_back(log_panel_);

  status_panel_ = std::make_shared<StatusPanel>(1, COLS, 0, 0, logs_);
  panels_.push_back(status_panel_);

  level_panel_ = std::make_shared<LevelPanel>(1, COLS, LINES - 1, 0, log_filter_);
  panels_.push_back(level_panel_);

  search_panel_ = std::make_shared<SearchPanel>(1, COLS, LINES - 1, 0, log_filter_);
  search_panel_->hide(true);
  panels_.push_back(search_panel_);

  filter_panel_ = std::make_shared<FilterPanel>(1, COLS, LINES - 1, 0, log_filter_);
  filter_panel_->hide(true);
  panels_.push_back(filter_panel_);

  exclude_panel_ = std::make_shared<ExcludePanel>(1, COLS, LINES - 1, 0, log_filter_);
  exclude_panel_->hide(true);
  panels_.push_back(exclude_panel_);

  node_panel_ = std::make_shared<NodePanel>(LINES - 2, COLS / 2, 1, COLS / 2 - (COLS + 1) % 2, log_filter_);
  node_panel_->hide(true);
  panels_.push_back(node_panel_);

  help_panel_ = std::make_shared<HelpPanel>(21, COLS - 8, 2, 4);
  help_panel_->hide(true);
  panels_.push_back(help_panel_);

  refreshLayout();

  update_panels();
  doupdate();

  refreshLayout();
}

void LogView::close() {
  endwin();
}

bool LogView::exited() const {
  return exited_;
}

void LogView::setConnected(bool connected) {
  status_panel_->setConnected(connected);
}

void LogView::setRosTime(const ros::Time& time) {
  status_panel_->setRosTime(time);
}

void LogView::setSystemTime(const ros::WallTime& time) {
  status_panel_->setSystemTime(time);
}

void LogView::update() {
  log_filter_.idleProcess();
  log_filter_.update();
  timeout(50);
  int ch = getch();

  bool key_used = false;
  if (ch == KEY_MOUSE) {
    MEVENT event;
    if (getmouse(&event) == OK) {
      if (event.bstate & BUTTON4_PRESSED) {
        ch = KEY_UP;
        key_used = false;
      }
      else {
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
        }
        else {
          for (auto& panel: panels_) {
            panel->handleMouse(event);
          }
        }
      }
    }
  }

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
        level_panel_->refresh();
        break;
      }
    }
  }

  if (!key_used && !mouse_down_) {
    if (ch == KEY_RESIZE)
    {
      refreshLayout();
    }
    else if (/*ch == KEY_ESC || */ch == ctrl('q') || ch == ctrl('c')) {
      exited_ = true;
    }
    else if (ch == '\t') {
      tab();
    }
    else if (ch == ctrl('s')) {
      search_panel_->toggle();
      if (search_panel_->focus()) {
        unfocusOthers(search_panel_);
      }
      else {
        focusNext(search_panel_);
      }
      refreshLayout();
    }
    else if (ch == ctrl('x')) {
      search_panel_->clearSearch();
      refreshLayout();
    }
    else if (ch == KEY_BACKSPACE) {
      log_filter_.prevMatch();
      log_panel_->forceRefresh();
    }
    else if (ch == KEY_ENTER_VAL) {
      log_filter_.nextMatch();
      log_panel_->forceRefresh();
    }
    else if (ch == ctrl('e')) {
      exclude_panel_->hide(exclude_panel_->visible());
      if (exclude_panel_->focus()) {
        unfocusOthers(exclude_panel_);
      }
      else {
        focusNext(exclude_panel_);
      }
      refreshLayout();
    }
    else if (ch == ctrl('f')) {
      filter_panel_->hide(filter_panel_->visible());
      if (filter_panel_->focus()) {
        unfocusOthers(filter_panel_);
      }
      else {
        focusNext(exclude_panel_);
      }
      refreshLayout();
    }
    else if (ch == ctrl('h')) {
      help_panel_->hide(help_panel_->visible());
    }
    else if (ch == ctrl('n')) {
      node_panel_->hide(node_panel_->visible());
      if (node_panel_->focus()) {
        unfocusOthers(node_panel_);
      }
      else {
        focusNext(node_panel_);
      }
    }
    else if (ch == KEY_F(1)) {
      level_panel_->toggleDebug();
    }
    else if (ch == KEY_F(2)) {
      level_panel_->toggleInfo();
    }
    else if (ch == KEY_F(3)) {
      level_panel_->toggleWarn();
    }
    else if (ch == KEY_F(4)) {
      level_panel_->toggleError();
    }
    else if (ch == KEY_F(5)) {
      level_panel_->toggleFatal();
    }
    else if (ch == KEY_F(7)) {
      level_panel_->toggleAllNodes();
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

  status_panel_->refresh();

  if (help_panel_->visible()) {
    help_panel_->toTop();
  }

  curs_set(0);
  for (auto& panel: panels_) {
    panel->setCursor();
  }

  update_panels();
  doupdate();
}

void LogView::refreshLayout() {
  status_panel_->resize(1, COLS, 0, 0);
  log_panel_->resize(LINES - (2 + filter_panel_->visible() + exclude_panel_->visible() + search_panel_->visible()), COLS, 1, 0);
  level_panel_->resize(1, COLS, LINES - (1 + filter_panel_->visible() + exclude_panel_->visible() + search_panel_->visible()), 0);
  search_panel_->resize(1, COLS, LINES - (1 + exclude_panel_->visible() + filter_panel_->visible()), 0);
  filter_panel_->resize(1, COLS, LINES - (1 + exclude_panel_->visible()), 0);
  exclude_panel_->resize(1, COLS, LINES - 1, 0);
  node_panel_->resize(LINES - (2 + filter_panel_->visible() + exclude_panel_->visible() + search_panel_->visible()), COLS / 2, 1, COLS / 2 - (COLS + 1) % 2 + !log_panel_->scrollbar());
  help_panel_->resize(21, COLS - 8, 2, 4);
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
  for (auto& panel: panels_) {
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

}  // namespace log_view