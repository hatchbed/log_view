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

#include <curses.h>
#include <panel.h>

#include <vector>

#include <rclcpp/rclcpp.hpp>

#include <log_view/log_filter.h>
#include <log_view/log_store.h>
#include <log_view/panel_interface.h>
#include <log_view/panels/details_panel.h>
#include <log_view/panels/exclude_panel.h>
#include <log_view/panels/filter_panel.h>
#include <log_view/panels/help_panel.h>
#include <log_view/panels/level_panel.h>
#include <log_view/panels/log_panel.h>
#include <log_view/panels/node_panel.h>
#include <log_view/panels/search_panel.h>
#include <log_view/panels/status_panel.h>

namespace log_view {

class LogView {
public:
  explicit LogView(LogStorePtr& logs);
  ~LogView();

  void init();
  void close();

  bool exited() const;

  void setRosTime(const rclcpp::Time& time);
  void setSystemTime(const rclcpp::Time& time);

  void update();

private:
  void refreshLayout();

  size_t viewSize() const;

  void tab();
  void focusNext(const PanelInterfacePtr& panel);
  void unfocusOthers(const PanelInterfacePtr& focused);

  LogStorePtr logs_;
  LogFilter log_filter_;

  bool exited_ = false;
  bool mouse_down_ = false;

  bool node_select_ = true;
  bool log_scroll_ = false;

  std::vector<PanelInterfacePtr> panels_;
  DetailsPanelPtr details_panel_;
  StatusPanelPtr status_panel_;
  LevelPanelPtr level_panel_;
  SearchPanelPtr search_panel_;
  FilterPanelPtr filter_panel_;
  ExcludePanelPtr exclude_panel_;
  LogPanelPtr log_panel_;
  NodePanelPtr node_panel_;
  HelpPanelPtr help_panel_;
};

}  // namespace log_view
