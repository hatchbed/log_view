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

#ifndef LOG_VIEW_LOG_FILTER_H_
#define LOG_VIEW_LOG_FILTER_H_

#include <array>
#include <cstdint>
#include <deque>
#include <map>

#include <log_view/datatypes.h>
#include <log_view/log_store.h>

namespace log_view {

class LogFilter {
public:
  explicit LogFilter(LogStorePtr& logs);

  void setCursorOffset(int64_t offset) { cursor_offset_ = offset; }

  void setFilter(const std::string& filter);
  std::string getFilterString() const { return filter_string_; }
  void setExclude(const std::string& exclude);
  void setDebugLevel(bool enable);
  void setInfoLevel(bool enable);
  void setWarnLevel(bool enable);
  void setErrorLevel(bool enable);
  void setFatalLevel(bool enable);
  void setEnableNodeFilter(bool enable);
  void toggleNode(const std::string& node);
  void selectAllNodes();
  void invertNodes();

  bool getDebugLevel() const { return debug_level_; }
  bool getInfoLevel() const { return info_level_; }
  bool getWarnLevel() const { return warn_level_; }
  bool getErrorLevel() const { return error_level_; }
  bool getFatalLevel() const { return fatal_level_; }
  bool getEnableNodeFilter() const { return filter_nodes_; }

  void reset();
  void update();
  void idleProcess();

  void setCursor(int64_t index);
  size_t getCursor();

  void clearSelect();

  void setSelectStart(int64_t index);
  int64_t getSelectStart();

  void setSelectEnd(int64_t index);
  int64_t getSelectEnd();

  void search(const std::string& pattern);
  void nextMatch();
  void prevMatch();
  void clearSearch();
  std::string getSearch() const { return search_; }

  const std::deque<LogLine>& indices() const { return log_indices_; }
  const std::map<std::string, NodeData>& nodes() const { return nodes_; }

  int64_t search_cursor_ = -1;
  int64_t search_cursor_fwd_ = -1;
  int64_t search_cursor_rev_ = -1;

private:
  bool accepted(const LogEntry& entry, bool new_entry = false);

  LogStorePtr logs_;

  std::deque<LogLine> log_indices_;
  size_t latest_log_index_ = 0;
  size_t earliest_log_index_ = 0;

  int64_t cursor_ = -1;
  int64_t select_start_ = -1;
  int64_t select_end_ = -1;

  int64_t cursor_offset_ = 0;

  enum SearchDirection { SEARCH_BOTH, SEARCH_FWD, SEARCH_REV };
  std::string search_;
  int search_direction_ = SEARCH_BOTH;

  bool debug_level_ = true;
  bool info_level_ = true;
  bool warn_level_ = true;
  bool error_level_ = true;
  bool fatal_level_ = true;
  bool filter_nodes_ = false;

  std::string filter_string_;

  std::vector<std::string> filter_list_;
  std::vector<std::string> exclude_list_;

  std::map<std::string, NodeData> nodes_;
};
typedef std::shared_ptr<LogFilter> LogFilterPtr;

}  // namespace log_view

#endif  // LOG_VIEW_LOG_FILTER_H_