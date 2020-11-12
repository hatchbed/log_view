#ifndef LOG_VIEW_LOG_STORE_H_
#define LOG_VIEW_LOG_STORE_H_

#include <deque>

#include <log_view/datatypes.h>
#include <rosgraph_msgs/Log.h>

namespace log_view {

class LogStore {
public:
  LogStore() = default;

  const std::deque<LogEntry>& logs() const { return logs_; }

  void addEntry(const rosgraph_msgs::LogConstPtr& msg);

private:
  std::deque<LogEntry> logs_;
};

}  // namespace log_view

#endif  // LOG_VIEW_LOG_STORE_H_