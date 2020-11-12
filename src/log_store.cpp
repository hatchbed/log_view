#include <log_view/log_store.h>

namespace log_view {

void LogStore::addEntry(const rosgraph_msgs::LogConstPtr& msg) {
  logs_.emplace_back(*msg);
}

}  // namespace log_view