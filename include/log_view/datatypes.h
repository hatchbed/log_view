#ifndef LOG_VIEW_DATATYPES_H_
#define LOG_VIEW_DATATYPES_H_

#include <ros/ros.h>
#include <rosgraph_msgs/Log.h>

namespace log_view {

struct LogEntry
{
  LogEntry(const rosgraph_msgs::Log& log) :
    stamp(log.header.stamp),
    seq(log.header.seq),
    level(log.level),
    node(log.name),
    file(log.file),
    function(log.function),
    line(log.line),
    text(log.msg)
  {}

  ros::Time stamp;
  uint32_t seq;
  uint8_t level;
  std::string node;
  std::string file;
  std::string function;
  uint32_t line;
  std::string text;
};

}  // namespace log_view

#endif  // LOG_VIEW_DATATYPES_H_