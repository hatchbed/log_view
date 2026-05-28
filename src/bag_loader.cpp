// Copyright 2026 Hatchbed L.L.C.
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

#include <log_view/bag_loader.h>

#include <algorithm>
#include <cstdio>
#include <string>

#include <rcl_interfaces/msg/log.hpp>
#include <rclcpp/serialization.hpp>
#include <rclcpp/serialized_message.hpp>
#include <rosbag2_cpp/reader.hpp>
#include <rosbag2_storage/storage_filter.hpp>
#include <rosbag2_storage/storage_options.hpp>

#include <log_view/datatypes.h>

namespace log_view {

void loadBagFiles(const std::vector<std::string>& paths, LogStorePtr& logs) {
  std::vector<LogEntry> entries;

  for (size_t bag_idx = 0; bag_idx < paths.size(); bag_idx++) {
    const auto& path = paths[bag_idx];
    try {
      rosbag2_cpp::Reader reader;
      rosbag2_storage::StorageOptions opts;
      opts.uri = path;
      reader.open(opts);

      std::vector<std::string> log_topics;
      for (const auto& t : reader.get_all_topics_and_types()) {
        if (t.type == "rcl_interfaces/msg/Log") {
          log_topics.push_back(t.name);
        }
      }
      if (log_topics.empty()) {
        fprintf(stderr, "[log_viewer] bag '%s' contains no rcl_interfaces/msg/Log topics\n",
                path.c_str());
        continue;
      }

      rosbag2_storage::StorageFilter filter;
      filter.topics = log_topics;
      reader.set_filter(filter);

      rclcpp::Serialization<rcl_interfaces::msg::Log> serializer;
      while (reader.has_next()) {
        auto bag_msg = reader.read_next();
        rcl_interfaces::msg::Log log_msg;
        rclcpp::SerializedMessage extracted(*bag_msg->serialized_data);
        serializer.deserialize_message(&extracted, &log_msg);
        LogEntry entry(log_msg);
        entry.bag_source_idx = static_cast<int>(bag_idx);
        entries.emplace_back(entry);
      }
    } catch (const std::exception& e) {
      fprintf(stderr, "[log_viewer] failed to read bag '%s': %s\n", path.c_str(), e.what());
    }
  }

  std::sort(entries.begin(), entries.end(),
    [](const LogEntry& a, const LogEntry& b) { return a.stamp < b.stamp; });

  if (!entries.empty()) {
    std::string label = "Loaded from bag(s): ";
    for (size_t i = 0; i < paths.size(); i++) {
      if (i > 0) { label += ", "; }
      label += paths[i];
    }

    LogEntry marker;
    marker.stamp    = entries.front().stamp - rclcpp::Duration(0, 1);
    marker.level    = rcl_interfaces::msg::Log::INFO;
    marker.node     = kMarkerNode;
    marker.text     = {"-------- " + label + " --------"};
    logs->addEntry(marker);
  }

  for (const auto& e : entries) {
    logs->addEntry(e);
  }
}

}  // namespace log_view
