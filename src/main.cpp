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

#include <csignal>

#include <atomic>
#include <chrono>
#include <string>
#include <thread>
#include <vector>

#include <log_view/bag_loader.h>
#include <log_view/log_store.h>
#include <log_view/log_view.h>
#include <rcl_interfaces/msg/log.hpp>
#include <rclcpp/rclcpp.hpp>
#include <rosgraph_msgs/msg/clock.hpp>

using namespace std::chrono_literals;

void handleSigint(int sig);

class LogViewer : public rclcpp::Node {
  public:
  static std::atomic<bool> exit;

  LogViewer() :
    rclcpp::Node("log_viewer"),
    logs_(std::make_shared<log_view::LogStore>()),
    view_(logs_)
  {}

  void setBagMode(const std::vector<std::string>& bags) {
    bag_files_ = bags;
    view_.setOfflineMode(true);
    view_.setBagFiles(bags);
  }

  void run() {
    rclcpp::Clock system_clock;

    if (!bag_files_.empty()) {
      log_view::loadBagFiles(bag_files_, logs_);
    }

    view_.init();

    if (bag_files_.empty()) {
      sub_ = create_subscription<rcl_interfaces::msg::Log>(
        "/rosout", 10000, std::bind(&LogViewer::handleMsg, this, std::placeholders::_1));

      clock_sub_ = create_subscription<rosgraph_msgs::msg::Clock>(
        "/clock", rclcpp::SensorDataQoS(),
        [this](const rosgraph_msgs::msg::Clock::SharedPtr msg) {
          sim_time_ns_ =
            rclcpp::Time(msg->clock.sec, msg->clock.nanosec, RCL_ROS_TIME).nanoseconds();
          has_sim_time_ = true;
        });
    }

    std::thread ros_thread([&](){ rclcpp::spin(get_node_base_interface()); });

    while (!exit && !view_.exited()) {
      view_.setSystemTime(system_clock.now());
      if (has_sim_time_) {
        view_.setSimTime(rclcpp::Time(sim_time_ns_.load(), RCL_ROS_TIME));
      }
      view_.update();
      std::this_thread::sleep_for(30ms);
    }
    view_.close();

    if (rclcpp::ok()) {
      rclcpp::shutdown();
    }

    ros_thread.join();
  }

  void handleMsg(const rcl_interfaces::msg::Log::SharedPtr msg) {
    logs_->addEntry(msg);
  }

  private:
    rclcpp::Subscription<rcl_interfaces::msg::Log>::SharedPtr sub_;
    rclcpp::Subscription<rosgraph_msgs::msg::Clock>::SharedPtr clock_sub_;
    std::vector<std::string> bag_files_;
    std::atomic<int64_t> sim_time_ns_{0};
    std::atomic<bool> has_sim_time_{false};

    log_view::LogStorePtr logs_;
    log_view::LogView view_;
};
std::atomic<bool> LogViewer::exit{false};

void handleSigint(int sig)
{
  LogViewer::exit = true;
}

int main(int argc, char ** argv)
{
  // prevent ncurses from pausing for 1 second on ESC key
  setenv("ESCDELAY", "0", 1);

  rclcpp::init(argc, argv);
  signal(SIGINT, handleSigint);

  auto non_ros_args = rclcpp::remove_ros_arguments(argc, argv);
  std::vector<std::string> bag_paths(non_ros_args.begin() + 1, non_ros_args.end());

  LogViewer log_viewer;
  if (!bag_paths.empty()) {
    log_viewer.setBagMode(bag_paths);
  }
  log_viewer.run();

  exit(0);
}
