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

#include <atomic>

#include <log_view/log_store.h>
#include <log_view/log_view.h>
#include <rosgraph_msgs/Log.h>
#include <rosgraph_msgs/Clock.h>

#include <csignal>

#include <ros/ros.h>

void handleSigint(int sig);

class LogViewer {
  public:
  static std::atomic<bool> exit;

  LogViewer() :
    logs_(std::make_shared<log_view::LogStore>()),
    view_(logs_)
  {

  }

  void run() {
    bool connected = false;
    view_.init();
    while (!exit && !view_.exited()) {
      bool master_status = ros::master::check();
      view_.setConnected(master_status);
      view_.setSystemTime(ros::WallTime::now());
      if (!connected && master_status) {
        ros::start();
        signal(SIGINT, handleSigint);
        ros::NodeHandle node;
        sub_ = node.subscribe("/rosout_agg", 10000, &LogViewer::handleMsg, this);
        clock_sub_ = node.subscribe<rosgraph_msgs::Clock>("/clock", 1,
          [this](const rosgraph_msgs::Clock::ConstPtr& msg) {
            sim_time_ns_ = msg->clock.toNSec();
            has_sim_time_ = true;
          });
      }
      else if (connected && !master_status) {
        ros::shutdown();
      }
      else if (connected && master_status) {
        ros::spinOnce();
        if (has_sim_time_) {
          view_.setSimTime(ros::Time().fromNSec(sim_time_ns_.load()));
        }
      }

      connected = master_status;
      view_.update();
    }
    view_.close();

    if (connected && ros::ok()) {
      ros::shutdown();
    }
  }

  void handleMsg(const rosgraph_msgs::LogConstPtr& msg) {
    logs_->addEntry(msg);
  }

  private:
    ros::Subscriber sub_;
    ros::Subscriber clock_sub_;
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

int main(int argc, char **argv)
{
  // prevent ncurses from pausing for 1 second on ESC key
  setenv("ESCDELAY", "0", 1);
  ros::init(argc, argv, "log_viewer", ros::init_options::AnonymousName | ros::init_options::NoRosout);

  LogViewer log_viewer;
  log_viewer.run();

  exit(0);
}