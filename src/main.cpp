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

#include <log_view/log_store.h>
#include <log_view/log_view.h>
#include <rosgraph_msgs/Log.h>

#include <csignal>

#include <ros/ros.h>

void handleSigint(int sig);

class LogViewer {
  public:
  static bool exit;

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
      }
      else if (connected && !master_status) {
        ros::shutdown();
      }
      else if (connected && master_status) {
        ros::spinOnce();
        view_.setRosTime(ros::Time::now());
      }

      connected = master_status;
      view_.update();
    }
    view_.close();
  }

  void handleMsg(const rosgraph_msgs::LogConstPtr& msg) {
    logs_->addEntry(msg);
  }

  private:
    ros::Subscriber sub_;

    log_view::LogStorePtr logs_;
    log_view::LogView view_;
};
bool LogViewer::exit = false;

void handleSigint(int sig)
{
  LogViewer::exit = true;
}

int main(int argc, char **argv)
{
  // prevent ncurses from pausing for 1 second on ESC key
  char escape_var[] = "ESCDELAY=0";
  putenv(escape_var);
  ros::init(argc, argv, "log_viewer", ros::init_options::AnonymousName | ros::init_options::NoRosout);

  LogViewer log_viewer;
  log_viewer.run();

  exit(0);
}