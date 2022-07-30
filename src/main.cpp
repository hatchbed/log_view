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

#include <chrono>
#include <csignal>
#include <thread>

#include <log_view/log_store.h>
#include <log_view/log_view.h>
#include <rcl_interfaces/msg/log.hpp>
#include <rclcpp/rclcpp.hpp>

using namespace std::chrono_literals;

void handleSigint(int sig);

class LogViewer : public rclcpp::Node {
  public:
  static bool exit;

  LogViewer() :
    rclcpp::Node("log_viewer"),
    logs_(std::make_shared<log_view::LogStore>()),
    view_(logs_)
  {

  }

  void run() {
    rclcpp::Clock system_clock;
    view_.init();
    sub_ = create_subscription<rcl_interfaces::msg::Log>("/rosout", 10000, std::bind(&LogViewer::handleMsg, this, std::placeholders::_1));

    std::thread ros_thread([&](){ rclcpp::spin(get_node_base_interface()); });

    while (!exit && !view_.exited()) {
      view_.setSystemTime(system_clock.now());
      view_.setRosTime(now());
      view_.update();
      std::this_thread::sleep_for(30ms);
    }
    view_.close();
    ros_thread.join();
  }

  void handleMsg(const rcl_interfaces::msg::Log::SharedPtr msg) {
    logs_->addEntry(msg);
  }

  private:
    rclcpp::Subscription<rcl_interfaces::msg::Log>::SharedPtr sub_;

    log_view::LogStorePtr logs_;
    log_view::LogView view_;
};
bool LogViewer::exit = false;

void handleSigint(int sig)
{
  LogViewer::exit = true;
  rclcpp::shutdown();
}

int main(int argc, char ** argv)
{
  // prevent ncurses from pausing for 1 second on ESC key
  char escape_var[] = "ESCDELAY=0";
  putenv(escape_var);

  rclcpp::init(argc, argv);
  signal(SIGINT, handleSigint);

  LogViewer log_viewer;
  log_viewer.run();

  exit(0);
}
