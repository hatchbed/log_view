#include <log_view/log_store.h>
#include <rosgraph_msgs/Log.h>

#include <chrono>
#include <thread>

#include <ros/ros.h>

class LogViewer {
  public:
  LogViewer() = default;

  ~LogViewer() {
    stop();
  }

  void init() {
    thread_ = std::make_shared<std::thread>(&LogViewer::run, this);
  }

  void stop() {
    exit_ = true;
    thread_->join();
  }

  void run() {
    printf("run...\n");
    bool connected = false;
    while (!exit_) {
      bool master_status = ros::master::check();
      if (!connected && master_status) {
        printf("connected to ROS master\n");
        ros::start();
        ros::NodeHandle node;
        sub_ = node.subscribe("/rosout_agg", 10000, &LogViewer::handleMsg, this);
      }
      else if (connected && !master_status) {
        ros::shutdown();
        printf("disconnected from ROS master\n");
      }
      else if (connected && master_status) {
        ros::spinOnce();
      }

      connected = master_status;
      std::this_thread::sleep_for(std::chrono::milliseconds(50));
    }
  }

  void handleMsg(const rosgraph_msgs::LogConstPtr& msg) {

  }

  private:
    bool exit_ = false;
    ros::Subscriber sub_;

    std::shared_ptr<std::thread> thread_;
};

int main(int argc, char **argv)
{
  ros::init(argc, argv, "log_viewer", ros::init_options::AnonymousName | ros::init_options::NoRosout);
  LogViewer log_viewer;
  log_viewer.init();
  while(true) {
    std::this_thread::sleep_for(std::chrono::milliseconds(5000));
  }

  return 0;
}