#ifndef MY_RVIZ_PLUGIN_BAG_READER_H
#define MY_RVIZ_PLUGIN_BAG_READER_H

#include <rosbag/bag.h>
#include <rosbag/view.h>
#include <string>
#include <vector>
#include <thread>
#include <atomic>
#include <functional>
#include <mutex>
#include <condition_variable>
#include <algorithm>
#include <cmath>

namespace my_rviz_plugin
{

class BagReader
{
public:
  using MessageCallback = std::function<void(const std::vector<rosbag::MessageInstance>&,
                                             const int&,
                                             const std::vector<int>&
                                            )>;
  using UpdateProgressBarCallback = std::function<void(float)>;

  BagReader();
  ~BagReader();

  void readBagFile(const std::string& file_path, int& radarFrameCount);
  void jumpToFrame(int frame_number);
  void playBag();
  void stopBag();
  void setPlayRate(double rate);
  void setMessageCallback(MessageCallback callback);
  void setUpdateProgressBarCallback(UpdateProgressBarCallback callback);

  int find_Closest_Frame(const ros::Time& selected_time, std::vector<rosbag::MessageInstance>& msgs);
  int getCurrentFrame() const;
  void setFinishProcessFlag(bool flag);
  int getMainRadarPclSize();
  ros::Time getMainRadarTime(int curIdx);
  void packetCallbackMsg();

private:
  void playLoop();

  rosbag::Bag bag_;

  // 雷达: /wf/radar/sofa_0 (arbe_msgs/sofaOutput)
  std::vector<rosbag::MessageInstance> radar_msgs_;

  // 电机: /wf/motor/motor_pub (arbe_msgs/sofaMotorOutput)
  std::vector<rosbag::MessageInstance> motor_msgs_;

  // 相机 (6路)
  std::vector<rosbag::MessageInstance> camera_msgs0_;
  std::vector<rosbag::MessageInstance> camera_msgs1_;
  std::vector<rosbag::MessageInstance> camera_msgs2_;
  std::vector<rosbag::MessageInstance> camera_msgs3_;
  std::vector<rosbag::MessageInstance> camera_msgs4_;
  std::vector<rosbag::MessageInstance> camera_msgs5_;

  std::vector<rosbag::MessageInstance> frame_msgs_;
  std::vector<int>    msg_flags_;

  int current_frame_;
  double play_rate_;
  std::mutex mutex_;
  std::condition_variable cv_;
  std::atomic<bool> playing_;
  std::atomic<bool> finishProcessFlag_;

  std::thread play_thread_;
  MessageCallback message_callback_;
  UpdateProgressBarCallback update_progress_bar_callback_;

  // frame_msgs_ 布局: [0]雷达 [1]电机 [2-7]相机
  static const int MAX_TOPIC_NUM = 8;
};

} // namespace my_rviz_plugin

#endif // MY_RVIZ_PLUGIN_BAG_READER_H
