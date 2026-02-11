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
                                            )>;//, const rosbag::MessageInstance&
  using UpdateProgressBarCallback = std::function<void(float)>;

  BagReader();
  ~BagReader();

  void readBagFile(const std::string& file_path, int& frameCount0, int& frameSPCount0,int& frameCount1, int& frameSPCount1, int& frameCount2, 
  int& frameSPCount2,int& frameCount3, int& frameSPCount3,int& frameCount4, int& frameSPCount4);
  void readBagFile(const std::string& file_path, int& frameCount0, int& frameCount1, int& frameCount2, 
  int& frameCount3, int& frameCount4);
  void jumpToFrame(int frame_number);
  void initialize();
  void playBag();
  void stopBag();
  void setPlayRate(double rate);
  void setMessageCallback(MessageCallback callback);
  void setUpdateProgressBarCallback(UpdateProgressBarCallback callback);
  int findClosestCarFrame(const ros::Time& selected_time);
  int findClosestCameraFrame(const ros::Time& selected_time, std::vector<rosbag::MessageInstance>& camera_msgs);
  int findClosestPtFrame(const ros::Time& selected_time, std::vector<rosbag::MessageInstance>& pcl_msgs);
  int getCurrentFrame() const;
  void setFinishProcessFlag(bool flag);
  void selectMainRadar(int index);
  int getMainRadarPclSize();
  ros::Time getMainRadarTime(int curIdx);
  void packetCallbackMsg();

private:
  void playLoop();

  rosbag::Bag bag_;
  std::vector<rosbag::MessageInstance> car_msgs_;       // "/wf/car_id6/parsed" 消息

  std::vector<rosbag::MessageInstance> camera_msgs0_;
  std::vector<rosbag::MessageInstance> camera_msgs1_;    // "/cv_camera_1/image_raw/compressed" 消息
  std::vector<rosbag::MessageInstance> camera_msgs2_;
  std::vector<rosbag::MessageInstance> camera_msgs3_;
  std::vector<rosbag::MessageInstance> camera_msgs4_;
  std::vector<rosbag::MessageInstance> camera_msgs5_;
  

  std::vector<rosbag::MessageInstance> corner_radar_warning_msgs_;
  
  std::vector<rosbag::MessageInstance> pointcloud_msgs0_;
  std::vector<rosbag::MessageInstance> pointcloud_msgs1_;     // "/wf/corner_radar/lgu_data__0" 消息
  std::vector<rosbag::MessageInstance> pointcloud_msgs2_;
  std::vector<rosbag::MessageInstance> pointcloud_msgs3_;
  std::vector<rosbag::MessageInstance> pointcloud_msgs4_;
  std::vector<rosbag::MessageInstance> pointcloud_msgs5_;


  std::vector<rosbag::MessageInstance> IMU_msgs_;
  std::vector<rosbag::MessageInstance> frame_msgs_; 
  std::vector<rosbag::MessageInstance> empty_msgs_; 
  std::vector<int>    msg_flags_;
  int current_frame_;// 当前帧索引
  double ratio_;
  double play_rate_;
  std::mutex mutex_;
  std::condition_variable cv_;
  std::atomic<bool> playing_;
  std::atomic<bool> finishProcessFlag_;
  std::atomic<bool> bPlaySPFlag_;
  std::atomic<int>  mainRadarIndex_;

  std::thread play_thread_;
  MessageCallback message_callback_;
  UpdateProgressBarCallback update_progress_bar_callback_;
  std::mutex frame_mutex;
  const int MAX_TOPIC_NUM = 14;
};

} // namespace my_rviz_plugin

#endif // MY_RVIZ_PLUGIN_BAG_READER_H
