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
  // 区间消息集合:每个topic在该时间区间内的所有msg(按topic索引分组)
  using IntervalMessages = std::vector<std::vector<rosbag::MessageInstance>>;
  // 回调:某时间区间内各topic的消息集合 + 区间索引
  using MessageCallback = std::function<void(const IntervalMessages&, const int&)>;
  using UpdateProgressBarCallback = std::function<void(float)>;

  BagReader();
  ~BagReader();

  void readBagFile(const std::string& file_path, int& frameCount0, int& frameSPCount0,int& frameCount1, int& frameSPCount1, int& frameCount2, 
  int& frameSPCount2,int& frameCount3, int& frameSPCount3,int& frameCount4, int& frameSPCount4);
  void jumpToFrame(int interval_index);  // 跳转到指定时间区间并发布该区间消息
  void initialize();
  void playBag();
  void stopBag();
  void setPlayRate(double rate);
  void setMessageCallback(MessageCallback callback);
  void setUpdateProgressBarCallback(UpdateProgressBarCallback callback);
  int getCurrentFrame() const;            // 当前区间索引
  void setFinishProcessFlag(bool flag);   // 步进阻塞:外部service确认算法处理OK后置true以推进下一区间
  void setSPFlag(bool flag);
  void selectMainRadar(int index);
  ros::Time getMainRadarTime(int curIdx); // 主雷达某帧时间(用于确定T0)
  // 以主雷达首帧时间为T0,按INTERVAL_SEC切片得到的区间总数
  int getIntervalCount() const;
  // 收集[start,end)时间区间内各topic的所有msg(按topic索引分组,二分查找+顺序扫描)
  void collectIntervalMessages(const ros::Time& start, const ros::Time& end, IntervalMessages& out);

private:
  void playLoop();
  // 主雷达当前(由mainRadarIndex_/bPlaySPFlag_决定)的消息数量
  int getMainRadarSize() const;
  // 依据主雷达首尾帧时间重算t0_与interval_count_(切换主雷达/SP时调用)
  void recomputeIntervals();

  rosbag::Bag bag_;
  std::vector<rosbag::MessageInstance> car_msgs_;       // "/wf/car_id6/parsed" 消息

  std::vector<rosbag::MessageInstance> camera_msgs0_;
  std::vector<rosbag::MessageInstance> camera_msgs1_;    // "/cv_camera_1/image_raw/compressed" 消息
  std::vector<rosbag::MessageInstance> camera_msgs2_;
  std::vector<rosbag::MessageInstance> camera_msgs3_;
  std::vector<rosbag::MessageInstance> camera_msgs4_;
  std::vector<rosbag::MessageInstance> camera_msgs5_;
  
  std::vector<rosbag::MessageInstance> pointcloud_msgs0_;
  std::vector<rosbag::MessageInstance> pointcloud_msgs1_;     // "/wf/corner_radar/parsed/float_data_0" 消息
  std::vector<rosbag::MessageInstance> pointcloud_msgs2_;
  std::vector<rosbag::MessageInstance> pointcloud_msgs3_;
  std::vector<rosbag::MessageInstance> pointcloud_msgs4_;
  std::vector<rosbag::MessageInstance> pointcloud_msgs5_;

  std::vector<rosbag::MessageInstance> pointcloud_sp_msgs0_;
  std::vector<rosbag::MessageInstance> pointcloud_sp_msgs1_;
  std::vector<rosbag::MessageInstance> pointcloud_sp_msgs2_;
  std::vector<rosbag::MessageInstance> pointcloud_sp_msgs3_;
  std::vector<rosbag::MessageInstance> pointcloud_sp_msgs4_;
  std::vector<rosbag::MessageInstance> pointcloud_sp_msgs5_;

  std::vector<rosbag::MessageInstance> pointcloud_sgu_msgs1_;     // "/wf/corner_radar/sgu_data_" 消息
  std::vector<rosbag::MessageInstance> pointcloud_sgu_msgs2_;
  std::vector<rosbag::MessageInstance> pointcloud_sgu_msgs3_;
  std::vector<rosbag::MessageInstance> pointcloud_sgu_msgs4_;

  std::vector<rosbag::MessageInstance> pointcloud_gt_msgs3_;
  std::vector<rosbag::MessageInstance> pointcloud_gt_msgs4_;

  std::vector<rosbag::MessageInstance> corner_radar_warning_msgs_;

  std::vector<rosbag::MessageInstance> sgu_adas_input_data1_;
  std::vector<rosbag::MessageInstance> sgu_adas_input_data2_;
  std::vector<rosbag::MessageInstance> sgu_adas_input_data3_;
  std::vector<rosbag::MessageInstance> sgu_adas_input_data4_;

  std::vector<rosbag::MessageInstance> IMU_msgs_;

  int current_frame_;// 当前区间索引
  double ratio_;
  double play_rate_;
  std::mutex mutex_;
  std::condition_variable cv_;
  std::atomic<bool> playing_;
  std::atomic<bool> finishProcessFlag_;  // 步进阻塞标志:false=等待外部确认,true=可推进下一区间
  std::atomic<bool> bPlaySPFlag_;
  std::atomic<int>  mainRadarIndex_;

  std::thread play_thread_;
  MessageCallback message_callback_;
  UpdateProgressBarCallback update_progress_bar_callback_;
  std::mutex frame_mutex;
  const int MAX_TOPIC_NUM = 21;
  // 时间区间切片参数:以主雷达首帧时间为T0,每15ms一个区间
  static const double INTERVAL_SEC; // 15ms/区间,定义在bag_reader.cpp
  ros::Time t0_;                // 切片起始时间(主雷达首帧)
  int interval_count_;          // 区间总数
  IntervalMessages interval_msgs_; // 复用缓冲,避免每区间重复分配
};

} // namespace my_rviz_plugin

#endif // MY_RVIZ_PLUGIN_BAG_READER_H
