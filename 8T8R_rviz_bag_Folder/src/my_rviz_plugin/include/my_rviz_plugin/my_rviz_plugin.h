#ifndef MY_RVIZ_PLUGIN_MY_RVIZ_PLUGIN_H
#define MY_RVIZ_PLUGIN_MY_RVIZ_PLUGIN_H

#include <rviz/panel.h>
#include <QPushButton>
#include <QLineEdit>
#include <QWidget>
#include <QLabel>
#include <QSpinBox>
#include <QComboBox>
#include <QCheckBox>
#include <QSlider>
#include <QProgressBar>
#include <functional>
#include "bag_reader.h"
// ROS 相关头文件
#include <ros/ros.h>
#include <sensor_msgs/CompressedImage.h>
#include <sensor_msgs/PointCloud2.h>
#include <arbe_msgs/wfRawDataMsg.h>
#include <arbe_msgs/wfTiFrameRD.h>
#include <arbe_msgs/VehStatusOutput.h>
#include <wf_srvs/PlaySingleFrame.h>
#include <std_msgs/UInt8MultiArray.h>

namespace my_rviz_plugin
{

class MyRvizPlugin : public rviz::Panel
{
Q_OBJECT
public:
  MyRvizPlugin(QWidget* parent = 0);
  ~MyRvizPlugin() override;

protected Q_SLOTS:
  void selectBagFile();
  void readBagFile();
  void jumpToFrame();
  void playBag();
  void stopBag();
  void stepForward();
  void stepBackward();
  void updatePlayRate();
  void sliderValueChanged(int value);
  void setPlayFloatFlag(int flag);
  void setPlaySPFlag(int flag);
  void selectMainRadar();
  void selectFolder();

private:
  void updateSliderAndSpinner();
  // 添加函数声明
  void recreateBagReader();  // 添加这一行 2025/9/17
  void publishClosestMessages(const std::vector<rosbag::MessageInstance>& frame_msg,  
                              const int& frame_number,
                              const std::vector<int>& msg_flag
                              );//const rosbag::MessageInstance& car_msg

  bool handleServiceRequest(wf_srvs::PlaySingleFrame::Request &req,
                            wf_srvs::PlaySingleFrame::Response &res);

  QLineEdit* bag_file_path_;
  QPushButton* select_button_;
  QPushButton* read_button_;
  QPushButton* play_button_;
  QPushButton* stop_button_;
  QPushButton* step_forward_button_;
  QPushButton* step_backward_button_;
  QSpinBox* frame_spinner_;
  QSpinBox* step_spinner_;
  QLabel* frame_count_label_;
  QLabel* frame_sp_count_label_;
  QLabel* frame_id_label_;
  QComboBox* play_rate_combo_;
  QCheckBox* play_float_data_;
  QCheckBox* play_sp_data_;
  QComboBox* select_main_radar_;
  QSlider* frame_slider_;
  QProgressBar* progress_bar_;


  QLineEdit* folder_path_;           // 新增：文件夹路径输入框
  QPushButton* select_folder_button_; // 新增：选择文件夹按钮
  QLabel* current_bag_label_;        // 新增：当前 bag 文件标签

  // 添加 ROS 发布者的声明
  ros::Publisher car_pub_;
  ros::Publisher camera_pub0_;
  ros::Publisher camera_pub1_;
  ros::Publisher camera_pub2_;
  ros::Publisher camera_pub3_;
  ros::Publisher camera_pub4_;
  ros::Publisher camera_pub5_;

  ros::Publisher pointcloud_pub0_;
  ros::Publisher pointcloud_pub1_;
  ros::Publisher pointcloud_pub2_;
  ros::Publisher pointcloud_pub3_;
  ros::Publisher pointcloud_pub4_;

  ros::Publisher pointcloud_sp_pub0_;
  ros::Publisher pointcloud_sp_pub1_;
  ros::Publisher pointcloud_sp_pub2_;
  ros::Publisher pointcloud_sp_pub3_;
  ros::Publisher pointcloud_sp_pub4_;

  ros::Publisher cube_data_pub0_;
  ros::Publisher cube_data_pub1_;
  ros::Publisher cube_data_pub2_;
  ros::Publisher cube_data_pub3_;
  ros::Publisher cube_data_pub4_;

  // ros节点
  ros::NodeHandle nh_;

  // ros服务
  ros::ServiceServer service0_;
  ros::ServiceServer service1_;
  ros::ServiceServer service2_;
  ros::ServiceServer service3_;
  ros::ServiceServer service4_;

  // 异步 spinner
  ros::AsyncSpinner* spinner_;      

  int frame_count0, frame_sp_count0;
  int frame_count1, frame_sp_count1;
  int frame_count2, frame_sp_count2;
  int frame_count3, frame_sp_count3;
  int frame_count4, frame_sp_count4;

  volatile bool bContinuePlayFlag;
  bool bPlayFloatData_;
  bool bPlaySPData_;
  int mainRadarIndex_;

    std::vector<std::string> bag_files_; // 新增：bag 文件路径列表
  int current_bag_index_;             // 新增：当前 bag 文件索引
  bool folder_mode_;                  // 新增：文件夹模式标志

public:
  BagReader* bag_reader_;
};

} // namespace my_rviz_plugin

#endif // MY_RVIZ_PLUGIN_MY_RVIZ_PLUGIN_H
