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
#include <arbe_msgs/sofaOutput.h>
#include <arbe_msgs/sofaMotorOutput.h>
#include <wf_srvs/PlaySingleFrame.h>

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
  void selectFolder();

private:
  void updateSliderAndSpinner();
  void publishClosestMessages(const std::vector<rosbag::MessageInstance>& frame_msg,  
                              const int& frame_number,
                              const std::vector<int>& msg_flag
                              );

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
  QLabel* frame_id_label_;
  QComboBox* play_rate_combo_;
  QSlider* frame_slider_;
  QProgressBar* progress_bar_;


  QLineEdit* folder_path_;           // 新增：文件夹路径输入框
  QPushButton* select_folder_button_; // 新增：选择文件夹按钮
  QLabel* current_bag_label_;        // 新增：当前 bag 文件标签

  // 添加 ROS 发布者的声明
  ros::Publisher radar_pub_;
  ros::Publisher motor_pub_;

  ros::Publisher camera_pub0_;
  ros::Publisher camera_pub1_;
  ros::Publisher camera_pub2_;
  ros::Publisher camera_pub3_;
  ros::Publisher camera_pub4_;
  ros::Publisher camera_pub5_;

  // ros节点
  ros::NodeHandle nh_;

  // ros服务
  ros::ServiceServer service0_;

  // 异步 spinner
  ros::AsyncSpinner* spinner_;

  int radar_frame_count_;

  volatile bool bContinuePlayFlag;

  std::vector<std::string> bag_files_;
  int current_bag_index_;
  bool folder_mode_;

public:
  BagReader* bag_reader_;
};

} // namespace my_rviz_plugin

#endif // MY_RVIZ_PLUGIN_MY_RVIZ_PLUGIN_H
