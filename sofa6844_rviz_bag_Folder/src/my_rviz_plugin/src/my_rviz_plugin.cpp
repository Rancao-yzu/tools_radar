#include <pluginlib/class_list_macros.h>
#include <QHBoxLayout>
#include <QVBoxLayout>
#include <QFileDialog>
#include <QMainWindow>
#include "my_rviz_plugin/my_rviz_plugin.h"
#include <boost/date_time/posix_time/posix_time.hpp>
#include <QDir>
#include <QTimer>

namespace my_rviz_plugin
{
MyRvizPlugin::MyRvizPlugin(QWidget* parent)
  : rviz::Panel(parent), bag_reader_(new BagReader()),
  radar_frame_count_(0),
  current_bag_index_(-1), folder_mode_(false)
{
  nh_ = ros::NodeHandle();

  radar_pub_ = nh_.advertise<arbe_msgs::sofaOutput>("/wf/radar/sofa_0", 10);
  motor_pub_ = nh_.advertise<arbe_msgs::sofaMotorOutput>("/wf/motor/motor_pub", 10);

  camera_pub0_ = nh_.advertise<sensor_msgs::CompressedImage>("/cv_camera_0/image_raw/compressed", 10);
  camera_pub1_ = nh_.advertise<sensor_msgs::CompressedImage>("/cv_camera_1/image_raw/compressed", 10);
  camera_pub2_ = nh_.advertise<sensor_msgs::CompressedImage>("/cv_camera_2/image_raw/compressed", 10);
  camera_pub3_ = nh_.advertise<sensor_msgs::CompressedImage>("/cv_camera_3/image_raw/compressed", 10);
  camera_pub4_ = nh_.advertise<sensor_msgs::CompressedImage>("/cv_camera_4/image_raw/compressed", 10);
  camera_pub5_ = nh_.advertise<sensor_msgs::CompressedImage>("/cv_camera_5/image_raw/compressed", 10);

  spinner_ = new ros::AsyncSpinner(1);
  spinner_->start();

  service0_ = nh_.advertiseService("/play_single_frame_0", &MyRvizPlugin::handleServiceRequest, this);

  folder_path_ = new QLineEdit;
  select_folder_button_ = new QPushButton("Select Folder");
  current_bag_label_ = new QLabel("Current Bag: N/A");
  bag_file_path_ = new QLineEdit;

  select_button_ = new QPushButton("Select");
  read_button_ = new QPushButton("Read");
  play_button_ = new QPushButton("Play");
  stop_button_ = new QPushButton("Stop");
  step_forward_button_ = new QPushButton("step->");
  step_backward_button_ = new QPushButton("<-step");
  frame_spinner_ = new QSpinBox;
  step_spinner_ = new QSpinBox;
  frame_count_label_ = new QLabel("Frame Count: 0");
  frame_id_label_ = new QLabel("Frame ID: N/A  Timestamp: N/A");
  play_rate_combo_ = new QComboBox;
  frame_slider_ = new QSlider(Qt::Horizontal);
  progress_bar_ = new QProgressBar();

  select_button_->setFixedSize(50, 30);
  read_button_->setFixedSize(50, 30);
  play_button_->setFixedSize(40, 30);
  stop_button_->setFixedSize(40, 30);
  step_forward_button_->setFixedSize(50, 30);
  step_backward_button_->setFixedSize(50, 30);
  select_folder_button_->setFixedSize(100, 30);

  play_rate_combo_->addItem("1.0");
  play_rate_combo_->addItem("0.25");
  play_rate_combo_->addItem("0.5");
  play_rate_combo_->addItem("1.25");
  play_rate_combo_->addItem("1.5");
  play_rate_combo_->addItem("2.0");

  frame_spinner_->setMinimum(0);
  step_spinner_->setMinimum(1);
  step_spinner_->setValue(1);

  frame_slider_->setMinimum(0);
  frame_slider_->setFixedSize(380, 20);

  progress_bar_->setRange(0.0, 1.2);
  progress_bar_->setValue(0.0);

  QVBoxLayout* layout = new QVBoxLayout;
  QHBoxLayout* file_layout = new QHBoxLayout;
  file_layout->addWidget(bag_file_path_);
  file_layout->addWidget(select_button_);
  file_layout->addWidget(select_folder_button_);
  file_layout->addWidget(read_button_);

  QHBoxLayout* layoutnubmber = new QHBoxLayout;
  layoutnubmber->addWidget(frame_spinner_);
  layoutnubmber->addWidget(frame_slider_);

  QHBoxLayout* control_layout = new QHBoxLayout;
  control_layout->addWidget(step_backward_button_);
  control_layout->addWidget(step_forward_button_);
  control_layout->addWidget(play_button_);
  control_layout->addWidget(stop_button_);
  control_layout->addWidget(new QLabel("Play Rate:"));
  control_layout->addWidget(play_rate_combo_);

  layout->addLayout(file_layout);
  layout->addWidget(folder_path_);
  layout->addWidget(current_bag_label_);
  layout->addWidget(frame_id_label_);
  layout->addLayout(layoutnubmber);
  layout->addLayout(control_layout);
  layout->addWidget(progress_bar_);
  setLayout(layout);

  connect(select_button_, SIGNAL(clicked()), this, SLOT(selectBagFile()));
  connect(read_button_, SIGNAL(clicked()), this, SLOT(readBagFile()));
  connect(play_button_, SIGNAL(clicked()), this, SLOT(playBag()));
  connect(stop_button_, SIGNAL(clicked()), this, SLOT(stopBag()));
  connect(frame_spinner_, SIGNAL(valueChanged(int)), this, SLOT(jumpToFrame()));
  connect(step_forward_button_, SIGNAL(clicked()), this, SLOT(stepForward()));
  connect(step_backward_button_, SIGNAL(clicked()), this, SLOT(stepBackward()));
  connect(play_rate_combo_, SIGNAL(currentIndexChanged(int)), this, SLOT(updatePlayRate()));
  connect(frame_slider_, SIGNAL(valueChanged(int)), this, SLOT(sliderValueChanged(int)));
  connect(select_folder_button_, SIGNAL(clicked()), this, SLOT(selectFolder()));

  play_button_->setEnabled(false);
  stop_button_->setEnabled(false);
  frame_spinner_->setEnabled(false);
  step_spinner_->setEnabled(false);
  step_forward_button_->setEnabled(false);
  step_backward_button_->setEnabled(false);
  play_rate_combo_->setEnabled(false);
  frame_slider_->setEnabled(false);

  bag_reader_->setMessageCallback([this](const std::vector<rosbag::MessageInstance>& frame_msg,
                                         const int& frame_number,
                                         const std::vector<int>& msg_flag
                                        ) {
    publishClosestMessages(frame_msg, frame_number, msg_flag);
    updateSliderAndSpinner();

    if (folder_mode_ && frame_number >= bag_reader_->getMainRadarPclSize() - 1)
    {
      ROS_INFO("Reached end of bag file %d/%lu: %s",
              current_bag_index_ + 1, bag_files_.size(), bag_files_[current_bag_index_].c_str());

      QMetaObject::invokeMethod(this, [this]()
      {
        stopBag();

        if (current_bag_index_ < static_cast<int>(bag_files_.size()) - 1)
        {
          ROS_INFO("loading next bag file %d/%lu: %s", current_bag_index_ + 2, bag_files_.size(), bag_files_[current_bag_index_ + 1].c_str());
          readBagFile();

          QTimer::singleShot(1000, this, [this]()
          {
            ROS_INFO("Starting playback for new bag file (index: %d)", current_bag_index_);
            bContinuePlayFlag = true;
            playBag();
          });
        } else {
          ROS_WARN("No more bag files to load (%lu files processed)", bag_files_.size());
          current_bag_label_->setText("Current Bag: No more files");
          folder_mode_ = false;
          current_bag_index_ = -1;
          bag_file_path_->clear();
        }
      }, Qt::QueuedConnection);
    }
  });

  bag_reader_->setUpdateProgressBarCallback([this](float value){
    progress_bar_->setValue(value);
  });

  bContinuePlayFlag = false;
}

MyRvizPlugin::~MyRvizPlugin()
{
  delete bag_reader_;
  spinner_->stop();
  delete spinner_;
}

bool MyRvizPlugin::handleServiceRequest(wf_srvs::PlaySingleFrame::Request &req,
                            wf_srvs::PlaySingleFrame::Response &res)
{
    if(req.status == 3)
      ROS_INFO("Received data: radar_pos = %d, frame_id = %d", req.radar_pos, req.frame_id);
    else if(req.status == 4)
      bag_reader_->setFinishProcessFlag(true);
    else
      ROS_INFO("Play single frame service error!");

    res.success = true;
    return true;
}

void MyRvizPlugin::publishClosestMessages(const std::vector<rosbag::MessageInstance>& frame_msg,
                                          const int& frame_number,
                                          const std::vector<int>& msg_flag
                                          )
{
  // frame_msg 布局: [0]雷达 [1]电机 [2-7]相机
  boost::posix_time::time_duration time_offset(8, 0, 0);

  // 雷达数据 sofaOutput
  boost::shared_ptr<arbe_msgs::sofaOutput> radar_data = frame_msg[0].instantiate<arbe_msgs::sofaOutput>();
  if (radar_data && (msg_flag[0] >= 0))
  {
    frame_id_label_->setText(QString("-Frame ID: %1").arg(radar_data->frameID)
      + "  -Timestamp: " + QString(boost::posix_time::to_simple_string(
        radar_data->header.stamp.toBoost() + time_offset).c_str()));
    radar_pub_.publish(*radar_data);
  }

  // 电机数据 sofaMotorOutput
  boost::shared_ptr<arbe_msgs::sofaMotorOutput> motor_data = frame_msg[1].instantiate<arbe_msgs::sofaMotorOutput>();
  if (motor_data && (msg_flag[1] >= 0))
  {
    motor_pub_.publish(*motor_data);
    ROS_INFO("motor_pub_.publish");
  }


  // 相机数据
  auto HandleCameraData = [&](int frame_idx, int flag_idx, ros::Publisher& pub)
  {
    auto data = frame_msg[frame_idx].instantiate<sensor_msgs::CompressedImage>();
    if (data && (msg_flag[flag_idx] >= 0))
        pub.publish(*data);
  };
  HandleCameraData(2, 2, camera_pub0_);
  HandleCameraData(3, 3, camera_pub1_);
  HandleCameraData(4, 4, camera_pub2_);
  HandleCameraData(5, 5, camera_pub3_);
  HandleCameraData(6, 6, camera_pub4_);
  HandleCameraData(7, 7, camera_pub5_);
}

void MyRvizPlugin::selectBagFile()
{
  QString file = QFileDialog::getOpenFileName(this, "Select Bag File", "", "Bag Files (*.bag)");
  if (!file.isEmpty())
  {
    bag_file_path_->setText(file);
    folder_mode_ = false;
    current_bag_index_ = -1;
    current_bag_label_->setText("Current Bag: N/A");
  }
}

void MyRvizPlugin::selectFolder()
{
  QString folder = QFileDialog::getExistingDirectory(this, "Select Bag Folder", "");
  if (!folder.isEmpty())
  {
    folder_path_->setText(folder);
    folder_mode_ = true;
    bag_files_.clear();
    current_bag_index_ = -1;

    QDir dir(folder); // 使用QDir类来遍历文件夹内容
    QStringList filters;
    filters << "*.bag";
    dir.setNameFilters(filters);
    QStringList bag_list = dir.entryList(QDir::Files | QDir::NoDotAndDotDot, QDir::Name);

    for (const QString& bag_file : bag_list)
      bag_files_.push_back(dir.filePath(bag_file).toStdString());
    

    if (!bag_files_.empty())
    {
      bag_file_path_->setText(QString::fromStdString(bag_files_[0]));
      current_bag_label_->setText("Current Bag: N/A");
      ROS_INFO("Found %lu bag files in folder: %s", bag_files_.size(), folder.toStdString().c_str());
    }else {
      ROS_WARN("No bag files found in folder: %s", folder.toStdString().c_str());
      folder_mode_ = false;
      current_bag_label_->setText("Current Bag: N/A");
      bag_file_path_->clear();
    }
  }
}

void MyRvizPlugin::readBagFile()
{
  std::string path;
  if (folder_mode_)// 判断是否处于文件夹模式（批量播放模式）
  {
    if (current_bag_index_ >= static_cast<int>(bag_files_.size()) - 1)// 是否已经处理完所有bag文件
    {
      current_bag_label_->setText("Current Bag: No more files");
      folder_mode_ = false;
      current_bag_index_ = -1;
      bag_file_path_->clear();
      return;
    }
    current_bag_index_++;
    path = bag_files_[current_bag_index_];
    bag_file_path_->setText(QString::fromStdString(path));
  }
  else// 单文件模式：直接获取界面上输入的文件路径
  {
    path = bag_file_path_->text().toStdString();
  }

  if (!path.empty())
  {
    // 重置进度条和帧号
    progress_bar_->setValue(0);
    frame_spinner_->setValue(0);
    frame_slider_->setValue(0);

    play_button_->setEnabled(false);
    stop_button_->setEnabled(false);
    frame_spinner_->setEnabled(false);
    step_spinner_->setEnabled(false);
    step_forward_button_->setEnabled(false);
    step_backward_button_->setEnabled(false);
    play_rate_combo_->setEnabled(false);
    frame_slider_->setEnabled(false);
    current_bag_label_->setText("Current Bag: " + QString::fromStdString(path));

    bag_reader_->readBagFile(path, radar_frame_count_);

    frame_count_label_->setText("Frame Count: " + QString::number(radar_frame_count_));

    frame_spinner_->setMaximum(radar_frame_count_);
    frame_slider_->setMaximum(radar_frame_count_);

    play_button_->setEnabled(true);
    stop_button_->setEnabled(false);
    frame_spinner_->setEnabled(true);
    step_spinner_->setEnabled(true);
    step_forward_button_->setEnabled(true);
    step_backward_button_->setEnabled(true);
    play_rate_combo_->setEnabled(true);
    frame_slider_->setEnabled(true);

    ROS_INFO("———Bag file read and cached success———: %s", path.c_str());
  }
}

void MyRvizPlugin::jumpToFrame()
{
  if(!bContinuePlayFlag)
  {
    int frame_number = frame_spinner_->value();
    if (frame_number >= -1 )
      bag_reader_->jumpToFrame(frame_number);
  }
}

void MyRvizPlugin::stepForward()
{
  int step = step_spinner_->value();
  int new_frame = std::min(frame_spinner_->value() + step, frame_spinner_->maximum() - 1);
  frame_spinner_->setValue(new_frame);
}

void MyRvizPlugin::stepBackward()
{
  int step = step_spinner_->value();
  int new_frame = std::max(frame_spinner_->value() - step, frame_spinner_->minimum());
  frame_spinner_->setValue(new_frame);
}
void MyRvizPlugin::playBag()
{
  bContinuePlayFlag = true;
  bag_reader_->playBag();

  play_button_->setEnabled(false);
  stop_button_->setEnabled(true);
  frame_spinner_->setEnabled(false);
  step_spinner_->setEnabled(false);
  step_forward_button_->setEnabled(false);
  step_backward_button_->setEnabled(false);
  play_rate_combo_->setEnabled(false);
  frame_slider_->setEnabled(false);
}

void MyRvizPlugin::stopBag()
{
  bContinuePlayFlag = false;
  bag_reader_->stopBag();

  play_button_->setEnabled(true);
  stop_button_->setEnabled(false);
  step_spinner_->setEnabled(true);
  frame_spinner_->setEnabled(true);
  step_forward_button_->setEnabled(true);
  step_backward_button_->setEnabled(true);
  play_rate_combo_->setEnabled(true);
  frame_slider_->setEnabled(true);
}


void MyRvizPlugin::updatePlayRate()
{
  double rate = play_rate_combo_->currentText().toDouble();
  bag_reader_->setPlayRate(rate);
}

void MyRvizPlugin::sliderValueChanged(int value)//播放进度条
{

  if(frame_spinner_->value() >= frame_spinner_->maximum() || value >= frame_spinner_->maximum())
  {
    frame_spinner_->setValue(0);
    frame_slider_->setValue(0);
  }else{
    frame_spinner_->setValue(value);
  }
}

void MyRvizPlugin::updateSliderAndSpinner()
{
  frame_slider_->setValue(bag_reader_->getCurrentFrame());
}

} // namespace my_rviz_plugin

PLUGINLIB_EXPORT_CLASS(my_rviz_plugin::MyRvizPlugin, rviz::Panel)
