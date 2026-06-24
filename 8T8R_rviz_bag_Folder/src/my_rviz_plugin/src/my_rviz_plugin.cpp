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
{// 构造函数：初始化UI组件、ROS publisher、subscriber、服务和插件核心对象 BagReader
MyRvizPlugin::MyRvizPlugin(QWidget* parent)
  : rviz::Panel(parent), bag_reader_(new BagReader()), 
  frame_count0(0), frame_sp_count0(0), 
  frame_count1(0), frame_sp_count1(0), frame_count2(0), frame_sp_count2(0), 
  frame_count3(0), frame_sp_count3(0), frame_count4(0), frame_sp_count4(0),
  mainRadarIndex_(0), current_bag_index_(-1), folder_mode_(false)
  //mainRadarIndex_ 在 MyRvizPlugin 构造函数里 没有初始化 ，导致 readBagFile 中计算 maxVal 时是垃圾值
{
  nh_ = ros::NodeHandle();

  pointcloud_pub0_ = nh_.advertise<arbe_msgs::wfRawDataMsg>("/wf/front_radar/parsed/float_data_0", 10);
  pointcloud_pub1_ = nh_.advertise<arbe_msgs::wfRawDataMsg>("/wf/front_radar/parsed/float_data_1", 10);
  pointcloud_pub2_ = nh_.advertise<arbe_msgs::wfRawDataMsg>("/wf/front_radar/parsed/float_data_2", 10);
  pointcloud_pub3_ = nh_.advertise<arbe_msgs::wfRawDataMsg>("/wf/front_radar/parsed/float_data_3", 10);
  pointcloud_pub4_ = nh_.advertise<arbe_msgs::wfRawDataMsg>("/wf/front_radar/parsed/float_data_4", 10);

  pointcloud_sp_pub0_ = nh_.advertise<arbe_msgs::wfTiFrameRD>("/wf/frame_rd_data/ti/radar_0", 10);
  pointcloud_sp_pub1_ = nh_.advertise<arbe_msgs::wfTiFrameRD>("/wf/frame_rd_data/ti/radar_1", 10);
  pointcloud_sp_pub2_ = nh_.advertise<arbe_msgs::wfTiFrameRD>("/wf/frame_rd_data/ti/radar_2", 10);
  pointcloud_sp_pub3_ = nh_.advertise<arbe_msgs::wfTiFrameRD>("/wf/frame_rd_data/ti/radar_3", 10);
  pointcloud_sp_pub4_ = nh_.advertise<arbe_msgs::wfTiFrameRD>("/wf/frame_rd_data/ti/radar_4", 10);

  cube_data_pub0_ = nh_.advertise<std_msgs::UInt8MultiArray>("/wf/front_radar/ti/cube_data_0", 10);
  cube_data_pub1_ = nh_.advertise<std_msgs::UInt8MultiArray>("/wf/front_radar/ti/cube_data_1", 10);
  cube_data_pub2_ = nh_.advertise<std_msgs::UInt8MultiArray>("/wf/front_radar/ti/cube_data_2", 10);
  cube_data_pub3_ = nh_.advertise<std_msgs::UInt8MultiArray>("/wf/front_radar/ti/cube_data_3", 10);
  cube_data_pub4_ = nh_.advertise<std_msgs::UInt8MultiArray>("/wf/front_radar/ti/cube_data_4", 10);

  camera_pub0_ = nh_.advertise<sensor_msgs::CompressedImage>("/cv_camera_0/image_raw/compressed", 10);  
  camera_pub1_ = nh_.advertise<sensor_msgs::CompressedImage>("/cv_camera_1/image_raw/compressed", 10);
  camera_pub2_ = nh_.advertise<sensor_msgs::CompressedImage>("/cv_camera_2/image_raw/compressed", 10);
  camera_pub3_ = nh_.advertise<sensor_msgs::CompressedImage>("/cv_camera_3/image_raw/compressed", 10);
  camera_pub4_ = nh_.advertise<sensor_msgs::CompressedImage>("/cv_camera_4/image_raw/compressed", 10);
  camera_pub5_ = nh_.advertise<sensor_msgs::CompressedImage>("/cv_camera_5/image_raw/compressed", 10);

  car_pub_ = nh_.advertise<arbe_msgs::VehStatusOutput>("/wf/car_id6/parsed2", 1);

  spinner_ = new ros::AsyncSpinner(1);    // 创建异步 spinner，并指定使用 1 个线程
  spinner_->start();
  
  // 创建一个服务并注册回调函数
  service0_ = nh_.advertiseService("/play_single_frame_0", &MyRvizPlugin::handleServiceRequest, this);
  service1_ = nh_.advertiseService("/play_single_frame_1", &MyRvizPlugin::handleServiceRequest, this);
  service2_ = nh_.advertiseService("/play_single_frame_2", &MyRvizPlugin::handleServiceRequest, this);
  service3_ = nh_.advertiseService("/play_single_frame_3", &MyRvizPlugin::handleServiceRequest, this);
  service4_ = nh_.advertiseService("/play_single_frame_4", &MyRvizPlugin::handleServiceRequest, this);

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
  frame_count_label_ = new QLabel("Frame Count: Radar(1-LT) 0;Radar(2-RT) 0;Radar(3-LB) 0;Radar(4-RB) 0");
  frame_sp_count_label_ = new QLabel("Frame Count(SP): Radar(1-LT) 0;Radar(2-RT) 0;Radar(3-LB) 0;Radar(4-RB) 0");
  frame_id_label_ = new QLabel("Frame ID: N/A  Timestamp: N/A");
  play_rate_combo_ = new QComboBox;
  play_sp_data_ = new QCheckBox("SP");
  play_sp_data_->setChecked(true);
  select_main_radar_ = new QComboBox;
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

  select_main_radar_->addItem("前雷达(0)");
  select_main_radar_->addItem("前左角(1)");
  select_main_radar_->addItem("前右角(2)");
  select_main_radar_->addItem("后左角(3)");
  select_main_radar_->addItem("后右角(4)");
  select_main_radar_->addItem("后雷达(5)");

  frame_spinner_->setMinimum(0);
  step_spinner_->setMinimum(1);
  step_spinner_->setValue(1);

  frame_slider_->setMinimum(0);
  frame_slider_->setFixedSize(380, 20);//帧号显示

  progress_bar_->setRange(0.0,1.2);
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
  control_layout->addWidget(play_sp_data_);
  control_layout->addWidget(select_main_radar_);

  layout->addLayout(file_layout);
  //layout->addWidget(frame_count_label_);
  //layout->addWidget(frame_sp_count_label_);
  layout->addWidget(folder_path_);
  layout->addWidget(current_bag_label_);
  layout->addWidget(frame_id_label_);
  layout->addLayout(layoutnubmber);
  //layout->addWidget(frame_spinner_);
  //layout->addWidget(frame_slider_);
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
  connect(play_sp_data_, SIGNAL(stateChanged(int)), this, SLOT(setPlaySPFlag(int)));
  connect(select_main_radar_, SIGNAL(currentIndexChanged(int)), this, SLOT(selectMainRadar()));
  connect(select_folder_button_, SIGNAL(clicked()), this, SLOT(selectFolder()));

  play_button_->setEnabled(false);
  stop_button_->setEnabled(false);
  frame_spinner_->setEnabled(false);
  step_spinner_->setEnabled(false);
  step_forward_button_->setEnabled(false);
  step_backward_button_->setEnabled(false);
  play_rate_combo_->setEnabled(false);
  frame_slider_->setEnabled(false);
  play_sp_data_->setEnabled(false);
  select_main_radar_->setEnabled(false);

  bag_reader_->setMessageCallback([this](const std::vector<rosbag::MessageInstance>& frame_msg,  
                                         const int& frame_number,
                                         const std::vector<int>& msg_flag
                                        ) {
    publishClosestMessages(frame_msg,frame_number,msg_flag);
    updateSliderAndSpinner();
           
    // 如果处于文件夹模式且已到达当前 bag 文件的最后一帧
    if (folder_mode_ && frame_number >= bag_reader_->getMainRadarPclSize() - 1) 
    {
      ROS_INFO("Reached end of bag file %d/%lu: %s, preparing to stop and load next",
              current_bag_index_ + 1, bag_files_.size(), bag_files_[current_bag_index_].c_str());

      // 使用 QMetaObject::invokeMethod 在主线程中执行
      QMetaObject::invokeMethod(this, [this]()
      {
        stopBag();
      
        if (current_bag_index_ < static_cast<int>(bag_files_.size()) - 1) // 判断是否还有未处理的 bag 文件
        {
          ROS_INFO("loading next bag file %d/%lu: %s",current_bag_index_ + 2, bag_files_.size(), bag_files_[current_bag_index_ + 1].c_str());

          readBagFile();
          
          QTimer::singleShot(1000, this, [this]() 
          {// 延迟调用 playBag，确保 readBagFile 完成
            ROS_INFO("Starting playback for new bag file (index: %d)", current_bag_index_);
            bContinuePlayFlag = true; // 确保播放标志为 true
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
  });//设置回调：当BagReader有新帧数据时，发布ROS消息并更新UI

  bag_reader_->setUpdateProgressBarCallback([this](float value){
    progress_bar_->setValue(value);
  });// 设置进度条更新回调

  bContinuePlayFlag = false;
  bPlaySPData_ = true;

  select_main_radar_->setCurrentIndex(0); // 默认选择
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
    if(mainRadarIndex_ == req.radar_pos)// 显示接收到的请求数据
    {
      if(req.status == 0)
        ROS_INFO("Received data: radar_pos = %d, frame_id = %d", req.radar_pos, req.frame_id);
      else if(req.status == 1)
        bag_reader_->setFinishProcessFlag(true);
      else
        ROS_INFO("Play single frame service error!");

      res.success = true;
    }
    return true;  // 表示服务成功响应
}

void MyRvizPlugin::publishClosestMessages(const std::vector<rosbag::MessageInstance>& frame_msg,  
                                          const int& frame_number,
                                          const std::vector<int>& msg_flag
                                          )
{ // 发布当前帧对应的所有传感器消息到ROS网络
  // frame_msg 布局: [0-4]float_data或SP_data [5-10]camera [11]car [12-16]cube_data
  boost::posix_time::time_duration time_offset(8, 0, 0);// 设置时区偏移

  // car 信息
  boost::shared_ptr<arbe_msgs::VehStatusOutput> car_status = frame_msg[11].instantiate<arbe_msgs::VehStatusOutput>();
  if (car_status&&(msg_flag[11]>=0))
    car_pub_.publish(*car_status);

  if(bPlaySPData_) // SP模式：播放SP_data + cube_data
  {
    boost::shared_ptr<arbe_msgs::wfTiFrameRD> sp_data0 = frame_msg[0].instantiate<arbe_msgs::wfTiFrameRD>();
    if (sp_data0 && (msg_flag[0]>=0))
    {
      frame_id_label_->setText(QString("-Frame ID: %1").arg(sp_data0->frameID)
        +"  -Timestamp: "+QString(boost::posix_time::to_simple_string(
          sp_data0->header.stamp.toBoost()+time_offset).c_str()));
      pointcloud_sp_pub0_.publish(*sp_data0);
    }
    boost::shared_ptr<arbe_msgs::wfTiFrameRD> sp_data1 = frame_msg[1].instantiate<arbe_msgs::wfTiFrameRD>();
    if (sp_data1 && (msg_flag[1]>=0)) pointcloud_sp_pub1_.publish(*sp_data1);
    boost::shared_ptr<arbe_msgs::wfTiFrameRD> sp_data2 = frame_msg[2].instantiate<arbe_msgs::wfTiFrameRD>();
    if (sp_data2 && (msg_flag[2]>=0)) pointcloud_sp_pub2_.publish(*sp_data2);
    boost::shared_ptr<arbe_msgs::wfTiFrameRD> sp_data3 = frame_msg[3].instantiate<arbe_msgs::wfTiFrameRD>();
    if (sp_data3 && (msg_flag[3]>=0)) pointcloud_sp_pub3_.publish(*sp_data3);
    boost::shared_ptr<arbe_msgs::wfTiFrameRD> sp_data4 = frame_msg[4].instantiate<arbe_msgs::wfTiFrameRD>();
    if (sp_data4 && (msg_flag[4]>=0)) pointcloud_sp_pub4_.publish(*sp_data4);

    // cube_data
    boost::shared_ptr<std_msgs::UInt8MultiArray> cube0 = frame_msg[12].instantiate<std_msgs::UInt8MultiArray>();
    if (cube0 && (msg_flag[12]>=0)) cube_data_pub0_.publish(*cube0);
    boost::shared_ptr<std_msgs::UInt8MultiArray> cube1 = frame_msg[13].instantiate<std_msgs::UInt8MultiArray>();
    if (cube1 && (msg_flag[13]>=0)) cube_data_pub1_.publish(*cube1);
    boost::shared_ptr<std_msgs::UInt8MultiArray> cube2 = frame_msg[14].instantiate<std_msgs::UInt8MultiArray>();
    if (cube2 && (msg_flag[14]>=0)) cube_data_pub2_.publish(*cube2);
    boost::shared_ptr<std_msgs::UInt8MultiArray> cube3 = frame_msg[15].instantiate<std_msgs::UInt8MultiArray>();
    if (cube3 && (msg_flag[15]>=0)) cube_data_pub3_.publish(*cube3);
    boost::shared_ptr<std_msgs::UInt8MultiArray> cube4 = frame_msg[16].instantiate<std_msgs::UInt8MultiArray>();
    if (cube4 && (msg_flag[16]>=0)) cube_data_pub4_.publish(*cube4);
  }
  else // float模式：只播放float_data
  {
    boost::shared_ptr<arbe_msgs::wfRawDataMsg> float_data0 = frame_msg[0].instantiate<arbe_msgs::wfRawDataMsg>();
    if (float_data0 && (msg_flag[0]>=0))
    {
      frame_id_label_->setText(QString("-Frame ID: %1").arg(float_data0->frame_id)
        +"  -Timestamp: "+QString(boost::posix_time::to_simple_string(
          float_data0->header.stamp.toBoost()+time_offset).c_str()));
      pointcloud_pub0_.publish(*float_data0);
    }
    boost::shared_ptr<arbe_msgs::wfRawDataMsg> float_data1 = frame_msg[1].instantiate<arbe_msgs::wfRawDataMsg>();
    if (float_data1 && (msg_flag[1]>=0)) pointcloud_pub1_.publish(*float_data1);
    boost::shared_ptr<arbe_msgs::wfRawDataMsg> float_data2 = frame_msg[2].instantiate<arbe_msgs::wfRawDataMsg>();
    if (float_data2 && (msg_flag[2]>=0)) pointcloud_pub2_.publish(*float_data2);
    boost::shared_ptr<arbe_msgs::wfRawDataMsg> float_data3 = frame_msg[3].instantiate<arbe_msgs::wfRawDataMsg>();
    if (float_data3 && (msg_flag[3]>=0)) pointcloud_pub3_.publish(*float_data3);
    boost::shared_ptr<arbe_msgs::wfRawDataMsg> float_data4 = frame_msg[4].instantiate<arbe_msgs::wfRawDataMsg>();
    if (float_data4 && (msg_flag[4]>=0)) pointcloud_pub4_.publish(*float_data4);
  }

  //—————Camera_Data—————//
  auto HandleCameraData = [&](int frame_idx, int flag_idx, ros::Publisher& pub) 
  {
    auto data = frame_msg[frame_idx].instantiate<sensor_msgs::CompressedImage>();
    if (data && (msg_flag[flag_idx] >= 0)) 
        pub.publish(*data);
  };
  HandleCameraData(5, 5, camera_pub0_);
  HandleCameraData(6, 6, camera_pub1_);
  HandleCameraData(7, 7, camera_pub2_);
  HandleCameraData(8, 8, camera_pub3_);
  HandleCameraData(9, 9, camera_pub4_);
  HandleCameraData(10, 10, camera_pub5_);
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
    play_sp_data_->setEnabled(false);
    select_main_radar_->setEnabled(false);
    current_bag_label_->setText("Current Bag: " + QString::fromStdString(path));

    bag_reader_->readBagFile(path, 
                             frame_count0, frame_sp_count0,
                             frame_count1, frame_sp_count1, 
                             frame_count2, frame_sp_count2,
                             frame_count3, frame_sp_count3, 
                             frame_count4, frame_sp_count4);
    
    frame_count_label_->setText("Frame Count: Radar(0) " + QString::number(frame_count0) + 
                               ";Radar(1-LT) " + QString::number(frame_count1) +
                               ";Radar(2-RT) " + QString::number(frame_count2) +
                               ";Radar(3-LB) " + QString::number(frame_count3) + 
                               ";Radar(4-RB) " + QString::number(frame_count4));
                               
    frame_sp_count_label_->setText("Frame Count(SP): Radar(0) " + QString::number(frame_sp_count0) + 
                                  ";Radar(1-LT) " + QString::number(frame_sp_count1) +
                                  ";Radar(2-RT) " + QString::number(frame_sp_count2) +
                                  ";Radar(3-LB) " + QString::number(frame_sp_count3) + 
                                  ";Radar(4-RB) " + QString::number(frame_sp_count4));
   
    int c = 0, sp = 0;
    if (mainRadarIndex_ == 0) { c = frame_count0; sp = frame_sp_count0; }
    else if (mainRadarIndex_ == 1) { c = frame_count1; sp = frame_sp_count1; }
    else if (mainRadarIndex_ == 2) { c = frame_count2; sp = frame_sp_count2; }
    else if (mainRadarIndex_ == 3) { c = frame_count3; sp = frame_sp_count3; }
    else if (mainRadarIndex_ == 4) { c = frame_count4; sp = frame_sp_count4; }
    int maxVal = bPlaySPData_ ? sp : c;
    ROS_INFO("readBagFile-maxVal: %d", maxVal);

    frame_spinner_->setMaximum(maxVal);
    frame_slider_->setMaximum(maxVal);
    

    play_button_->setEnabled(true);
    stop_button_->setEnabled(false);
    frame_spinner_->setEnabled(true);
    step_spinner_->setEnabled(true);
    step_forward_button_->setEnabled(true);
    step_backward_button_->setEnabled(true);
    play_rate_combo_->setEnabled(true);
    frame_slider_->setEnabled(true);
    play_sp_data_->setEnabled(true);
    select_main_radar_->setEnabled(true);

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
  play_sp_data_->setEnabled(false);
  select_main_radar_->setEnabled(false);
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
  play_sp_data_->setEnabled(true);
  select_main_radar_->setEnabled(true);
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

void MyRvizPlugin::setPlaySPFlag(int flag)
{
  bPlaySPData_ = flag;
  bag_reader_->setSPFlag(flag);

  int c = 0, sp = 0;
  if (mainRadarIndex_ == 0) { c = frame_count0; sp = frame_sp_count0; }
  else if (mainRadarIndex_ == 1) { c = frame_count1; sp = frame_sp_count1; }
  else if (mainRadarIndex_ == 2) { c = frame_count2; sp = frame_sp_count2; }
  else if (mainRadarIndex_ == 3) { c = frame_count3; sp = frame_sp_count3; }
  else if (mainRadarIndex_ == 4) { c = frame_count4; sp = frame_sp_count4; }
  int maxVal = bPlaySPData_ ? sp : c;

  frame_spinner_->setMaximum(maxVal);
  frame_slider_->setMaximum(maxVal);
  ROS_INFO("sp is %s, change maxVal: %d", bPlaySPData_ ? "true" : "false", maxVal);

  if(frame_spinner_->value() >= frame_spinner_->maximum())
  {
    frame_spinner_->setValue(0);
    frame_slider_->setValue(0);
  }
}

void MyRvizPlugin::selectMainRadar()
{
  mainRadarIndex_ = select_main_radar_->currentIndex();

  bag_reader_->selectMainRadar(mainRadarIndex_);

  int c = 0, sp = 0;
  if (mainRadarIndex_ == 0) { c = frame_count0; sp = frame_sp_count0; }
  else if (mainRadarIndex_ == 1) { c = frame_count1; sp = frame_sp_count1; }
  else if (mainRadarIndex_ == 2) { c = frame_count2; sp = frame_sp_count2; }
  else if (mainRadarIndex_ == 3) { c = frame_count3; sp = frame_sp_count3; }
  else if (mainRadarIndex_ == 4) { c = frame_count4; sp = frame_sp_count4; }
  int maxVal = bPlaySPData_ ? sp : c;

  frame_spinner_->setMaximum(maxVal);
  frame_slider_->setMaximum(maxVal);
  ROS_INFO("change maxVal: %d", maxVal);
  ROS_INFO("change frame_spinner_->value(): %d", frame_spinner_->value());
  ROS_INFO("change frame_spinner_->maximum(): %d", frame_spinner_->maximum());

  if(frame_spinner_->value() >= frame_spinner_->maximum())
  {
    frame_spinner_->setValue(0);
    frame_slider_->setValue(0);
  }
}
void MyRvizPlugin::updateSliderAndSpinner()
{
  frame_slider_->setValue(bag_reader_->getCurrentFrame());
}

} // namespace my_rviz_plugin

PLUGINLIB_EXPORT_CLASS(my_rviz_plugin::MyRvizPlugin, rviz::Panel)
