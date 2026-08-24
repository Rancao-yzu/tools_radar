#include <pluginlib/class_list_macros.h>
#include <QHBoxLayout>
#include <QVBoxLayout>
#include <QFileDialog>
#include <QMainWindow>
#include "my_rviz_plugin/my_rviz_plugin.h"
#include <boost/date_time/posix_time/posix_time.hpp>


namespace my_rviz_plugin
{// 构造函数：初始化UI组件、ROS publisher、subscriber、服务和插件核心对象 BagReader
MyRvizPlugin::MyRvizPlugin(QWidget* parent)
  : rviz::Panel(parent), bag_reader_(new BagReader()), 
  frame_count0(0), frame_sp_count0(0), 
  frame_count1(0), frame_sp_count1(0), frame_count2(0), frame_sp_count2(0), 
  frame_count3(0), frame_sp_count3(0), frame_count4(0), frame_sp_count4(0),
  mainRadarIndex_(0)
{
  nh_ = ros::NodeHandle();

  pointcloud_pub0_ = nh_.advertise<arbe_msgs::wfRawDataMsg>("/wf/corner_radar/parsed/float_data_0", 10);

  pointcloud_sp_pub0_ = nh_.advertise<arbe_msgs::wfTiFrameRD>("/wf/frame_rd_data/ti/radar_0", 10);
  pointcloud_sp_pub1_ = nh_.advertise<arbe_msgs::wfTiFrameRD>("/wf/frame_rd_data/ti/radar_1", 10);
  pointcloud_sp_pub2_ = nh_.advertise<arbe_msgs::wfTiFrameRD>("/wf/frame_rd_data/ti/radar_2", 10);
  pointcloud_sp_pub3_ = nh_.advertise<arbe_msgs::wfTiFrameRD>("/wf/frame_rd_data/ti/radar_3", 10);
  pointcloud_sp_pub4_ = nh_.advertise<arbe_msgs::wfTiFrameRD>("/wf/frame_rd_data/ti/radar_4", 10);
  pointcloud_sp_pub5_ = nh_.advertise<arbe_msgs::wfTiFrameRD>("/wf/frame_rd_data/ti/radar_5", 10);

  camera_pub0_ = nh_.advertise<sensor_msgs::CompressedImage>("/cv_camera_0/image_raw/compressed", 10);  
  camera_pub1_ = nh_.advertise<sensor_msgs::CompressedImage>("/cv_camera_1/image_raw/compressed", 10);
  camera_pub2_ = nh_.advertise<sensor_msgs::CompressedImage>("/cv_camera_2/image_raw/compressed", 10);
  camera_pub3_ = nh_.advertise<sensor_msgs::CompressedImage>("/cv_camera_3/image_raw/compressed", 10);
  camera_pub4_ = nh_.advertise<sensor_msgs::CompressedImage>("/cv_camera_4/image_raw/compressed", 10);
  camera_pub5_ = nh_.advertise<sensor_msgs::CompressedImage>("/cv_camera_5/image_raw/compressed", 10);

  pointcloud_sgu_msgs1_=nh_.advertise<arbe_msgs::wfSguRawData>("/wf/corner_radar/sgu_data_1", 10);
  pointcloud_sgu_msgs2_=nh_.advertise<arbe_msgs::wfSguRawData>("/wf/corner_radar/sgu_data_2", 10);
  pointcloud_sgu_msgs3_=nh_.advertise<arbe_msgs::wfSguRawData>("/wf/corner_radar/sgu_data_3", 10);
  pointcloud_sgu_msgs4_=nh_.advertise<arbe_msgs::wfSguRawData>("/wf/corner_radar/sgu_data_4", 10);

  pointcloud_gt_msgs3_ = nh_.advertise<arbe_msgs::wfSguRawData>("gt/corner_radar/sgu_data_3", 10);
  pointcloud_gt_msgs4_ = nh_.advertise<arbe_msgs::wfSguRawData>("gt/corner_radar/sgu_data_4", 10);

  IMU_msgs_pub = nh_.advertise<arbe_msgs::ImuOutput>("/wf/imu_data/parsed", 10);


  sgu_adas_input_msgs1_=nh_.advertise<arbe_msgs::wfAdasInput>("/wf/corner_radar/adas_input_data_1", 10);
  sgu_adas_input_msgs2_=nh_.advertise<arbe_msgs::wfAdasInput>("/wf/corner_radar/adas_input_data_2", 10);
  sgu_adas_input_msgs3_=nh_.advertise<arbe_msgs::wfAdasInput>("/wf/corner_radar/adas_input_data_3", 10);
  sgu_adas_input_msgs4_=nh_.advertise<arbe_msgs::wfAdasInput>("/wf/corner_radar/adas_input_data_4", 10);

  corner_radar_warning_status_pub_ = nh_.advertise<std_msgs::UInt8MultiArray>("/corner_radar/warning_status", 10);

  car_pub_ = nh_.advertise<arbe_msgs::VehStatusOutput>("/wf/car_id6/parsed2", 1);

  spinner_ = new ros::AsyncSpinner(1);    // 创建异步 spinner，并指定使用 1 个线程
  spinner_->start();

  // 创建服务并注册回调函数(步进阻塞模式:外部算法处理完一个区间后调用,确认OK并推进下一区间)
  service0_ = nh_.advertiseService("/play_single_frame_0", &MyRvizPlugin::handleServiceRequest, this);
  service1_ = nh_.advertiseService("/play_single_frame_1", &MyRvizPlugin::handleServiceRequest, this);
  service2_ = nh_.advertiseService("/play_single_frame_2", &MyRvizPlugin::handleServiceRequest, this);
  service3_ = nh_.advertiseService("/play_single_frame_3", &MyRvizPlugin::handleServiceRequest, this);
  service4_ = nh_.advertiseService("/play_single_frame_4", &MyRvizPlugin::handleServiceRequest, this);

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
  play_sp_date_ = new QCheckBox("SP");
  select_main_radar_ = new QComboBox;
  frame_slider_ = new QSlider(Qt::Horizontal);
  progress_bar_ = new QProgressBar();


  select_button_->setFixedSize(50, 30);
  read_button_->setFixedSize(50, 30);
  play_button_->setFixedSize(40, 30);
  stop_button_->setFixedSize(40, 30);
  step_forward_button_->setFixedSize(50, 30);
  step_backward_button_->setFixedSize(50, 30);

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
  control_layout->addWidget(play_sp_date_);
  control_layout->addWidget(select_main_radar_);

  layout->addLayout(file_layout);
  //layout->addWidget(frame_count_label_);
  //layout->addWidget(frame_sp_count_label_);
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
  connect(play_sp_date_, SIGNAL(stateChanged(int)), this, SLOT(setPlaySPFlag(int)));
  connect(select_main_radar_, SIGNAL(currentIndexChanged(int)), this, SLOT(selectMainRadar()));

  play_button_->setEnabled(false);
  stop_button_->setEnabled(false);
  frame_spinner_->setEnabled(false);
  step_spinner_->setEnabled(false);
  step_forward_button_->setEnabled(false);
  step_backward_button_->setEnabled(false);
  play_rate_combo_->setEnabled(false);
  frame_slider_->setEnabled(false);
  play_sp_date_->setEnabled(false);
  select_main_radar_->setEnabled(false);

  bag_reader_->setMessageCallback([this](const BagReader::IntervalMessages& interval_msgs,  
                                         const int& interval_idx
                                        ) {
    publishIntervalMessages(interval_msgs,interval_idx);
    updateSliderAndSpinner();
  });//设置回调：当BagReader有新区间消息时，发布ROS消息并更新UI

  bag_reader_->setUpdateProgressBarCallback([this](float value){
    progress_bar_->setValue(value);
  });// 设置进度条更新回调

  bContinuePlayFlag = false;
  bSPFlag = false;

  select_main_radar_->setCurrentIndex(3); // 默认选择“后左角雷达(3)”
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
    if(mainRadarIndex_ == req.radar_pos)// 仅处理与当前主雷达位匹配的请求
    {
      if(req.status == 0)
        ROS_INFO("Received data: radar_pos = %d, frame_id = %d ,req.status=%d", req.radar_pos, req.frame_id,req.status);
      else if(req.status == 1){
        bag_reader_->setFinishProcessFlag(true);
        ROS_INFO("--------Received data: radar_pos = %d, frame_id = %d,req.status=%d", req.radar_pos, req.frame_id,req.status);
      }else{
        ROS_INFO("Play single frame service error!");
      }
      res.success = true;
    }
    return true;  // 表示服务成功响应
}

void MyRvizPlugin::publishIntervalMessages(const BagReader::IntervalMessages& interval_msgs,
                                          const int& interval_idx
                                          )
{ // 发布当前时间区间内所有传感器消息到ROS网络(区间内每个topic的所有msg全部发布)
  boost::posix_time::time_duration time_offset(8, 0, 0);// 设置时区偏移

  //—————车辆信息(总发,不分SP)—————//
  for (const auto& m : interval_msgs[12])
  {
    auto car_status = m.instantiate<arbe_msgs::VehStatusOutput>();
    if (car_status) car_pub_.publish(*car_status);
  }

  if(!bSPFlag)
  {
    //—————sgu信息—————//
    for (const auto& m : interval_msgs[0])
    {
      auto d = m.instantiate<arbe_msgs::wfRawDataMsg>();
      if (d) pointcloud_pub0_.publish(*d);
    }

    auto pubSgu = [&](int idx, ros::Publisher& pub, const std::string& err)
    {
      for (const auto& m : interval_msgs[idx])
      {
        auto d = m.instantiate<arbe_msgs::wfSguRawData>();
        if (d) pub.publish(*d);
        else ROS_INFO("%s", err.c_str());
      }
    };
    pubSgu(1, pointcloud_sgu_msgs1_, "pointcloud_sgu_data1 is null");
    pubSgu(2, pointcloud_sgu_msgs2_, "pointcloud_sgu_data2 is null");
    pubSgu(3, pointcloud_sgu_msgs3_, "pointcloud_sgu_data3 is null");
    pubSgu(4, pointcloud_sgu_msgs4_, "pointcloud_sgu_data4 is null");

    //—————gt信号—————//
    pubSgu(17, pointcloud_gt_msgs3_, "pointcloud_gt_data3 is null");
    pubSgu(18, pointcloud_gt_msgs4_, "pointcloud_gt_data4 is null");

    //—————warning_status信号—————//
    for (const auto& m : interval_msgs[5])
    {
      auto d = m.instantiate<std_msgs::UInt8MultiArray>();
      if (d) corner_radar_warning_status_pub_.publish(*d);
    }

    //—————adas_sgu_使能信号—————//
    auto pubAdas = [&](int idx, ros::Publisher& pub)
    {
      for (const auto& m : interval_msgs[idx])
      {
        auto d = m.instantiate<arbe_msgs::wfAdasInput>();
        if (d) pub.publish(*d);
      }
    };
    pubAdas(13, sgu_adas_input_msgs1_);
    pubAdas(14, sgu_adas_input_msgs2_);
    pubAdas(15, sgu_adas_input_msgs3_);
    pubAdas(16, sgu_adas_input_msgs4_);

    // 用区间内第一个sgu3更新frame_id显示(与原逻辑一致)
    if (!interval_msgs[3].empty())
    {
      auto d3 = interval_msgs[3][0].instantiate<arbe_msgs::wfSguRawData>();
      if (d3)
      {
        uint16_t custom_frame_id = d3->frameID;
        ros::Time time_ = d3->header.stamp;
        boost::posix_time::ptime boost_time = time_.toBoost();
        boost_time += time_offset;
        std::string time_str = boost::posix_time::to_simple_string(boost_time);

        ROS_INFO("Pointcloud custom frame_id: %d", custom_frame_id);
        frame_id_label_->setText(QString("-Frame ID: %1").arg(custom_frame_id)+"  -Timestamp: "+QString(time_str.c_str()));
      }
    }
  }
  else
  {
    //—————SP信息—————//
    auto pubSp = [&](int idx, ros::Publisher& pub, const std::string& err)
    {
      for (const auto& m : interval_msgs[idx])
      {
        auto d = m.instantiate<arbe_msgs::wfTiFrameRD>();
        if (d) pub.publish(*d);
        else ROS_INFO("%s", err.c_str());
      }
    };
    pubSp(0, pointcloud_sp_pub0_, "pointcloud_data0 (SP) is null");
    pubSp(1, pointcloud_sp_pub1_, "pointcloud_data1 (SP) is null");
    pubSp(2, pointcloud_sp_pub2_, "pointcloud_data2 (SP) is null");
    pubSp(3, pointcloud_sp_pub3_, "pointcloud_data3 (SP) is null");
    pubSp(4, pointcloud_sp_pub4_, "pointcloud_data4 (SP) is null");

    // 用区间内第一个sp3更新frame_id显示(与原逻辑一致)
    if (!interval_msgs[3].empty())
    {
      auto d3 = interval_msgs[3][0].instantiate<arbe_msgs::wfTiFrameRD>();
      if (d3)
      {
        uint16_t custom_frame_id = d3->frameID;
        ros::Time time_ = d3->header.stamp;
        boost::posix_time::ptime boost_time = time_.toBoost();
        boost_time += time_offset;
        std::string time_str = boost::posix_time::to_simple_string(boost_time);

        ROS_INFO("Pointcloud custom frame_id: %d", custom_frame_id);
        frame_id_label_->setText(QString("-Frame ID: %1").arg(custom_frame_id)+"  -Timestamp: "+QString(time_str.c_str()));
      }
    }
  }

  //—————IMU(总发,不分SP)—————//
  for (const auto& m : interval_msgs[19])
  {
    auto imu_data = m.instantiate<arbe_msgs::ImuOutput>();
    if (imu_data) IMU_msgs_pub.publish(*imu_data);
    else ROS_INFO("IMU_msgs_pub is null");
  }

  //—————Camera_Data(总发,不分SP)—————//
  auto pubCam = [&](int idx, ros::Publisher& pub)
  {
    for (const auto& m : interval_msgs[idx])
    {
      auto data = m.instantiate<sensor_msgs::CompressedImage>();
      if (data) pub.publish(*data);
    }
  };
  pubCam(6,  camera_pub0_);
  pubCam(7,  camera_pub1_);
  pubCam(8,  camera_pub2_);
  pubCam(9,  camera_pub3_);
  pubCam(10, camera_pub4_);
  pubCam(11, camera_pub5_);
}

void MyRvizPlugin::selectBagFile()
{
  QString file = QFileDialog::getOpenFileName(this, "Select Bag File", "", "Bag Files (*.bag)");
  if (!file.isEmpty())//文件路径不为空
    bag_file_path_->setText(file);
}

void MyRvizPlugin::recreateBagReader()
{
  delete bag_reader_;  // 删除旧的bag_reader_
  bag_reader_ = new BagReader();  // 创建新的bag_reader_
  
  bag_reader_->setMessageCallback([this](const BagReader::IntervalMessages& interval_msgs,  
                                         const int& interval_idx
                                        ) {
    publishIntervalMessages(interval_msgs,interval_idx);
    updateSliderAndSpinner();

  });

  bag_reader_->setUpdateProgressBarCallback([this](float value){
    progress_bar_->setValue(value);
  });
  
}

void MyRvizPlugin::readBagFile()// 2025/9/17
{
  static std::string last_path = "";  // 保存上一次的路径
  std::string path = bag_file_path_->text().toStdString();

  // 只有当路径非空且路径发生变化时才继续处理
  if (!path.empty() && static_cast<void*>(const_cast<char*>(last_path.c_str())) != static_cast<void*>(const_cast<char*>(path.c_str())))
  {
    recreateBagReader();
    play_button_->setEnabled(false);
    stop_button_->setEnabled(false);
    frame_spinner_->setEnabled(false);
    step_spinner_->setEnabled(false);
    step_forward_button_->setEnabled(false);
    step_backward_button_->setEnabled(false);
    play_rate_combo_->setEnabled(false);
    frame_slider_->setEnabled(false);
    play_sp_date_->setEnabled(false);
    select_main_radar_->setEnabled(false);

    
    bag_reader_->readBagFile(path, frame_count0, frame_sp_count0,frame_count1, frame_sp_count1, frame_count2, frame_sp_count2,
      frame_count3, frame_sp_count3, frame_count4, frame_sp_count4);
    frame_count_label_->setText("Frame Count: Radar(0) " + QString::number(frame_count0) + ";Radar(1-LT) " + QString::number(frame_count1) +
      ";Radar(2-RT) " + QString::number(frame_count2) +";Radar(3-LB) " + QString::number(frame_count3) + ";Radar(4-RB) " + QString::number(frame_count4));
    frame_sp_count_label_->setText("Frame Count(SP): Radar(0) " + QString::number(frame_sp_count0) + ";Radar(1-LT) " + QString::number(frame_sp_count1) +
      ";Radar(2-RT) " + QString::number(frame_sp_count2) +";Radar(3-LB) " + QString::number(frame_sp_count3) + ";Radar(4-RB) " + QString::number(frame_sp_count4));
   
    //  bag_reader_->jumpToFrame(0);  // 重置当前帧为 0;取消注释这行代码开头id可能会重复

  // 区间数:由主雷达首帧时间T0按15ms切片得到(切换主雷达/SP后会重算)
  int maxVal = bag_reader_->getIntervalCount();
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
    play_sp_date_->setEnabled(true);
    select_main_radar_->setEnabled(true);

    ROS_INFO("———Bag file read and cached success———: %s", path.c_str());

    last_path = path;  
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
  play_sp_date_->setEnabled(false);
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
  play_sp_date_->setEnabled(true);
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
  bSPFlag = flag;
  bag_reader_->setSPFlag(flag);

  // 区间数:由主雷达首帧时间T0按15ms切片得到(切换主雷达/SP后会重算)
  int maxVal = bag_reader_->getIntervalCount();
  frame_spinner_->setMaximum(maxVal);
  frame_slider_->setMaximum(maxVal);

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

  // 区间数:由主雷达首帧时间T0按15ms切片得到(切换主雷达/SP后会重算)
  int maxVal = bag_reader_->getIntervalCount();
  frame_spinner_->setMaximum(maxVal);
  frame_slider_->setMaximum(maxVal);

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
