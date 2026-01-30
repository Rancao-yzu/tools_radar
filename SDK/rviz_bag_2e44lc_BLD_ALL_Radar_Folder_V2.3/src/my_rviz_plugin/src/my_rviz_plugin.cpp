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
  frame_count0(0), frame_sp_count0(0), frame_rd0(0),
  frame_count1(0), frame_sp_count1(0), frame_rd1(0), frame_count2(0), frame_sp_count2(0), frame_rd2(0),
  frame_count3(0), frame_sp_count3(0), frame_rd3(0), frame_count4(0), frame_sp_count4(0), frame_rd4(0),
  current_bag_index_(-1), folder_mode_(false)
{
  nh_ = ros::NodeHandle();

  pointcloud_pub0_ = nh_.advertise<arbe_msgs::wfRawDataMsg>("/wf/corner_radar/parsed/float_data_0", 10);
  pointcloud_pub1_ = nh_.advertise<arbe_msgs::wfRawDataMsg>("/wf/corner_radar/parsed/float_data_1", 10);
  pointcloud_pub2_ = nh_.advertise<arbe_msgs::wfRawDataMsg>("/wf/corner_radar/parsed/float_data_2", 10);
  pointcloud_pub3_ = nh_.advertise<arbe_msgs::wfRawDataMsg>("/wf/corner_radar/parsed/float_data_3", 10);
  pointcloud_pub4_ = nh_.advertise<arbe_msgs::wfRawDataMsg>("/wf/corner_radar/parsed/float_data_4", 10);
  pointcloud_pub5_ = nh_.advertise<arbe_msgs::wfRawDataMsg>("/wf/corner_radar/parsed/float_data_5", 10);

  pointcloud_sp_pub0_ = nh_.advertise<arbe_msgs::wfTiFrameRD>("/wf/frame_rd_data/ti/radar_0", 10);
  pointcloud_sp_pub1_ = nh_.advertise<arbe_msgs::wfTiFrameRD>("/wf/frame_rd_data/ti/radar_1", 10);
  pointcloud_sp_pub2_ = nh_.advertise<arbe_msgs::wfTiFrameRD>("/wf/frame_rd_data/ti/radar_2", 10);
  pointcloud_sp_pub3_ = nh_.advertise<arbe_msgs::wfTiFrameRD>("/wf/frame_rd_data/ti/radar_3", 10);
  pointcloud_sp_pub4_ = nh_.advertise<arbe_msgs::wfTiFrameRD>("/wf/frame_rd_data/ti/radar_4", 10);
  pointcloud_sp_pub5_ = nh_.advertise<arbe_msgs::wfTiFrameRD>("/wf/frame_rd_data/ti/radar_5", 10);

  pointcloud_rd_pub0_ = nh_.advertise<arbe_msgs::wfTiRDdata>("/wf/corner_radar/rd_data_0", 10);
  pointcloud_rd_pub1_ = nh_.advertise<arbe_msgs::wfTiRDdata>("/wf/corner_radar/rd_data_1", 10);
  pointcloud_rd_pub2_ = nh_.advertise<arbe_msgs::wfTiRDdata>("/wf/corner_radar/rd_data_2", 10);
  pointcloud_rd_pub3_ = nh_.advertise<arbe_msgs::wfTiRDdata>("/wf/corner_radar/rd_data_3", 10);
  pointcloud_rd_pub4_ = nh_.advertise<arbe_msgs::wfTiRDdata>("/wf/corner_radar/rd_data_4", 10);
  pointcloud_rd_pub5_ = nh_.advertise<arbe_msgs::wfTiRDdata>("/wf/corner_radar/rd_data_5", 10);

  camera_pub0_ = nh_.advertise<sensor_msgs::CompressedImage>("/cv_camera_0/image_raw/compressed", 10);  
  camera_pub1_ = nh_.advertise<sensor_msgs::CompressedImage>("/cv_camera_1/image_raw/compressed", 10);
  camera_pub2_ = nh_.advertise<sensor_msgs::CompressedImage>("/cv_camera_2/image_raw/compressed", 10);
  camera_pub3_ = nh_.advertise<sensor_msgs::CompressedImage>("/cv_camera_3/image_raw/compressed", 10);
  camera_pub4_ = nh_.advertise<sensor_msgs::CompressedImage>("/cv_camera_4/image_raw/compressed", 10);
  camera_pub5_ = nh_.advertise<sensor_msgs::CompressedImage>("/cv_camera_5/image_raw/compressed", 10);

  
  car_pub_ = nh_.advertise<arbe_msgs::VehStatusOutput>("/wf/car_id6/parsed2", 1);

   // 创建异步 spinner，并指定使用 1 个线程
  spinner_ = new ros::AsyncSpinner(1);  // 可以根据需要指定更多的线程
  spinner_->start();
  
  // 创建一个服务并注册回调函数
  service0_ = nh_.advertiseService("/play_single_frame_0", &MyRvizPlugin::handleServiceRequest, this);
  service1_ = nh_.advertiseService("/play_single_frame_1", &MyRvizPlugin::handleServiceRequest, this);
  service2_ = nh_.advertiseService("/play_single_frame_2", &MyRvizPlugin::handleServiceRequest, this);
  service3_ = nh_.advertiseService("/play_single_frame_3", &MyRvizPlugin::handleServiceRequest, this);
  service4_ = nh_.advertiseService("/play_single_frame_4", &MyRvizPlugin::handleServiceRequest, this);

  
  folder_path_ = new QLineEdit;
  select_folder_button_ = new QPushButton("Folder");
  current_bag_label_ = new QLabel("Current Bag: N/A");
  bag_file_path_ = new QLineEdit;
  select_button_ = new QPushButton("Bag");
  read_button_ = new QPushButton("Read");
  play_button_ = new QPushButton("Play");
  stop_button_ = new QPushButton("Stop");
  step_forward_button_ = new QPushButton("step->");
  step_backward_button_ = new QPushButton("<-step");
  frame_spinner_ = new QSpinBox;
  step_spinner_ = new QSpinBox;
  frame_count_label_ = new QLabel("Frame Count: Radar(1-LT) 0;Radar(2-RT) 0;Radar(3-LB) 0;Radar(4-RB) 0");
  frame_sp_count_label_ = new QLabel("Frame Count(SP): Radar(1-LT) 0;Radar(2-RT) 0;Radar(3-LB) 0;Radar(4-RB) 0");
  frame_rd_count_label_ = new QLabel("Frame Count(RD): Radar(1-LT) 0;Radar(2-RT) 0;Radar(3-LB) 0;Radar(4-RB) 0");
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
  select_folder_button_->setFixedSize(50, 30);


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
  control_layout->addWidget(new QLabel("Step:"));
  control_layout->addWidget(step_spinner_);
  control_layout->addWidget(step_backward_button_);
  control_layout->addWidget(step_forward_button_);
  control_layout->addWidget(play_button_);
  control_layout->addWidget(stop_button_);
  control_layout->addWidget(new QLabel("Play Rate:"));
  control_layout->addWidget(play_rate_combo_);
  control_layout->addWidget(play_sp_date_);
  control_layout->addWidget(select_main_radar_);

  layout->addLayout(file_layout);
  layout->addWidget(folder_path_);
  layout->addWidget(current_bag_label_);
  //layout->addWidget(frame_count_label_);
  //layout->addWidget(frame_sp_count_label_);
  layout->addWidget(frame_rd_count_label_);
  layout->addWidget(frame_id_label_);
    layout->addLayout(layoutnubmber);
  // layout->addWidget(frame_spinner_);
  // layout->addWidget(frame_slider_);
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
  connect(select_folder_button_, SIGNAL(clicked()), this, SLOT(selectFolder()));

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

bag_reader_->setMessageCallback([this](const std::vector<rosbag::MessageInstance>& frame_msg,  
                                       const int& frame_number,
                                       const std::vector<int>& msg_flag) {
  publishClosestMessages(frame_msg, frame_number, msg_flag);
  updateSliderAndSpinner();
  ROS_DEBUG("Frame %d/%d, folder_mode_: %d, current_bag_index_: %d/%lu",
           frame_number, bag_reader_->getMainRadarPclSize(), folder_mode_, current_bag_index_, bag_files_.size());
  if (folder_mode_ && frame_number >= bag_reader_->getMainRadarPclSize() - 1) {
    ROS_INFO("Reached end of bag file %d/%lu: %s, preparing to stop and load next",
             current_bag_index_ + 1, bag_files_.size(), bag_files_[current_bag_index_].c_str());
    // 使用 QMetaObject::invokeMethod 在主线程中执行
    QMetaObject::invokeMethod(this, [this]() {
      ROS_INFO("Stopping current bag playback (index: %d)", current_bag_index_);
      stopBag();
      if (current_bag_index_ < static_cast<int>(bag_files_.size()) - 1) {
        ROS_INFO("Automatically loading next bag file %d/%lu: %s",
                 current_bag_index_ + 2, bag_files_.size(), bag_files_[current_bag_index_ + 1].c_str());
        readBagFile();
        // 延迟调用 playBag，确保 readBagFile 完成
        QTimer::singleShot(1000, this, [this]() {
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
});

  bag_reader_->setUpdateProgressBarCallback([this](float value){
    progress_bar_->setValue(value);
  });
  bContinuePlayFlag = false;
  bSPFlag = false;


  select_main_radar_->setCurrentIndex(3);
}

MyRvizPlugin::~MyRvizPlugin()
{
  delete bag_reader_;

  spinner_->stop();
  delete spinner_;
}

// 服务回调函数：处理客户端请求
bool MyRvizPlugin::handleServiceRequest(wf_srvs::PlaySingleFrame::Request &req,
                            wf_srvs::PlaySingleFrame::Response &res)
{
    // 显示接收到的请求数据
    if(mainRadarIndex_ == req.radar_pos)
    {
      if(req.status == 0)
      {
        ROS_INFO("Received data: radar_pos = %d, frame_id = %d", req.radar_pos, req.frame_id);
      }
      else if(req.status == 1)
      {
        ROS_INFO("Finish process: radar_pos = %d, frame_id = %d", req.radar_pos, req.frame_id);

        bag_reader_->setFinishProcessFlag(true);
      }
      else
      {
        ROS_INFO("Play single frame service error!");
      }

      res.success = true;
    }
    
    return true;  // 表示服务成功响应
}

void MyRvizPlugin::publishClosestMessages(const std::vector<rosbag::MessageInstance>& frame_msg,  
                                          const int& frame_number,
                                          const std::vector<int>& msg_flag
                                          )
{
  boost::shared_ptr<arbe_msgs::VehStatusOutput> car_status = frame_msg[12].instantiate<arbe_msgs::VehStatusOutput>();
  if (car_status&&(msg_flag[12]>=0))
  {
    car_pub_.publish(*car_status);
  }
  boost::posix_time::time_duration time_offset(8, 0, 0);

     boost::shared_ptr<arbe_msgs::wfTiRDdata> pointcloud_rd_data0 = frame_msg[18].instantiate<arbe_msgs::wfTiRDdata>();
  if (pointcloud_rd_data0&&(msg_flag[18]>=0))
  {
    pointcloud_rd_pub0_.publish(*pointcloud_rd_data0);
  }
  else
  {
    ROS_INFO("pointcloud_rd_data0 is null");
  }

  boost::shared_ptr<arbe_msgs::wfTiRDdata> pointcloud_rd_data1 = frame_msg[19].instantiate<arbe_msgs::wfTiRDdata>();
  if (pointcloud_rd_data1&&(msg_flag[19]>=0))
  {
    pointcloud_rd_pub1_.publish(*pointcloud_rd_data1);
  }
  else
  {
    ROS_INFO("pointcloud_rd_data1 is null");
  }

  boost::shared_ptr<arbe_msgs::wfTiRDdata> pointcloud_rd_data2 = frame_msg[20].instantiate<arbe_msgs::wfTiRDdata>();
  if (pointcloud_rd_data2&&(msg_flag[20]>=0))
  {
    pointcloud_rd_pub2_.publish(*pointcloud_rd_data2);
  }
  else
  {
    ROS_INFO("pointcloud_rd_data2 is null");
  }

  boost::shared_ptr<arbe_msgs::wfTiRDdata> pointcloud_rd_data3 = frame_msg[21].instantiate<arbe_msgs::wfTiRDdata>();
  if (pointcloud_rd_data3&&(msg_flag[21]>=0))
  {
    uint16_t custom_frame_id = pointcloud_rd_data3->frameID;
    ros::Time time_ = pointcloud_rd_data3->header.stamp;
    boost::posix_time::ptime boost_time = time_.toBoost();
    boost_time += time_offset;
    // 格式化为字符串
    std::string time_str = boost::posix_time::to_simple_string(boost_time);
    ROS_INFO("Pointcloud custom frame_id: %d", custom_frame_id);
    frame_id_label_->setText(QString("Frame ID: %1").arg(custom_frame_id)+"  Timestamp: "+QString(time_str.c_str()));
    pointcloud_rd_pub3_.publish(*pointcloud_rd_data3);
  }
  else
  {
    ROS_INFO("pointcloud_rd_data3 is null");
  }

  boost::shared_ptr<arbe_msgs::wfTiRDdata> pointcloud_rd_data4 = frame_msg[22].instantiate<arbe_msgs::wfTiRDdata>();
  if (pointcloud_rd_data4&&(msg_flag[22]>=0))
  {
    pointcloud_rd_pub4_.publish(*pointcloud_rd_data4);
  }
  else
  {
    ROS_INFO("pointcloud_rd_data4 is null");
  }

  boost::shared_ptr<arbe_msgs::wfRawDataMsg> pointcloud_data0 = frame_msg[13].instantiate<arbe_msgs::wfRawDataMsg>();
  if (pointcloud_data0&&(msg_flag[13]>=0))
  {
    pointcloud_pub0_.publish(*pointcloud_data0);
  }
  else
  {
    ROS_INFO("pointcloud_data0 is null");
  }

  boost::shared_ptr<arbe_msgs::wfRawDataMsg> pointcloud_data1 = frame_msg[14].instantiate<arbe_msgs::wfRawDataMsg>();
  if (pointcloud_data1&&(msg_flag[14]>=0))
  {
    pointcloud_pub1_.publish(*pointcloud_data1);
  }
  else
  {
    
    ROS_INFO("pointcloud_data1 is null");
  }

  boost::shared_ptr<arbe_msgs::wfRawDataMsg> pointcloud_data2 = frame_msg[15].instantiate<arbe_msgs::wfRawDataMsg>();
  if (pointcloud_data2&&(msg_flag[15]>=0))
  {
    pointcloud_pub2_.publish(*pointcloud_data2);
  }
  else
  {
    ROS_INFO("pointcloud_data2 is null");
  }

  boost::shared_ptr<arbe_msgs::wfRawDataMsg> pointcloud_data3 = frame_msg[16].instantiate<arbe_msgs::wfRawDataMsg>();
  if (pointcloud_data3&&(msg_flag[16]>=0))
  {
    pointcloud_pub3_.publish(*pointcloud_data3);
  }
  else
  {
    ROS_INFO("pointcloud_data3 is null");
  }

  boost::shared_ptr<arbe_msgs::wfRawDataMsg> pointcloud_data4 = frame_msg[17].instantiate<arbe_msgs::wfRawDataMsg>();
  if (pointcloud_data4&&(msg_flag[17]>=0))
  {
    pointcloud_pub4_.publish(*pointcloud_data4);
  }
  else
  {
    ROS_INFO("pointcloud_data4 is null");
  }


  boost::shared_ptr<arbe_msgs::wfTiFrameRD> pointcloud_sp_data0 = frame_msg[0].instantiate<arbe_msgs::wfTiFrameRD>();
  if (pointcloud_sp_data0&&(msg_flag[0]>=0))
  {
      pointcloud_sp_pub0_.publish(*pointcloud_sp_data0);
  }
  else
  {
      ROS_INFO("pointcloud_sp_data0 is null");
  }

  boost::shared_ptr<arbe_msgs::wfTiFrameRD> pointcloud_sp_data1 = frame_msg[1].instantiate<arbe_msgs::wfTiFrameRD>();
  if (pointcloud_sp_data1&&(msg_flag[1]>=0))
  {
      pointcloud_sp_pub1_.publish(*pointcloud_sp_data1);
  }
  else
  {
      ROS_INFO("pointcloud_sp_data1 is null");
  }

  boost::shared_ptr<arbe_msgs::wfTiFrameRD> pointcloud_sp_data2 = frame_msg[2].instantiate<arbe_msgs::wfTiFrameRD>();
  if (pointcloud_sp_data2&&(msg_flag[2]>=0))
  {
      pointcloud_sp_pub2_.publish(*pointcloud_sp_data2);
  }
  else
  {
      ROS_INFO("pointcloud_sp_data2 is null");
  }

  boost::shared_ptr<arbe_msgs::wfTiFrameRD> pointcloud_sp_data3 = frame_msg[3].instantiate<arbe_msgs::wfTiFrameRD>();
  if (pointcloud_sp_data3&&(msg_flag[3]>=0))
  {
      pointcloud_sp_pub3_.publish(*pointcloud_sp_data3);
  }
  else
  {
      ROS_INFO("pointcloud_sp_data3 is null");
  }

  boost::shared_ptr<arbe_msgs::wfTiFrameRD> pointcloud_sp_data4 = frame_msg[4].instantiate<arbe_msgs::wfTiFrameRD>();
  if (pointcloud_sp_data4&&(msg_flag[4]>=0))
  {
      pointcloud_sp_pub4_.publish(*pointcloud_sp_data4);
  }
  else
  {
      ROS_INFO("pointcloud_sp_data4 is null");
  }


  boost::shared_ptr<sensor_msgs::CompressedImage> camera_data0 = frame_msg[6].instantiate<sensor_msgs::CompressedImage>();
  if (camera_data0&&(msg_flag[6]>=0))
  {
    camera_pub0_.publish(*camera_data0);
  }

  boost::shared_ptr<sensor_msgs::CompressedImage> camera_data1 = frame_msg[7].instantiate<sensor_msgs::CompressedImage>();
  if (camera_data1&&(msg_flag[7]>=0))
  {
    camera_pub1_.publish(*camera_data1);
  }

  boost::shared_ptr<sensor_msgs::CompressedImage> camera_data2 = frame_msg[8].instantiate<sensor_msgs::CompressedImage>();
  if (camera_data2&&(msg_flag[8]>=0))
  {
    camera_pub2_.publish(*camera_data2);
  }

  boost::shared_ptr<sensor_msgs::CompressedImage> camera_data3 = frame_msg[9].instantiate<sensor_msgs::CompressedImage>();
  if (camera_data3&&(msg_flag[9]>=0))
  {
    camera_pub3_.publish(*camera_data3);
  }

  boost::shared_ptr<sensor_msgs::CompressedImage> camera_data4 = frame_msg[10].instantiate<sensor_msgs::CompressedImage>();
  if (camera_data4&&(msg_flag[10]>=0))
  {
    camera_pub4_.publish(*camera_data4);
  }
  boost::shared_ptr<sensor_msgs::CompressedImage> camera_data5 = frame_msg[11].instantiate<sensor_msgs::CompressedImage>();
  if (camera_data5&&(msg_flag[11]>=0))
  {
    camera_pub5_.publish(*camera_data5);
  }
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

    QDir dir(folder);
    QStringList filters;
    filters << "*.bag";
    dir.setNameFilters(filters);
    QStringList bag_list = dir.entryList(QDir::Files | QDir::NoDotAndDotDot, QDir::Name);

    for (const QString& bag_file : bag_list)
    {
      bag_files_.push_back(dir.filePath(bag_file).toStdString());
    }

    if (!bag_files_.empty())
    {
      bag_file_path_->setText(QString::fromStdString(bag_files_[0]));
      current_bag_label_->setText("Current Bag: N/A");
      ROS_INFO("Found %lu bag files in folder: %s", bag_files_.size(), folder.toStdString().c_str());
    }
    else
    {
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
  if (folder_mode_)
  {
    if (current_bag_index_ >= static_cast<int>(bag_files_.size()) - 1)
    {
      ROS_INFO("No more bag files to load in folder");
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
  else
  {
    path = bag_file_path_->text().toStdString();
  }

  if (!path.empty())
  {

    ROS_INFO("Attempting to read bag file: %s", path.c_str());
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
    current_bag_label_->setText("Current Bag: " + QString::fromStdString(path));

    try
    {
      bag_reader_->readBagFile(path, frame_count0, frame_sp_count0, frame_rd0, frame_count1, frame_sp_count1, frame_rd1,
                               frame_count2, frame_sp_count2, frame_rd2, frame_count3, frame_sp_count3, frame_rd3,
                               frame_count4, frame_sp_count4, frame_rd4);
    }
    catch (const rosbag::BagException& e)
    {
      ROS_ERROR("Failed to read bag file %s: %s", path.c_str(), e.what());
      current_bag_label_->setText("Current Bag: Failed to load");
      return;
    }

    frame_count_label_->setText("Frame Count: Radar(0) " + QString::number(frame_count0) + ";Radar(1-LT) " + QString::number(frame_count1) +
                               ";Radar(2-RT) " + QString::number(frame_count2) + ";Radar(3-LB) " + QString::number(frame_count3) +
                               ";Radar(4-RB) " + QString::number(frame_count4));
    frame_sp_count_label_->setText("Frame Count(SP): Radar(0) " + QString::number(frame_sp_count0) + ";Radar(1-LT) " + QString::number(frame_sp_count1) +
                                  ";Radar(2-RT) " + QString::number(frame_sp_count2) + ";Radar(3-LB) " + QString::number(frame_sp_count3) +
                                  ";Radar(4-RB) " + QString::number(frame_sp_count4));
    frame_rd_count_label_->setText("Frame Count(RD): Radar(0) " + QString::number(frame_rd0) + ";Radar(1-LT) " + QString::number(frame_rd1) +
                                  ";Radar(2-RT) " + QString::number(frame_rd2) + ";Radar(3-LB) " + QString::number(frame_rd3) +
                                  ";Radar(4-RB) " + QString::number(frame_rd4));

    frame_spinner_->setValue(0);
    frame_slider_->setValue(0);


    if(mainRadarIndex_ == 0)
    {
        frame_spinner_->setMaximum(frame_rd0);
        frame_slider_->setMaximum(frame_rd0);
    }
    else if(mainRadarIndex_ == 1)
    {
        frame_spinner_->setMaximum(frame_rd1);
        frame_slider_->setMaximum(frame_rd1);
    }
    else if(mainRadarIndex_ == 2)
    {
        frame_spinner_->setMaximum(frame_rd2);
        frame_slider_->setMaximum(frame_rd2);
    }
    else if(mainRadarIndex_ == 3)
    {
        frame_spinner_->setMaximum(frame_rd3);
        frame_slider_->setMaximum(frame_rd3);
    }
    else if(mainRadarIndex_ == 4)
    {
        frame_spinner_->setMaximum(frame_rd4);
        frame_slider_->setMaximum(frame_rd4);
    }
    
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

    ROS_INFO("Bag file read and cached success: %s", path.c_str());
  }
  else
  {
    ROS_WARN("No bag file path provided");
  }
}

void MyRvizPlugin::jumpToFrame()
{
  if(!bContinuePlayFlag)
  {
    int frame_number = frame_spinner_->value();
    if (frame_number >= 0 )
    {
      bag_reader_->jumpToFrame(frame_number);
    }
  }
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

  ROS_INFO("Playback stopped, waiting for manual read");
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


void MyRvizPlugin::updatePlayRate()
{
  double rate = play_rate_combo_->currentText().toDouble();
  bag_reader_->setPlayRate(rate);
}

void MyRvizPlugin::sliderValueChanged(int value)
{
  if(frame_spinner_->value() >= frame_spinner_->maximum() || value >= frame_spinner_->maximum())
  {
    frame_spinner_->setValue(0);
    frame_slider_->setValue(0);
  }
  else
  {
    frame_spinner_->setValue(value);
  }
}

void MyRvizPlugin::setPlaySPFlag(int flag)
{
  bSPFlag = flag;
  bag_reader_->setSPFlag(flag);


  if(mainRadarIndex_ == 0)
  {
      frame_spinner_->setMaximum(frame_rd0);
      frame_slider_->setMaximum(frame_rd0);
  }

  if(mainRadarIndex_ == 1)
  {
      frame_spinner_->setMaximum(frame_rd1);
      frame_slider_->setMaximum(frame_rd1);
  }
  else if(mainRadarIndex_ == 2)
  {
      frame_spinner_->setMaximum(frame_rd2);
      frame_slider_->setMaximum(frame_rd2);
  }
  else if(mainRadarIndex_ == 3)
  {
      frame_spinner_->setMaximum(frame_rd3);
      frame_slider_->setMaximum(frame_rd3);
  }
  else if(mainRadarIndex_ == 4)
  {
      frame_spinner_->setMaximum(frame_rd4);
      frame_slider_->setMaximum(frame_rd4);
  }

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

  if(mainRadarIndex_ == 0)
  {
      frame_spinner_->setMaximum(frame_rd0);
      frame_slider_->setMaximum(frame_rd0);
  }
  else if(mainRadarIndex_ == 1)
  {
      frame_spinner_->setMaximum(frame_rd1);
      frame_slider_->setMaximum(frame_rd1);
  }
  else if(mainRadarIndex_ == 2)
  {
      frame_spinner_->setMaximum(frame_rd2);
      frame_slider_->setMaximum(frame_rd2);
  }
  else if(mainRadarIndex_ == 3)
  {
      frame_spinner_->setMaximum(frame_rd3);
      frame_slider_->setMaximum(frame_rd3);
  }
  else if(mainRadarIndex_ == 4)
  {
      frame_spinner_->setMaximum(frame_rd4);
      frame_slider_->setMaximum(frame_rd4);
  }
  else
  {
    frame_spinner_->setMaximum(0);
    frame_slider_->setMaximum(0);
  }

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
