#include "my_rviz_plugin/bag_reader.h"
#include <rosbag/view.h>
#include <chrono>
#include <thread>

namespace my_rviz_plugin
{
BagReader::BagReader() 
  : current_frame_(0),           // 当前播放的帧索引
    play_rate_(1.0),             // 播放速率（倍速）
    playing_(false),             // 是否正在播放
    finishProcessFlag_(true),    // 是否完成处理的标志
    bPlaySPFlag_(false),         // 是否播放特殊点云的标志（未明确使用）
    mainRadarIndex_(3)           // 主雷达索引，默认为3
{  }

BagReader::~BagReader()
{
  if (playing_)
  {
    playing_ = false;
    if (play_thread_.joinable())
      play_thread_.join();
  }
  bag_.close();
}

// 读取bag文件，并按话题分类存储消息
void BagReader::readBagFile(
    const std::string& file_path, 
    int& frameCount0, int& frameSPCount0, int& frameRDCount0,
    int& frameCount1, int& frameSPCount1, int& frameRDCount1,
    int& frameCount2, int& frameSPCount2, int& frameRDCount2,
    int& frameCount3, int& frameSPCount3, int& frameRDCount3,
    int& frameCount4, int& frameSPCount4, int& frameRDCount4)
{

  if(update_progress_bar_callback_)// 更新进度条：开始读取bag文件
  {
    update_progress_bar_callback_(0.05f);
  }

  stopBag(); // 停止当前播放或加载

  if (bag_.isOpen()) {
    ROS_INFO("Closing previous bag file.");
    bag_.close();
  }

  if(update_progress_bar_callback_)// 更新进度条
  {
    update_progress_bar_callback_(0.1f);
  }

  // 清空所有存储的消息容器
  car_msgs_.clear();
  camera_msgs0_.clear(); camera_msgs1_.clear(); camera_msgs2_.clear(); 
  camera_msgs3_.clear(); camera_msgs4_.clear(); camera_msgs5_.clear();
  pointcloud_msgs0_.clear(); pointcloud_msgs1_.clear(); pointcloud_msgs2_.clear();
  pointcloud_msgs3_.clear(); pointcloud_msgs4_.clear();
  pointcloud_rd_msgs0_.clear(); pointcloud_rd_msgs1_.clear(); pointcloud_rd_msgs2_.clear();
  pointcloud_rd_msgs3_.clear(); pointcloud_rd_msgs4_.clear();
  pointcloud_sp_msgs0_.clear(); pointcloud_sp_msgs1_.clear(); pointcloud_sp_msgs2_.clear();
  pointcloud_sp_msgs3_.clear(); pointcloud_sp_msgs4_.clear();
  empty_msgs_.clear();
  msg_flags_.clear();

  bag_.open(file_path, rosbag::bagmode::Read);

  if(update_progress_bar_callback_)
  {
    update_progress_bar_callback_(0.15f);
  }

  // 只读取指定的话题
  std::vector<std::string> topics = {"/wf/corner_radar/parsed/float_data_0","/wf/corner_radar/parsed/float_data_1", "/wf/corner_radar/parsed/float_data_2", 
    "/wf/corner_radar/parsed/float_data_3", "/wf/corner_radar/parsed/float_data_4", 
    "/wf/frame_rd_data/ti/radar_0","/wf/frame_rd_data/ti/radar_1", "/wf/frame_rd_data/ti/radar_2",
    "/wf/frame_rd_data/ti/radar_3", "/wf/frame_rd_data/ti/radar_4",
    "/wf/corner_radar/rd_data_0","/wf/corner_radar/rd_data_1", "/wf/corner_radar/rd_data_2",
    "/wf/corner_radar/rd_data_3", "/wf/corner_radar/rd_data_4",
    "/cv_camera_0/image_raw/compressed","/cv_camera_1/image_raw/compressed", "/cv_camera_2/image_raw/compressed",
    "/cv_camera_3/image_raw/compressed", "/cv_camera_4/image_raw/compressed","/cv_camera_5/image_raw/compressed",
    "/wf/car_id6/parsed2"};
  rosbag::View view(bag_, rosbag::TopicQuery(topics));

  // 遍历读取的消息，按话题分类存储
  if(update_progress_bar_callback_)
  {
    update_progress_bar_callback_(0.2f);
  }

  size_t count = 0;
  size_t totalCount = view.size();

 for (const auto& msg : view)
  {
    const std::string& topic = msg.getTopic();// 根据话题将消息存入对应的容器
    
    if (topic == "/wf/car_id6/parsed2")
      car_msgs_.push_back(msg);
    else if (topic == "/wf/corner_radar/parsed/float_data_0")
      pointcloud_msgs0_.push_back(msg);
    else if (topic == "/wf/corner_radar/parsed/float_data_1")
      pointcloud_msgs1_.push_back(msg);
    else if (topic == "/wf/corner_radar/parsed/float_data_2")
      pointcloud_msgs2_.push_back(msg);
    else if (topic == "/wf/corner_radar/parsed/float_data_3")
      pointcloud_msgs3_.push_back(msg);
    else if (topic == "/wf/corner_radar/parsed/float_data_4")
      pointcloud_msgs4_.push_back(msg);
    else if (topic == "/wf/frame_rd_data/ti/radar_0")
      pointcloud_sp_msgs0_.push_back(msg);
    else if (topic == "/wf/frame_rd_data/ti/radar_1")
      pointcloud_sp_msgs1_.push_back(msg);
    else if (topic == "/wf/frame_rd_data/ti/radar_2")
      pointcloud_sp_msgs2_.push_back(msg);
    else if (topic == "/wf/frame_rd_data/ti/radar_3")
      pointcloud_sp_msgs3_.push_back(msg);
    else if (topic == "/wf/frame_rd_data/ti/radar_4")
      pointcloud_sp_msgs4_.push_back(msg);
    else if (topic == "/wf/corner_radar/rd_data_0")
      pointcloud_rd_msgs0_.push_back(msg);
    else if (topic == "/wf/corner_radar/rd_data_1")
      pointcloud_rd_msgs1_.push_back(msg);
    else if (topic == "/wf/corner_radar/rd_data_2")
      pointcloud_rd_msgs2_.push_back(msg);
    else if (topic == "/wf/corner_radar/rd_data_3")
      pointcloud_rd_msgs3_.push_back(msg);
    else if (topic == "/wf/corner_radar/rd_data_4")
      pointcloud_rd_msgs4_.push_back(msg);
    else if (topic == "/cv_camera_0/image_raw/compressed")
      camera_msgs0_.push_back(msg);
    else if (topic == "/cv_camera_1/image_raw/compressed")
      camera_msgs1_.push_back(msg);
    else if (topic == "/cv_camera_2/image_raw/compressed")
      camera_msgs2_.push_back(msg);
    else if (topic == "/cv_camera_3/image_raw/compressed")
      camera_msgs3_.push_back(msg);
    else if (topic == "/cv_camera_4/image_raw/compressed")
      camera_msgs4_.push_back(msg);
    else if (topic == "/cv_camera_5/image_raw/compressed")
      camera_msgs5_.push_back(msg);

    count++;
    if(update_progress_bar_callback_)
      update_progress_bar_callback_(((float)count)/((float)totalCount) + 0.2);
  }

  // 初始化当前帧为 0
  current_frame_ = 0;

  frameCount0 = pointcloud_msgs0_.size();  // 获取各话题消息的数量
  frameSPCount0 = pointcloud_sp_msgs0_.size();
  frameRDCount0 = pointcloud_rd_msgs0_.size();
  
  frameCount1 = pointcloud_msgs1_.size();
  frameSPCount1 = pointcloud_sp_msgs1_.size();
  frameRDCount1 = pointcloud_rd_msgs1_.size();

  frameCount2 = pointcloud_msgs2_.size();
  frameSPCount2 = pointcloud_sp_msgs2_.size();
  frameRDCount2 = pointcloud_rd_msgs2_.size();

  frameCount3 = pointcloud_msgs3_.size();
  frameSPCount3 = pointcloud_sp_msgs3_.size();
  frameRDCount3 = pointcloud_rd_msgs3_.size();

  frameCount4 = pointcloud_msgs4_.size();
  frameSPCount4 = pointcloud_sp_msgs4_.size();
  frameRDCount4 = pointcloud_rd_msgs4_.size();

  if(frameRDCount0>0)// 如果存在雷达数据，将第一个雷达数据作为空消息的默认值
    empty_msgs_.push_back(pointcloud_rd_msgs0_[0]);
  else if(frameRDCount1>0)
    empty_msgs_.push_back(pointcloud_rd_msgs1_[0]);
  else if(frameRDCount2>0)
    empty_msgs_.push_back(pointcloud_rd_msgs2_[0]);
  else if(frameRDCount3>0)
    empty_msgs_.push_back(pointcloud_rd_msgs3_[0]);
  else if(frameRDCount4>0)
    empty_msgs_.push_back(pointcloud_rd_msgs4_[0]);

  for(int i=0;i<23;i++)// 初始化消息标志数组
    msg_flags_.push_back(-1);
}

int BagReader::getMainRadarPclSize()// 获取主雷达点云数据的大小
{
 switch (mainRadarIndex_)
  {
    case 0: return pointcloud_rd_msgs0_.size();
    case 1: return pointcloud_rd_msgs1_.size();
    case 2: return pointcloud_rd_msgs2_.size();
    case 3: return pointcloud_rd_msgs3_.size();
    case 4: default: return pointcloud_rd_msgs4_.size();
  }
}

ros::Time BagReader::getMainRadarTime(int curIdx)// 获取主雷达在指定索引处的时间戳
{
  switch (mainRadarIndex_)
  {
    case 0: return pointcloud_rd_msgs0_[curIdx].getTime();
    case 1: return pointcloud_rd_msgs1_[curIdx].getTime();
    case 2: return pointcloud_rd_msgs2_[curIdx].getTime();
    case 3: return pointcloud_rd_msgs3_[curIdx].getTime();
    case 4: default: return pointcloud_rd_msgs4_[curIdx].getTime();
  }
}

void BagReader::packetCallbackMsg()// 回调函数：根据当前消息标志填充当前帧的消息
{
  frame_msgs_.clear();
  for(int i=0;i<MAX_TOPIC_NUM;i++)
  {
    if(msg_flags_[i]<0){
      frame_msgs_.push_back(empty_msgs_[0]);// 如果没有对应消息，使用空消息
    }
    else{
      switch (i)// 根据索引从对应的消息容器中获取消息
      {
        case 0:
            frame_msgs_.push_back(pointcloud_sp_msgs0_[msg_flags_[i]]); break;
        case 1:
            frame_msgs_.push_back(pointcloud_sp_msgs1_[msg_flags_[i]]); break;
        case 2:
            frame_msgs_.push_back(pointcloud_sp_msgs2_[msg_flags_[i]]); break;
        case 3:
            frame_msgs_.push_back(pointcloud_sp_msgs3_[msg_flags_[i]]); break;
        case 4:
            frame_msgs_.push_back(pointcloud_sp_msgs4_[msg_flags_[i]]); break;
        case 6:
          frame_msgs_.push_back(camera_msgs0_[msg_flags_[i]]); break;
        case 7: //camera 1
          frame_msgs_.push_back(camera_msgs1_[msg_flags_[i]]); break;  
        case 8: //camera 2
          frame_msgs_.push_back(camera_msgs2_[msg_flags_[i]]); break;
        case 9: //camera 3
          frame_msgs_.push_back(camera_msgs3_[msg_flags_[i]]); break;
        case 10: //camera 4
          frame_msgs_.push_back(camera_msgs4_[msg_flags_[i]]); break;
        case 11: //camera 5
          frame_msgs_.push_back(camera_msgs5_[msg_flags_[i]]); break;
        case 12: //car
          frame_msgs_.push_back(car_msgs_[msg_flags_[i]]); break;
        case 13:
          frame_msgs_.push_back(pointcloud_msgs0_[msg_flags_[i]]); break;
        case 14:
          frame_msgs_.push_back(pointcloud_msgs1_[msg_flags_[i]]); break;
        case 15:  
          frame_msgs_.push_back(pointcloud_msgs2_[msg_flags_[i]]); break;
        case 16:  
          frame_msgs_.push_back(pointcloud_msgs3_[msg_flags_[i]]); break;
        case 17:  
          frame_msgs_.push_back(pointcloud_msgs4_[msg_flags_[i]]); break;
        case 18:
          frame_msgs_.push_back(pointcloud_rd_msgs0_[msg_flags_[i]]); break;
        case 19:
          frame_msgs_.push_back(pointcloud_rd_msgs1_[msg_flags_[i]]); break;
        case 20:
          frame_msgs_.push_back(pointcloud_rd_msgs2_[msg_flags_[i]]); break;
        case 21:
          frame_msgs_.push_back(pointcloud_rd_msgs3_[msg_flags_[i]]); break;
        case 22:
          frame_msgs_.push_back(pointcloud_rd_msgs4_[msg_flags_[i]]); break;

        default: break;
      }
    }
  }  
}


void BagReader::jumpToFrame(int frame_number)// 跳转到指定帧
{
  int mainRadarSize = getMainRadarPclSize();
  
  if (frame_number >= 0 && frame_number < mainRadarSize)
  {
    current_frame_ = frame_number;
    if (message_callback_)
    {
      ros::Time selected_time = getMainRadarTime(current_frame_);
      // 为每个消息类型找到最接近选定时间戳的消息索引
      msg_flags_[0] = findClosestFPtFrame(selected_time,pointcloud_sp_msgs0_);
      msg_flags_[1] = findClosestFPtFrame(selected_time,pointcloud_sp_msgs1_);
      msg_flags_[2] = findClosestFPtFrame(selected_time,pointcloud_sp_msgs2_);
      msg_flags_[3] = findClosestFPtFrame(selected_time,pointcloud_sp_msgs3_);
      msg_flags_[4] = findClosestFPtFrame(selected_time,pointcloud_sp_msgs4_);

      msg_flags_[6] = findClosestCameraFrame(selected_time,camera_msgs0_);
      msg_flags_[7] = findClosestCameraFrame(selected_time,camera_msgs1_);
      msg_flags_[8]= findClosestCameraFrame(selected_time,camera_msgs2_);
      msg_flags_[9] = findClosestCameraFrame(selected_time,camera_msgs3_);
      msg_flags_[10] = findClosestCameraFrame(selected_time,camera_msgs4_);
      msg_flags_[11] = findClosestCameraFrame(selected_time,camera_msgs5_);

      msg_flags_[12]= findClosestCarFrame(selected_time);

      msg_flags_[13] = findClosestPtFrame(selected_time,pointcloud_msgs0_);
      msg_flags_[14] = findClosestPtFrame(selected_time,pointcloud_msgs1_);
      msg_flags_[15] = findClosestPtFrame(selected_time,pointcloud_msgs2_);
      msg_flags_[16] = findClosestPtFrame(selected_time,pointcloud_msgs3_);
      msg_flags_[17] = findClosestPtFrame(selected_time,pointcloud_msgs4_);

      msg_flags_[18] = findClosestRPtFrame(selected_time, pointcloud_rd_msgs0_);
      msg_flags_[19] = findClosestRPtFrame(selected_time, pointcloud_rd_msgs1_);
      msg_flags_[20] = findClosestRPtFrame(selected_time, pointcloud_rd_msgs2_);
      msg_flags_[21] = findClosestRPtFrame(selected_time, pointcloud_rd_msgs3_);
      msg_flags_[22] = findClosestRPtFrame(selected_time, pointcloud_rd_msgs4_);
      
      packetCallbackMsg();// 填充当前帧消息
      message_callback_(frame_msgs_,current_frame_,msg_flags_);// 回调通知外部
      ROS_INFO("Playing frame %d", current_frame_);
    }
  }
}

void BagReader::playBag(){// 开始播放bag文件
  if (!playing_) {
      playing_ = true;
      play_thread_ = std::thread(&BagReader::playLoop, this);
  }
}

void BagReader::stopBag() {// 停止播放
  if (playing_) {
      finishProcessFlag_ = true;
      playing_ = false;
      cv_.notify_all();
      if (play_thread_.joinable()) {
            play_thread_.join();
      }
  }
}

void BagReader::setPlayRate(double rate)
{
  play_rate_ = rate;
}

void BagReader::setMessageCallback(MessageCallback callback)
{
  message_callback_ = callback;//
}

void BagReader::setUpdateProgressBarCallback(UpdateProgressBarCallback callback)
{
  update_progress_bar_callback_ = callback;
}

int BagReader::getCurrentFrame() const
{
  return current_frame_;
}

int BagReader::findClosestCameraFrame(const ros::Time& selected_time, std::vector<rosbag::MessageInstance>& camera_msgs)
{// 查找最接近指定时间的相机消息索引
    int closest_index = -1;
    double min_diff = std::numeric_limits<double>::max();

    for (size_t i = 0; i < camera_msgs.size(); ++i) {
        ros::Time camera_time = camera_msgs[i].getTime();
        double time_diff = std::abs((selected_time - camera_time).toSec());

        if (time_diff < min_diff) {
            min_diff = time_diff;
            closest_index = i;
        }
    }

    return closest_index;
}


int BagReader::findClosestCarFrame(const ros::Time& selected_time)
{// 查找最接近指定时间的车辆消息索引
    int closest_index = -1;
    double min_diff = std::numeric_limits<double>::max();

    for (size_t i = 0; i < car_msgs_.size(); ++i) {
        ros::Time car_time = car_msgs_[i].getTime();
        double time_diff = std::abs((selected_time - car_time).toSec());

        if (time_diff < min_diff) {
            min_diff = time_diff;
            closest_index = i;
        }
    }

    return closest_index;
}

int BagReader::findClosestPtFrame(const ros::Time& selected_time, std::vector<rosbag::MessageInstance>& pcl_msgs)
{// 查找最接近指定时间的点云消息索引
    int closest_index = -1;
    double min_diff = std::numeric_limits<double>::max();
    for (size_t i = 0; i < pcl_msgs.size(); ++i) {
      ros::Time pt_time = pcl_msgs[i].getTime();
      double time_diff = std::abs((selected_time - pt_time).toSec());

      if (time_diff < min_diff) {
          min_diff = time_diff;
          closest_index = i;
      }
    }

    return closest_index;
}

int BagReader::findClosestFPtFrame(const ros::Time& selected_time, std::vector<rosbag::MessageInstance>& pcl_sp_msgs)
{// 查找最接近指定时间的特殊点云消息索引
    int closest_index = -1;
    double min_diff = std::numeric_limits<double>::max();
    for (size_t i = 0; i < pcl_sp_msgs.size(); ++i) {
      ros::Time spt_time = pcl_sp_msgs[i].getTime();
      double time_diff = std::abs((selected_time - spt_time).toSec());

      if (time_diff < min_diff) {
          min_diff = time_diff;
          closest_index = i;
      }
    }

    return closest_index;
}

int BagReader::findClosestRPtFrame(const ros::Time& selected_time, std::vector<rosbag::MessageInstance>& pcl_rd_msgs)
{// 查找最接近指定时间的雷达点云消息索引
    int closest_index = -1;
    double min_diff = std::numeric_limits<double>::max();
    for (size_t i = 0; i < pcl_rd_msgs.size(); ++i) {
      ros::Time rpt_time = pcl_rd_msgs[i].getTime();
      double time_diff = std::abs((selected_time - rpt_time).toSec());

      if (time_diff < min_diff) {
          min_diff = time_diff;
          closest_index = i;
      }
    }

    return closest_index;
}



void BagReader::playLoop() // 播放循环：按帧播放bag数据
{

  int last_msg_flag_19 = -1;
  int last_msg_flag_20 = -1;
  int last_msg_flag_21 = -1;
  int last_msg_flag_22 = -1;
  int now_Fix = -1;

  int mainRadarSize = getMainRadarPclSize();
   if (current_frame_ != 0)//2025/9/18
       current_frame_=current_frame_+1;// 从下一帧开始播放

  for (size_t i = current_frame_; i < mainRadarSize; ++i) 
  {
    while(!finishProcessFlag_)  
      std::this_thread::sleep_for(std::chrono::milliseconds(1));// 等待处理完成标志

    std::lock_guard<std::mutex> lock(mutex_);
    if (!playing_) // 如果停止播放，退出循环
      break;
    
    current_frame_ = i;
    ros::Time selected_time = getMainRadarTime(current_frame_); // 为每个消息类型找到最接近的时间戳索引

    if (current_frame_ > 20 && current_frame_ < mainRadarSize - 10 && now_Fix != 80) 
    {
        msg_flags_[18] = findClosestRPtFrame(selected_time, pointcloud_rd_msgs0_);
        if(last_msg_flag_19 != -1){
          last_msg_flag_19++;
          msg_flags_[19] = last_msg_flag_19;
        }

        if(last_msg_flag_20 != -1){
          last_msg_flag_20++;
          msg_flags_[20] = last_msg_flag_20;
        }
        if(last_msg_flag_21 != -1){
          last_msg_flag_21++;
          msg_flags_[21] = last_msg_flag_21;
        }

        if(last_msg_flag_22 != -1){
          last_msg_flag_22++;
          msg_flags_[22] = last_msg_flag_22;
        }

        ROS_INFO("go to now_Fix %d", now_Fix);
    } 
    else 

    {
      msg_flags_[18] = findClosestRPtFrame(selected_time, pointcloud_rd_msgs0_);
      msg_flags_[19] = findClosestRPtFrame(selected_time, pointcloud_rd_msgs1_);
      msg_flags_[20] = findClosestRPtFrame(selected_time, pointcloud_rd_msgs2_);
      msg_flags_[21] = findClosestRPtFrame(selected_time, pointcloud_rd_msgs3_);
      msg_flags_[22] = findClosestRPtFrame(selected_time, pointcloud_rd_msgs4_);
      
      last_msg_flag_19=msg_flags_[19];
      last_msg_flag_20=msg_flags_[20];
      last_msg_flag_21=msg_flags_[21];
      last_msg_flag_22=msg_flags_[22];
      now_Fix=0;
    }
    now_Fix++;

    // 根据 pointcloud_rd_msgs 帧索引，获取对应的时间戳
    ros::Time float_data_time0 = (msg_flags_[18] >= 0) ? pointcloud_rd_msgs0_[msg_flags_[18]].getTime() : selected_time;
    ros::Time float_data_time1 = (msg_flags_[19] >= 0) ? pointcloud_rd_msgs1_[msg_flags_[19]].getTime() : selected_time;
    ros::Time float_data_time2 = (msg_flags_[20] >= 0) ? pointcloud_rd_msgs2_[msg_flags_[20]].getTime() : selected_time;
    ros::Time float_data_time3 = (msg_flags_[21] >= 0) ? pointcloud_rd_msgs3_[msg_flags_[21]].getTime() : selected_time;
    ros::Time float_data_time4 = (msg_flags_[22] >= 0) ? pointcloud_rd_msgs4_[msg_flags_[22]].getTime() : selected_time;


    msg_flags_[0] = findClosestFPtFrame(selected_time,pointcloud_sp_msgs0_);
    msg_flags_[1] = findClosestFPtFrame(selected_time,pointcloud_sp_msgs1_);
    msg_flags_[2] = findClosestFPtFrame(selected_time,pointcloud_sp_msgs2_);
    msg_flags_[3] = findClosestFPtFrame(selected_time,pointcloud_sp_msgs3_);
    msg_flags_[4] = findClosestFPtFrame(selected_time,pointcloud_sp_msgs4_);

    msg_flags_[6] = findClosestCameraFrame(selected_time,camera_msgs0_);
    msg_flags_[7] = findClosestCameraFrame(selected_time,camera_msgs1_);
    msg_flags_[8]= findClosestCameraFrame(selected_time,camera_msgs2_);
    msg_flags_[9] = findClosestCameraFrame(selected_time,camera_msgs3_);
    msg_flags_[10] = findClosestCameraFrame(selected_time,camera_msgs4_);
    msg_flags_[11] = findClosestCameraFrame(selected_time,camera_msgs5_);

    msg_flags_[12]= findClosestCarFrame(selected_time);
    
    msg_flags_[13] = findClosestPtFrame(float_data_time0,pointcloud_msgs0_);
    msg_flags_[14] = findClosestPtFrame(float_data_time1,pointcloud_msgs1_);
    msg_flags_[15] = findClosestPtFrame(float_data_time2,pointcloud_msgs2_);
    msg_flags_[16] = findClosestPtFrame(float_data_time3,pointcloud_msgs3_);
    msg_flags_[17] = findClosestPtFrame(float_data_time4,pointcloud_msgs4_);

    packetCallbackMsg();
    finishProcessFlag_ = false;
    message_callback_(frame_msgs_,current_frame_,msg_flags_);
    ROS_INFO("Playing frame %d", current_frame_);
    std::this_thread::sleep_for(std::chrono::milliseconds((int)(50 / play_rate_)));//ms/1s   控制播放速度
  }
}

// 设置处理完成标志
void BagReader::setFinishProcessFlag(bool flag)
{
  finishProcessFlag_ = flag;
}

// 设置特殊点云播放标志
void BagReader::setSPFlag(bool flag)
{
  bPlaySPFlag_ = flag;
}

// 选择主雷达索引
void BagReader::selectMainRadar(int index)
{
  mainRadarIndex_ = index;
}

} // namespace my_rviz_plugin
