#include "my_rviz_plugin/bag_reader.h"
#include <rosbag/view.h>
#include <chrono>
#include <thread>

namespace my_rviz_plugin
{

BagReader::BagReader() : current_frame_(0), play_rate_(1.0), playing_(false), finishProcessFlag_(true), bPlaySPFlag_(false), mainRadarIndex_(3)  
{
  
}

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

void BagReader::readBagFile(const std::string& file_path, int& frameCount0, int& frameCount1, int& frameCount2, 
int& frameCount3, int& frameCount4)
{
  if(update_progress_bar_callback_)
  {
    update_progress_bar_callback_(0.05f);
  }
  stopBag(); 

  if (bag_.isOpen()) {
    ROS_INFO("Closing previous bag file.");
    bag_.close();
  }


  if(update_progress_bar_callback_)
  {
    update_progress_bar_callback_(0.1f);
  }

  car_msgs_.clear();

  camera_msgs0_.clear();
  camera_msgs1_.clear();
  camera_msgs2_.clear();
  camera_msgs3_.clear();
  camera_msgs4_.clear();
  camera_msgs5_.clear();

  pointcloud_msgs0_.clear();
  pointcloud_msgs1_.clear();
  pointcloud_msgs2_.clear();
  pointcloud_msgs3_.clear();
  pointcloud_msgs4_.clear();

  corner_radar_warning_msgs_.clear();
  empty_msgs_.clear();
  msg_flags_.clear();

  bag_.open(file_path, rosbag::bagmode::Read);

  if(update_progress_bar_callback_)
  {
    update_progress_bar_callback_(0.15f);
  }

  // 只读取指定的话题
  std::vector<std::string> topics = {"/wf/corner_radar/lgu_data_0","/wf/corner_radar/lgu_data_1", "/wf/corner_radar/lgu_data_2", 
    "/wf/corner_radar/lgu_data_3", "/wf/corner_radar/lgu_data_4", 
    "/cv_camera_0/image_raw/compressed","/cv_camera_1/image_raw/compressed", "/cv_camera_2/image_raw/compressed",
    "/cv_camera_3/image_raw/compressed", "/cv_camera_4/image_raw/compressed","/cv_camera_5/image_raw/compressed",
    "/wf/car_id6/parsed2","/corner_radar/warning_status"
  
  };
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
    if (msg.getTopic() == "/wf/car_id6/parsed2")
    {
      car_msgs_.push_back(msg);
    }
    else if (msg.getTopic() == "/wf/corner_radar/lgu_data_0")
    {
      pointcloud_msgs0_.push_back(msg);
    }
    else if (msg.getTopic() == "/wf/corner_radar/lgu_data_1")
    {
      pointcloud_msgs1_.push_back(msg);
    }
    else if (msg.getTopic() == "/wf/corner_radar/lgu_data_2")
    {
      pointcloud_msgs2_.push_back(msg);
    }
    else if (msg.getTopic() == "/wf/corner_radar/lgu_data_3")
    {
      pointcloud_msgs3_.push_back(msg);
    }
    else if (msg.getTopic() == "/wf/corner_radar/lgu_data_4")
    {
      pointcloud_msgs4_.push_back(msg);
    }
    else if (msg.getTopic() == "/corner_radar/warning_status")  
    {
      corner_radar_warning_msgs_.push_back(msg);
    }
    else if (msg.getTopic() == "/cv_camera_0/image_raw/compressed")
    {
      camera_msgs0_.push_back(msg);
    }
    else if (msg.getTopic() == "/cv_camera_1/image_raw/compressed")
    {
      camera_msgs1_.push_back(msg);
    }
    else if (msg.getTopic() == "/cv_camera_2/image_raw/compressed")
    {
      camera_msgs2_.push_back(msg);
    }
    else if (msg.getTopic() == "/cv_camera_3/image_raw/compressed")
    {
      camera_msgs3_.push_back(msg);
    }
    else if (msg.getTopic() == "/cv_camera_4/image_raw/compressed")
    {
      camera_msgs4_.push_back(msg);
    }
    else if (msg.getTopic() == "/cv_camera_5/image_raw/compressed")
    {
      camera_msgs5_.push_back(msg);
    }

    count++;
    if(update_progress_bar_callback_)
    {
      update_progress_bar_callback_(((float)count)/((float)totalCount) + 0.2);
    }
  }

  // 初始化当前帧为 0
  current_frame_ = 0;

  frameCount0 = pointcloud_msgs0_.size();
  frameCount1 = pointcloud_msgs1_.size();
  frameCount2 = pointcloud_msgs2_.size();
  frameCount3 = pointcloud_msgs3_.size();
  frameCount4 = pointcloud_msgs4_.size();


  if(frameCount0>0)
  {
    empty_msgs_.push_back(pointcloud_msgs0_[0]);
  }
  else if(frameCount1>0)
  {
    empty_msgs_.push_back(pointcloud_msgs1_[0]);
  }
  else if(frameCount2>0)
  {
    empty_msgs_.push_back(pointcloud_msgs2_[0]);
  }
  else if(frameCount3>0)
  {
    empty_msgs_.push_back(pointcloud_msgs3_[0]);
  }
  else if(frameCount4>0)
  {
    empty_msgs_.push_back(pointcloud_msgs4_[0]);
  }
  for(int i=0;i<13;i++)
  msg_flags_.push_back(-1);
}

int BagReader::getMainRadarPclSize()
{
  int result = 0;

  switch (mainRadarIndex_)
  {

    case 0:
      result = pointcloud_msgs0_.size();
    break;
  case 1:
      result = pointcloud_msgs1_.size();
    break;
  case 2:
      result = pointcloud_msgs2_.size();
    break;
  case 3:
      result = pointcloud_msgs3_.size();
    break;
  case 4:
  default:
      result = pointcloud_msgs4_.size();

    break;
  }

  return result;
}

ros::Time BagReader::getMainRadarTime(int curIdx)
{
  ros::Time result;

  switch (mainRadarIndex_)
  {
    case 0:
      result = pointcloud_msgs0_[curIdx].getTime();
    break;
  case 1:
      result = pointcloud_msgs1_[curIdx].getTime();
    break;
  case 2:
      result = pointcloud_msgs2_[curIdx].getTime();
    break;
  case 3:
      result = pointcloud_msgs3_[curIdx].getTime();
    break;
  case 4:
  default:
      result = pointcloud_msgs4_[curIdx].getTime();
    break;
  }

  return result;
}

void BagReader::packetCallbackMsg()
{
  frame_msgs_.clear();


  for(int i=0;i<MAX_TOPIC_NUM;i++)
  {
    if(msg_flags_[i]<0)
    {
      frame_msgs_.push_back(empty_msgs_[0]);
    }
    else
    {
      switch (i)
      {
        case 0:
            frame_msgs_.push_back(pointcloud_msgs0_[msg_flags_[i]]);
          break;
        case 1:
            frame_msgs_.push_back(pointcloud_msgs1_[msg_flags_[i]]);
          break;
        case 2:
            frame_msgs_.push_back(pointcloud_msgs2_[msg_flags_[i]]);
          break;
        case 3:
            frame_msgs_.push_back(pointcloud_msgs3_[msg_flags_[i]]);  
          break;
        case 4:
            frame_msgs_.push_back(pointcloud_msgs4_[msg_flags_[i]]);
          break;
        case 5:
          frame_msgs_.push_back(corner_radar_warning_msgs_[msg_flags_[i]]);
          break;
        case 6:
          frame_msgs_.push_back(camera_msgs0_[msg_flags_[i]]);
          break;
        case 7:
          //camera 1
          frame_msgs_.push_back(camera_msgs1_[msg_flags_[i]]);
          break;
        case 8:
          //camera 2
          frame_msgs_.push_back(camera_msgs2_[msg_flags_[i]]);
          break;
        case 9:
          //camera 3
          frame_msgs_.push_back(camera_msgs3_[msg_flags_[i]]);
          break;
        case 10:
          //camera 4
          frame_msgs_.push_back(camera_msgs4_[msg_flags_[i]]);
          break;
        case 11:
          //camera 5
          frame_msgs_.push_back(camera_msgs5_[msg_flags_[i]]);
          break;
        case 12:
          //car
          frame_msgs_.push_back(car_msgs_[msg_flags_[i]]);
          break;
        default:  
          break;
      }
    }
  }
  
}

void BagReader::jumpToFrame(int frame_number)
{
  int mainRadarSize = getMainRadarPclSize();
  
  if (frame_number >= 0 && frame_number < mainRadarSize)
  {
    current_frame_ = frame_number;
    if (message_callback_)
    {
      ros::Time selected_time = getMainRadarTime(current_frame_);

      msg_flags_[0] = findClosestPtFrame(selected_time,pointcloud_msgs0_);
      msg_flags_[1] = findClosestPtFrame(selected_time,pointcloud_msgs1_);
      msg_flags_[2] = findClosestPtFrame(selected_time,pointcloud_msgs2_);
      msg_flags_[3] = findClosestPtFrame(selected_time,pointcloud_msgs3_);
      msg_flags_[4] = findClosestPtFrame(selected_time,pointcloud_msgs4_);

      msg_flags_[5] = findClosestPtFrame(selected_time,corner_radar_warning_msgs_);

      msg_flags_[6] = findClosestCameraFrame(selected_time,camera_msgs0_);
      msg_flags_[7] = findClosestCameraFrame(selected_time,camera_msgs1_);
      msg_flags_[8]= findClosestCameraFrame(selected_time,camera_msgs2_);
      msg_flags_[9] = findClosestCameraFrame(selected_time,camera_msgs3_);
      msg_flags_[10] = findClosestCameraFrame(selected_time,camera_msgs4_);
      msg_flags_[11] = findClosestCameraFrame(selected_time,camera_msgs5_);

      msg_flags_[12]= findClosestCarFrame(selected_time);

      packetCallbackMsg();
      message_callback_(frame_msgs_,current_frame_,msg_flags_);
      ROS_INFO("Playing frame %d", current_frame_);

    }
  }
}

void BagReader::playBag()
{
  if (!playing_) {
      playing_ = true;
      play_thread_ = std::thread(&BagReader::playLoop, this);
  }
}

void BagReader::stopBag() {
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
{
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
{
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
{
    int closest_index = -1;
    double min_diff = std::numeric_limits<double>::max();

    auto& auto_msgs = pcl_msgs;

    for (size_t i = 0; i < auto_msgs.size(); ++i) 
    {
        ros::Time msg_time = auto_msgs[i].getTime();
        double time_diff = std::abs((selected_time - msg_time).toSec());

        if (time_diff < min_diff) {
            min_diff = time_diff;
            closest_index = i;
        }
    }

    return closest_index;
}


void BagReader::playLoop() 
{
  int mainRadarSize = getMainRadarPclSize();
  if (current_frame_ != 0)//2025/9/17
    current_frame_=current_frame_+1;

  for (size_t i = current_frame_; i < mainRadarSize; ++i) 
  {
    while(!finishProcessFlag_)
      std::this_thread::sleep_for(std::chrono::milliseconds(1));
    
    std::lock_guard<std::mutex> lock(mutex_);
    if (!playing_) 
        break;
    
    current_frame_ = i;

    
    ros::Time selected_time = getMainRadarTime(current_frame_);

    msg_flags_[0] = findClosestPtFrame(selected_time,pointcloud_msgs0_);
    msg_flags_[1] = findClosestPtFrame(selected_time,pointcloud_msgs1_);
    msg_flags_[2] = findClosestPtFrame(selected_time,pointcloud_msgs2_);
    msg_flags_[3] = findClosestPtFrame(selected_time,pointcloud_msgs3_);
    msg_flags_[4] = findClosestPtFrame(selected_time,pointcloud_msgs4_);

    msg_flags_[5] = findClosestPtFrame(selected_time,corner_radar_warning_msgs_);

    msg_flags_[6] = findClosestCameraFrame(selected_time,camera_msgs0_);
    msg_flags_[7] = findClosestCameraFrame(selected_time,camera_msgs1_);
    msg_flags_[8]= findClosestCameraFrame(selected_time,camera_msgs2_);
    msg_flags_[9] = findClosestCameraFrame(selected_time,camera_msgs3_);
    msg_flags_[10] = findClosestCameraFrame(selected_time,camera_msgs4_);
    msg_flags_[11] = findClosestCameraFrame(selected_time,camera_msgs5_);

    msg_flags_[12]= findClosestCarFrame(selected_time);


    packetCallbackMsg();
    finishProcessFlag_ = false;
    message_callback_(frame_msgs_,current_frame_,msg_flags_);
    ROS_INFO("Playing frame %d", current_frame_);
    std::this_thread::sleep_for(std::chrono::milliseconds((int)(50 / play_rate_)));//ms/1s
  }
}

void BagReader::setFinishProcessFlag(bool flag)
{
  finishProcessFlag_ = flag;
}


void BagReader::selectMainRadar(int index)
{
  mainRadarIndex_ = index;
}

} // namespace my_rviz_plugin
