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

void BagReader::readBagFile(const std::string& file_path, int& frameCount0, int& frameSPCount0,int& frameCount1, int& frameSPCount1, int& frameCount2, 
  int& frameSPCount2,int& frameCount3, int& frameSPCount3,int& frameCount4, int& frameSPCount4)
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

  pointcloud_sp_msgs0_.clear();
  pointcloud_sp_msgs1_.clear();
  pointcloud_sp_msgs2_.clear();
  pointcloud_sp_msgs3_.clear();
  pointcloud_sp_msgs4_.clear();

  Object_tag_msgs_1.clear();
  Object_tag_msgs_2.clear();
  Object_tag_msgs_3.clear();
  Object_tag_msgs_4.clear();

  Object_msgs_1.clear();
  Object_msgs_2.clear();
  Object_msgs_3.clear();
  Object_msgs_4.clear();

  IMU_msgs_.clear();

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
    "/cv_camera_0/image_raw/compressed","/cv_camera_1/image_raw/compressed", "/cv_camera_2/image_raw/compressed",
    "/cv_camera_3/image_raw/compressed", "/cv_camera_4/image_raw/compressed","/cv_camera_5/image_raw/compressed",
    "/wf/car_id6/parsed2"
    ,
    "/wf/imu_data/parsed",
    "/wf/rviz/objects_tags_1","/wf/rviz/objects_tags_2",  "/wf/rviz/objects_tags_3",  "/wf/rviz/objects_tags_4",
    "/wf/rviz/objects_1", "/wf/rviz/objects_2", "/wf/rviz/objects_3", "/wf/rviz/objects_4"
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
    if (msg.getTopic() == "/wf/car_id6/parsed2")  car_msgs_.push_back(msg);
    else if (msg.getTopic() == "/wf/corner_radar/parsed/float_data_0")  pointcloud_msgs0_.push_back(msg);
    else if (msg.getTopic() == "/wf/corner_radar/parsed/float_data_1")  pointcloud_msgs1_.push_back(msg);
    else if (msg.getTopic() == "/wf/corner_radar/parsed/float_data_2")  pointcloud_msgs2_.push_back(msg);
    else if (msg.getTopic() == "/wf/corner_radar/parsed/float_data_3")  pointcloud_msgs3_.push_back(msg);
    else if (msg.getTopic() == "/wf/corner_radar/parsed/float_data_4")  pointcloud_msgs4_.push_back(msg);
    else if(msg.getTopic() == "/wf/frame_rd_data/ti/radar_0") pointcloud_sp_msgs0_.push_back(msg);
    else if(msg.getTopic() == "/wf/frame_rd_data/ti/radar_1") pointcloud_sp_msgs1_.push_back(msg);
    else if(msg.getTopic() == "/wf/frame_rd_data/ti/radar_2") pointcloud_sp_msgs2_.push_back(msg);
    else if(msg.getTopic() == "/wf/frame_rd_data/ti/radar_3") pointcloud_sp_msgs3_.push_back(msg);
    else if(msg.getTopic() == "/wf/frame_rd_data/ti/radar_4") pointcloud_sp_msgs4_.push_back(msg);
    else if (msg.getTopic() == "/cv_camera_0/image_raw/compressed") camera_msgs0_.push_back(msg);
    else if (msg.getTopic() == "/cv_camera_1/image_raw/compressed") camera_msgs1_.push_back(msg);
    else if (msg.getTopic() == "/cv_camera_2/image_raw/compressed") camera_msgs2_.push_back(msg);
    else if (msg.getTopic() == "/cv_camera_3/image_raw/compressed") camera_msgs3_.push_back(msg);
    else if (msg.getTopic() == "/cv_camera_4/image_raw/compressed") camera_msgs4_.push_back(msg);
    else if (msg.getTopic() == "/cv_camera_5/image_raw/compressed") camera_msgs5_.push_back(msg);
    else if (msg.getTopic() == "/wf/imu_data/parsed") IMU_msgs_.push_back(msg);
    else if (msg.getTopic() == "/wf/rviz/objects_tags_1") Object_tag_msgs_1.push_back(msg);
    else if (msg.getTopic() == "/wf/rviz/objects_tags_2") Object_tag_msgs_2.push_back(msg);
    else if (msg.getTopic() == "/wf/rviz/objects_tags_3") Object_tag_msgs_3.push_back(msg);
    else if (msg.getTopic() == "/wf/rviz/objects_tags_4") Object_tag_msgs_4.push_back(msg);
    else if (msg.getTopic() == "/wf/rviz/objects_1")  Object_msgs_1.push_back(msg);
    else if (msg.getTopic() == "/wf/rviz/objects_2")  Object_msgs_2.push_back(msg);
    else if (msg.getTopic() == "/wf/rviz/objects_3")  Object_msgs_3.push_back(msg);
    else if (msg.getTopic() == "/wf/rviz/objects_4")  Object_msgs_4.push_back(msg);

    count++;
    if(update_progress_bar_callback_) 
     update_progress_bar_callback_(((float)count)/((float)totalCount) + 0.2);

  }

  // 初始化当前帧为 0
  current_frame_ = 0;

  frameCount0 = pointcloud_msgs0_.size();
  frameSPCount0 = pointcloud_sp_msgs0_.size();
  
  frameCount1 = pointcloud_msgs1_.size();
  frameSPCount1 = pointcloud_sp_msgs1_.size();

  frameCount2 = pointcloud_msgs2_.size();
  frameSPCount2 = pointcloud_sp_msgs2_.size();

  frameCount3 = pointcloud_msgs3_.size();
  frameSPCount3 = pointcloud_sp_msgs3_.size();

  frameCount4 = pointcloud_msgs4_.size();
  frameSPCount4 = pointcloud_sp_msgs4_.size();


  if(frameCount0>0)
    empty_msgs_.push_back(pointcloud_msgs0_[0]);
  else if(frameCount1>0)
    empty_msgs_.push_back(pointcloud_msgs1_[0]);
  else if(frameCount2>0)
    empty_msgs_.push_back(pointcloud_msgs2_[0]);
  else if(frameCount3>0)
    empty_msgs_.push_back(pointcloud_msgs3_[0]);
  else if(frameCount4>0)
    empty_msgs_.push_back(pointcloud_msgs4_[0]);
    
  for(int i=0;i<MAX_TOPIC_NUM;i++)  msg_flags_.push_back(-1);
}

int BagReader::getMainRadarPclSize()
{
  int result = 0;

  switch (mainRadarIndex_)
  {

    case 0:
    if(!bPlaySPFlag_)
      result = pointcloud_msgs0_.size();
    else
      result = pointcloud_sp_msgs0_.size();
    break;

  case 1:
    if(!bPlaySPFlag_)
      result = pointcloud_msgs1_.size();
    else
      result = pointcloud_sp_msgs1_.size();
    break;

  case 2:
    if(!bPlaySPFlag_)
      result = pointcloud_msgs2_.size();
    else
      result = pointcloud_sp_msgs2_.size();
    break;

  case 3:
    if(!bPlaySPFlag_)
      result = pointcloud_msgs3_.size();
    else
      result = pointcloud_sp_msgs3_.size();
    break;
    
  case 4:
  default:
    if(!bPlaySPFlag_)
      result = pointcloud_msgs4_.size();
    else
      result = pointcloud_sp_msgs4_.size();
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
    if(!bPlaySPFlag_)
      result = pointcloud_msgs0_[curIdx].getTime();
    else
      result = pointcloud_sp_msgs0_[curIdx].getTime();
    break;

  case 1:
    if(!bPlaySPFlag_)
      result = pointcloud_msgs1_[curIdx].getTime();
    else
      result = pointcloud_sp_msgs1_[curIdx].getTime();
    break;

  case 2:
    if(!bPlaySPFlag_)
      result = pointcloud_msgs2_[curIdx].getTime();
    else
      result = pointcloud_sp_msgs2_[curIdx].getTime();
    break;

  case 3:
    if(!bPlaySPFlag_)
      result = pointcloud_msgs3_[curIdx].getTime();
    else
      result = pointcloud_sp_msgs3_[curIdx].getTime();
    break;

  case 4:
  default:
    if(!bPlaySPFlag_)
      result = pointcloud_msgs4_[curIdx].getTime();
    else
      result = pointcloud_sp_msgs4_[curIdx].getTime();
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
          if(!bPlaySPFlag_)
            frame_msgs_.push_back(pointcloud_msgs0_[msg_flags_[i]]);
          else
            frame_msgs_.push_back(pointcloud_sp_msgs0_[msg_flags_[i]]);
          break;

        case 1:
          if(!bPlaySPFlag_)
            frame_msgs_.push_back(pointcloud_msgs1_[msg_flags_[i]]);
          else
            frame_msgs_.push_back(pointcloud_sp_msgs1_[msg_flags_[i]]);
          break;

        case 2:
          if(!bPlaySPFlag_)
            frame_msgs_.push_back(pointcloud_msgs2_[msg_flags_[i]]);
          else
            frame_msgs_.push_back(pointcloud_sp_msgs2_[msg_flags_[i]]);
          break;

        case 3:
          if(!bPlaySPFlag_)
            frame_msgs_.push_back(pointcloud_msgs3_[msg_flags_[i]]);  
          else
            frame_msgs_.push_back(pointcloud_sp_msgs3_[msg_flags_[i]]);
          break;

        case 4:
          if(!bPlaySPFlag_)
            frame_msgs_.push_back(pointcloud_msgs4_[msg_flags_[i]]);
          else
            frame_msgs_.push_back(pointcloud_sp_msgs4_[msg_flags_[i]]);
          break;

        case 6:
          frame_msgs_.push_back(camera_msgs0_[msg_flags_[i]]);
          break;

        case 7:
          frame_msgs_.push_back(camera_msgs1_[msg_flags_[i]]);//camera 1
          break;

        case 8:
          frame_msgs_.push_back(camera_msgs2_[msg_flags_[i]]);
          break;

        case 9:
          frame_msgs_.push_back(camera_msgs3_[msg_flags_[i]]);
          break;

        case 10:
          frame_msgs_.push_back(camera_msgs4_[msg_flags_[i]]);
          break;

        case 11:
          frame_msgs_.push_back(camera_msgs5_[msg_flags_[i]]);
          break;

        case 12:
          //car
          frame_msgs_.push_back(car_msgs_[msg_flags_[i]]);
          break;

        case 13:
          //IMU_msgs_
          frame_msgs_.push_back(IMU_msgs_[msg_flags_[i]]);
          break;

        case 14:
          //Object_msgs
          frame_msgs_.push_back(Object_tag_msgs_1[msg_flags_[i]]);
          break;

        case 15:
          frame_msgs_.push_back(Object_tag_msgs_2[msg_flags_[i]]);
          break;

        case 16:
          frame_msgs_.push_back(Object_tag_msgs_3[msg_flags_[i]]);
          break;

        case 17:
          frame_msgs_.push_back(Object_tag_msgs_4[msg_flags_[i]]);
          break;

        case 18:
          frame_msgs_.push_back(Object_msgs_1[msg_flags_[i]]);
          break;
        
        case 19:
          frame_msgs_.push_back(Object_msgs_2[msg_flags_[i]]);
          break;

        case 20:
          frame_msgs_.push_back(Object_msgs_3[msg_flags_[i]]);
          break;

        case 21:
          frame_msgs_.push_back(Object_msgs_4[msg_flags_[i]]);
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

      msg_flags_[0] = findClosestPtFrame(selected_time,pointcloud_msgs0_,pointcloud_sp_msgs0_);
      msg_flags_[1] = findClosestPtFrame(selected_time,pointcloud_msgs1_,pointcloud_sp_msgs1_);
      msg_flags_[2] = findClosestPtFrame(selected_time,pointcloud_msgs2_,pointcloud_sp_msgs2_);
      msg_flags_[3] = findClosestPtFrame(selected_time,pointcloud_msgs3_,pointcloud_sp_msgs3_);
      msg_flags_[4] = findClosestPtFrame(selected_time,pointcloud_msgs4_,pointcloud_sp_msgs4_);



      msg_flags_[12]= findClosestCarFrame(selected_time);

      msg_flags_[13] = findClosestFrame(selected_time,IMU_msgs_);

      

      msg_flags_[14] = findClosestFrame(selected_time,Object_tag_msgs_1);
      msg_flags_[15] = findClosestFrame(selected_time,Object_tag_msgs_2); 
      msg_flags_[16] = findClosestFrame(selected_time,Object_tag_msgs_3); 
      msg_flags_[17] = findClosestFrame(selected_time,Object_tag_msgs_4);

      msg_flags_[6] = findClosestCameraFrame(selected_time,camera_msgs0_);
      msg_flags_[7] = findClosestCameraFrame(selected_time,camera_msgs1_);
      msg_flags_[8]= findClosestCameraFrame(selected_time,camera_msgs2_);
      msg_flags_[9] = findClosestCameraFrame(selected_time,camera_msgs3_);
      msg_flags_[10] = findClosestCameraFrame(selected_time,camera_msgs4_);
      msg_flags_[11] = findClosestCameraFrame(selected_time,camera_msgs5_);

      msg_flags_[18] = findClosestObjFrame(selected_time,Object_msgs_1);
      msg_flags_[19] = findClosestObjFrame(selected_time,Object_msgs_2);
      msg_flags_[20] = findClosestObjFrame(selected_time,Object_msgs_3);
      msg_flags_[21] = findClosestObjFrame(selected_time,Object_msgs_4);

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
        {
            std::lock_guard<std::mutex> lock(mutex_);
            finishProcessFlag_ = true;
            playing_ = false;
        }
        cv_.notify_all();  // 通知所有等待的线程
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

int BagReader::findClosestObjFrame(const ros::Time& selected_time, std::vector<rosbag::MessageInstance>& Obj_msgs)
{
    int closest_index = -1;
    double min_diff = std::numeric_limits<double>::max();

    for (size_t i = 0; i < Obj_msgs.size(); ++i) {
        ros::Time camera_time = Obj_msgs[i].getTime();
        double time_diff = std::abs((selected_time - camera_time).toSec());

        if (time_diff < min_diff) {
            min_diff = time_diff;
            closest_index = i;
        }
    }

    return closest_index;
}


int BagReader::findClosestFrame(const ros::Time& selected_time, std::vector<rosbag::MessageInstance>& c_msgs)
{
    int closest_index = -1;
    double min_diff = std::numeric_limits<double>::max();

    for (size_t i = 0; i < c_msgs.size(); ++i) {
        ros::Time camera_time = c_msgs[i].getTime();
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

int BagReader::findClosestPtFrame(const ros::Time& selected_time, std::vector<rosbag::MessageInstance>& pcl_msgs, std::vector<rosbag::MessageInstance>& pcl_sp_msgs)
{
    int closest_index = -1;
    double min_diff = std::numeric_limits<double>::max();

    auto& auto_msgs = bPlaySPFlag_ ? pcl_sp_msgs : pcl_msgs;

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
    
    //std::lock_guard<std::mutex> lock(mutex_);
     std::unique_lock<std::mutex> lock(mutex_);
    if (!playing_) 
        break;
    
    current_frame_ = i;

    ros::Time selected_time = getMainRadarTime(current_frame_);

    msg_flags_[0] = findClosestPtFrame(selected_time,pointcloud_msgs0_,pointcloud_sp_msgs0_);
    msg_flags_[1] = findClosestPtFrame(selected_time,pointcloud_msgs1_,pointcloud_sp_msgs1_);
    msg_flags_[2] = findClosestPtFrame(selected_time,pointcloud_msgs2_,pointcloud_sp_msgs2_);
    msg_flags_[3] = findClosestPtFrame(selected_time,pointcloud_msgs3_,pointcloud_sp_msgs3_);
    msg_flags_[4] = findClosestPtFrame(selected_time,pointcloud_msgs4_,pointcloud_sp_msgs4_);

    msg_flags_[6] = findClosestCameraFrame(selected_time,camera_msgs0_);
    msg_flags_[7] = findClosestCameraFrame(selected_time,camera_msgs1_);
    msg_flags_[8]= findClosestCameraFrame(selected_time,camera_msgs2_);
    msg_flags_[9] = findClosestCameraFrame(selected_time,camera_msgs3_);
    msg_flags_[10] = findClosestCameraFrame(selected_time,camera_msgs4_);
    msg_flags_[11] = findClosestCameraFrame(selected_time,camera_msgs5_);

    msg_flags_[12]= findClosestCarFrame(selected_time);

    msg_flags_[13] = findClosestFrame(selected_time,IMU_msgs_);

    msg_flags_[14] = findClosestFrame(selected_time,Object_tag_msgs_1);
    msg_flags_[15] = findClosestFrame(selected_time,Object_tag_msgs_2); 
    msg_flags_[16] = findClosestFrame(selected_time,Object_tag_msgs_3); 
    msg_flags_[17] = findClosestFrame(selected_time,Object_tag_msgs_4);
    
    msg_flags_[18] = findClosestObjFrame(selected_time,Object_msgs_1);
    msg_flags_[19] = findClosestObjFrame(selected_time,Object_msgs_2);
    msg_flags_[20] = findClosestObjFrame(selected_time,Object_msgs_3);
    msg_flags_[21] = findClosestObjFrame(selected_time,Object_msgs_4);

    packetCallbackMsg();
    finishProcessFlag_ = false;

      auto temp_callback = message_callback_;
    lock.unlock();
    
    if (temp_callback) 
        temp_callback(frame_msgs_, current_frame_, msg_flags_);
    

    ROS_INFO("Playing frame %d", current_frame_);
    std::this_thread::sleep_for(std::chrono::milliseconds((int)(50 / play_rate_)));//ms/1s
  }
}

void BagReader::setFinishProcessFlag(bool flag)
{
  finishProcessFlag_ = flag;
}

void BagReader::setSPFlag(bool flag)
{
  bPlaySPFlag_ = flag;
}

void BagReader::selectMainRadar(int index)
{
  mainRadarIndex_ = index;
}

} // namespace my_rviz_plugin
