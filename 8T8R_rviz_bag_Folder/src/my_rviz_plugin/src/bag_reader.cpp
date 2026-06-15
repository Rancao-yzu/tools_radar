#include "my_rviz_plugin/bag_reader.h"
#include <rosbag/view.h>
#include <chrono>
#include <thread>

namespace my_rviz_plugin
{

BagReader::BagReader() : current_frame_(0), play_rate_(1.0), playing_(false), finishProcessFlag_(true), bPlayFloatData_(false), bPlaySPData_(true), mainRadarIndex_(0)  
{   }

BagReader::~BagReader()// 析构函数，确保播放线程安全退出，并关闭bag文件
{
  if (playing_)
  {
    playing_ = false;
    if (play_thread_.joinable())  play_thread_.join();
  }
  bag_.close();
}

void BagReader::readBagFile// 读取bag文件，并按话题分类存储消息；同时更新进度条
(const std::string& file_path, int& frameCount0, int& frameSPCount0,int& frameCount1,int& frameSPCount1, int& frameCount2, 
  int& frameSPCount2,int& frameCount3, int& frameSPCount3,int& frameCount4, int& frameSPCount4)
{
  //进度条更新回调函数，传入进度值
  if(update_progress_bar_callback_) update_progress_bar_callback_(0.05f);

  stopBag(); 

  if (bag_.isOpen()) {
    ROS_INFO("Closing previous bag file.");
    bag_.close();
  }

  if(update_progress_bar_callback_) update_progress_bar_callback_(0.1f);
  
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

  cube_data_msgs0_.clear();
  cube_data_msgs1_.clear();
  cube_data_msgs2_.clear();
  cube_data_msgs3_.clear();
  cube_data_msgs4_.clear();


  empty_msgs_.clear();
  msg_flags_.clear();

  bag_.open(file_path, rosbag::bagmode::Read);// 打开新的bag文件

  if(update_progress_bar_callback_) update_progress_bar_callback_(0.15f);

  // 只读取指定的话题
  std::vector<std::string> topics = {
    "/wf/front_radar/parsed/float_data_0",
    "/wf/front_radar/parsed/float_data_1",
    "/wf/front_radar/parsed/float_data_2",
    "/wf/front_radar/parsed/float_data_3",
    "/wf/front_radar/parsed/float_data_4",
    "/wf/frame_rd_data/ti/radar_0",
    "/wf/frame_rd_data/ti/radar_1",
    "/wf/frame_rd_data/ti/radar_2",
    "/wf/frame_rd_data/ti/radar_3",
    "/wf/frame_rd_data/ti/radar_4",
    "/cv_camera_0/image_raw/compressed",
    "/cv_camera_1/image_raw/compressed",
    "/cv_camera_2/image_raw/compressed",
    "/cv_camera_3/image_raw/compressed",
    "/cv_camera_4/image_raw/compressed",
    "/cv_camera_5/image_raw/compressed",
    "/wf/car_id6/parsed2",
    "/wf/front_radar/ti/cube_data_0",
    "/wf/front_radar/ti/cube_data_1",
    "/wf/front_radar/ti/cube_data_2",
    "/wf/front_radar/ti/cube_data_3",
    "/wf/front_radar/ti/cube_data_4",
  };

  rosbag::View view(bag_, rosbag::TopicQuery(topics));//创建消息视图

  if(update_progress_bar_callback_)
    update_progress_bar_callback_(0.2f);
  
  size_t count = 0;
  size_t totalCount = view.size();

  for (const auto& msg : view)  // 遍历读取的消息，按话题分类存储
  {
    if (msg.getTopic() == "/wf/car_id6/parsed2")  car_msgs_.push_back(msg);
    else if (msg.getTopic() == "/wf/front_radar/parsed/float_data_0") pointcloud_msgs0_.push_back(msg);
    else if (msg.getTopic() == "/wf/front_radar/parsed/float_data_1") pointcloud_msgs1_.push_back(msg);
    else if (msg.getTopic() == "/wf/front_radar/parsed/float_data_2") pointcloud_msgs2_.push_back(msg);
    else if (msg.getTopic() == "/wf/front_radar/parsed/float_data_3") pointcloud_msgs3_.push_back(msg);
    else if (msg.getTopic() == "/wf/front_radar/parsed/float_data_4") pointcloud_msgs4_.push_back(msg);
    else if (msg.getTopic() == "/wf/frame_rd_data/ti/radar_0") pointcloud_sp_msgs0_.push_back(msg);
    else if (msg.getTopic() == "/wf/frame_rd_data/ti/radar_1") pointcloud_sp_msgs1_.push_back(msg);
    else if (msg.getTopic() == "/wf/frame_rd_data/ti/radar_2") pointcloud_sp_msgs2_.push_back(msg);
    else if (msg.getTopic() == "/wf/frame_rd_data/ti/radar_3") pointcloud_sp_msgs3_.push_back(msg);
    else if (msg.getTopic() == "/wf/frame_rd_data/ti/radar_4") pointcloud_sp_msgs4_.push_back(msg);
    else if (msg.getTopic() == "/cv_camera_0/image_raw/compressed") camera_msgs0_.push_back(msg);
    else if (msg.getTopic() == "/cv_camera_1/image_raw/compressed") camera_msgs1_.push_back(msg);
    else if (msg.getTopic() == "/cv_camera_2/image_raw/compressed") camera_msgs2_.push_back(msg);
    else if (msg.getTopic() == "/cv_camera_3/image_raw/compressed") camera_msgs3_.push_back(msg);
    else if (msg.getTopic() == "/cv_camera_4/image_raw/compressed") camera_msgs4_.push_back(msg);
    else if (msg.getTopic() == "/cv_camera_5/image_raw/compressed") camera_msgs5_.push_back(msg);
    else if (msg.getTopic() == "/wf/front_radar/ti/cube_data_0") cube_data_msgs0_.push_back(msg);
    else if (msg.getTopic() == "/wf/front_radar/ti/cube_data_1") cube_data_msgs1_.push_back(msg);
    else if (msg.getTopic() == "/wf/front_radar/ti/cube_data_2") cube_data_msgs2_.push_back(msg);
    else if (msg.getTopic() == "/wf/front_radar/ti/cube_data_3") cube_data_msgs3_.push_back(msg);
    else if (msg.getTopic() == "/wf/front_radar/ti/cube_data_4") cube_data_msgs4_.push_back(msg);

    count++;
    if(update_progress_bar_callback_){
      update_progress_bar_callback_(((float)count)/((float)totalCount) + 0.2);
    }
  }
 
  current_frame_ = 0; // 初始化当前帧为 0

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

   // 若无有效点云数据，则放入一个空消息作为默认值
  if(frameCount0>0) empty_msgs_.push_back(pointcloud_msgs0_[0]);
  else if(frameCount1>0)  empty_msgs_.push_back(pointcloud_msgs1_[0]);
  else if(frameCount2>0)  empty_msgs_.push_back(pointcloud_msgs2_[0]);
  else if(frameCount3>0)  empty_msgs_.push_back(pointcloud_msgs3_[0]);
  else if(frameCount4>0)  empty_msgs_.push_back(pointcloud_msgs4_[0]);
      
  // 初始化消息标志位
  for(int i=0;i<MAX_TOPIC_NUM;i++)  msg_flags_.push_back(-1);
}

int BagReader::getMainRadarPclSize()// 获取主雷达点云数量
{
  int result = 0;
  // 优先用 float_data 做帧率基准, float 未勾选时用 SP
  bool useFloat = bPlayFloatData_ || !bPlaySPData_;

  switch (mainRadarIndex_)
  {
  case 0:
    result = useFloat ? pointcloud_msgs0_.size() : pointcloud_sp_msgs0_.size();
    break;

  case 1:
    result = useFloat ? pointcloud_msgs1_.size() : pointcloud_sp_msgs1_.size();
    break;

  case 2:
    result = useFloat ? pointcloud_msgs2_.size() : pointcloud_sp_msgs2_.size();
    break;

  case 3:
    result = useFloat ? pointcloud_msgs3_.size() : pointcloud_sp_msgs3_.size();
    break;

  case 4:
  default:
    result = useFloat ? pointcloud_msgs4_.size() : pointcloud_sp_msgs4_.size();
    break;
  }

  return result;
}

ros::Time BagReader::getMainRadarTime(int curIdx)
{
  ros::Time result;
  bool useFloat = bPlayFloatData_ || !bPlaySPData_;

  switch (mainRadarIndex_)
  {
  case 0:
    result = useFloat ? pointcloud_msgs0_[curIdx].getTime() : pointcloud_sp_msgs0_[curIdx].getTime();
    break;

  case 1:
    result = useFloat ? pointcloud_msgs1_[curIdx].getTime() : pointcloud_sp_msgs1_[curIdx].getTime();
    break;

  case 2:
    result = useFloat ? pointcloud_msgs2_[curIdx].getTime() : pointcloud_sp_msgs2_[curIdx].getTime();
    break;

  case 3:
    result = useFloat ? pointcloud_msgs3_[curIdx].getTime() : pointcloud_sp_msgs3_[curIdx].getTime();
    break;

  case 4:
  default:
    result = useFloat ? pointcloud_msgs4_[curIdx].getTime() : pointcloud_sp_msgs4_[curIdx].getTime();
    break;
  }

  return result;
}

void BagReader::packetCallbackMsg()
{
  // frame_msgs_ 布局: [0-4]float_data [5-9]SP_data [10-15]camera [16]car [17-21]cube_data
  frame_msgs_.clear();
  for(int i=0;i<MAX_TOPIC_NUM;i++)
  {
    if(msg_flags_[i]<0)//当某个话题没有对应时间戳的消息时，用作提供默认消息
    {
      frame_msgs_.push_back(empty_msgs_[0]);
    }
    else
    {
      switch (i)
      {
        // float_data 0~4
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

        // SP_data 0~4
        case 5:
          frame_msgs_.push_back(pointcloud_sp_msgs0_[msg_flags_[i]]);
          break;
        case 6:
          frame_msgs_.push_back(pointcloud_sp_msgs1_[msg_flags_[i]]);
          break;
        case 7:
          frame_msgs_.push_back(pointcloud_sp_msgs2_[msg_flags_[i]]);
          break;
        case 8:
          frame_msgs_.push_back(pointcloud_sp_msgs3_[msg_flags_[i]]);
          break;
        case 9:
          frame_msgs_.push_back(pointcloud_sp_msgs4_[msg_flags_[i]]);
          break;

        // camera 0~5
        case 10:
          frame_msgs_.push_back(camera_msgs0_[msg_flags_[i]]);
          break;
        case 11:
          frame_msgs_.push_back(camera_msgs1_[msg_flags_[i]]);
          break;
        case 12:
          frame_msgs_.push_back(camera_msgs2_[msg_flags_[i]]);
          break;
        case 13:
          frame_msgs_.push_back(camera_msgs3_[msg_flags_[i]]);
          break;
        case 14:
          frame_msgs_.push_back(camera_msgs4_[msg_flags_[i]]);
          break;
        case 15:
          frame_msgs_.push_back(camera_msgs5_[msg_flags_[i]]);
          break;

        case 16:
          frame_msgs_.push_back(car_msgs_[msg_flags_[i]]);
          break;

        // cube_data 0~4
        case 17:
          frame_msgs_.push_back(cube_data_msgs0_[msg_flags_[i]]);
          break;
        case 18:
          frame_msgs_.push_back(cube_data_msgs1_[msg_flags_[i]]);
          break;
        case 19:
          frame_msgs_.push_back(cube_data_msgs2_[msg_flags_[i]]);
          break;
        case 20:
          frame_msgs_.push_back(cube_data_msgs3_[msg_flags_[i]]);
          break;
        case 21:
          frame_msgs_.push_back(cube_data_msgs4_[msg_flags_[i]]);
          break;
          
        default:  
          break;
      }
    }
  }
}

void BagReader::jumpToFrame(int frame_number)// 跳转到指定帧，并查找匹配的各传感器消息
{
  int mainRadarSize = getMainRadarPclSize();
  
  if (frame_number >= 0 && frame_number < mainRadarSize)
  {
    current_frame_ = frame_number;
    if (message_callback_)
    {
      ros::Time selected_time = getMainRadarTime(current_frame_);

      // float_data 0~4
      msg_flags_[0] = find_Closest_Frame(selected_time,pointcloud_msgs0_);
      msg_flags_[1] = find_Closest_Frame(selected_time,pointcloud_msgs1_);
      msg_flags_[2] = find_Closest_Frame(selected_time,pointcloud_msgs2_);
      msg_flags_[3] = find_Closest_Frame(selected_time,pointcloud_msgs3_);
      msg_flags_[4] = find_Closest_Frame(selected_time,pointcloud_msgs4_);

      // SP_data 0~4
      msg_flags_[5] = find_Closest_Frame(selected_time,pointcloud_sp_msgs0_);
      msg_flags_[6] = find_Closest_Frame(selected_time,pointcloud_sp_msgs1_);
      msg_flags_[7] = find_Closest_Frame(selected_time,pointcloud_sp_msgs2_);
      msg_flags_[8] = find_Closest_Frame(selected_time,pointcloud_sp_msgs3_);
      msg_flags_[9] = find_Closest_Frame(selected_time,pointcloud_sp_msgs4_);

      // camera 0~5
      msg_flags_[10] = find_Closest_Frame(selected_time,camera_msgs0_);
      msg_flags_[11] = find_Closest_Frame(selected_time,camera_msgs1_);
      msg_flags_[12] = find_Closest_Frame(selected_time,camera_msgs2_);
      msg_flags_[13] = find_Closest_Frame(selected_time,camera_msgs3_);
      msg_flags_[14] = find_Closest_Frame(selected_time,camera_msgs4_);
      msg_flags_[15] = find_Closest_Frame(selected_time,camera_msgs5_);

      msg_flags_[16]= find_Closest_Frame(selected_time,car_msgs_);

      // cube_data 0~4
      msg_flags_[17]= find_Closest_Frame(selected_time,cube_data_msgs0_);
      msg_flags_[18]= find_Closest_Frame(selected_time,cube_data_msgs1_);
      msg_flags_[19]= find_Closest_Frame(selected_time,cube_data_msgs2_);
      msg_flags_[20]= find_Closest_Frame(selected_time,cube_data_msgs3_);
      msg_flags_[21]= find_Closest_Frame(selected_time,cube_data_msgs4_);

      packetCallbackMsg();// 获取当前帧消息
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
      if (play_thread_.joinable()) 
        play_thread_.join();
  }
}

void BagReader::setPlayRate(double rate)
{
  play_rate_ = rate;
}

void BagReader::setMessageCallback(MessageCallback callback)
{
  message_callback_ = callback;
}

void BagReader::setUpdateProgressBarCallback(UpdateProgressBarCallback callback)
{
  update_progress_bar_callback_ = callback;
}

int BagReader::getCurrentFrame() const
{
  return current_frame_;
}

int BagReader::find_Closest_Frame(const ros::Time& selected_time, std::vector<rosbag::MessageInstance>& c_msgs)
{
    int closest_index = -1;
    double min_diff = std::numeric_limits<double>::max();

    for (size_t i = 0; i < c_msgs.size(); ++i) {
        ros::Time c_msgs_time = c_msgs[i].getTime();
        double time_diff = std::abs((selected_time - c_msgs_time).toSec());

        if (time_diff <= min_diff) {
            min_diff = time_diff;
            closest_index = i;
        }
    }

    return closest_index;
}

void BagReader::setFloatDataFlag(bool flag)
{
  bPlayFloatData_ = flag;
}

void BagReader::setSPDataFlag(bool flag)
{
  bPlaySPData_ = flag;
}

void BagReader::playLoop() 
{
  int mainRadarSize = getMainRadarPclSize();
  if (current_frame_ != 0)  current_frame_++; //2025/9/17

  ROS_INFO("=========== playLoop  ==========");

  for (size_t i = current_frame_; i < mainRadarSize; ++i) 
  {
    while(!finishProcessFlag_)
      std::this_thread::sleep_for(std::chrono::milliseconds(1));
    
    std::lock_guard<std::mutex> lock(mutex_);

    if (!playing_) 
      break;

    current_frame_ = i;
    
    ros::Time selected_time = getMainRadarTime(current_frame_);
    ROS_INFO("Set current_frame_ to %d, Selected time: %f", current_frame_,selected_time.toSec());

    msg_flags_[0] = find_Closest_Frame(selected_time,pointcloud_msgs0_);
    msg_flags_[1] = find_Closest_Frame(selected_time,pointcloud_msgs1_);
    msg_flags_[2] = find_Closest_Frame(selected_time,pointcloud_msgs2_);
    msg_flags_[3] = find_Closest_Frame(selected_time,pointcloud_msgs3_);
    msg_flags_[4] = find_Closest_Frame(selected_time,pointcloud_msgs4_);

    msg_flags_[5] = find_Closest_Frame(selected_time,pointcloud_sp_msgs0_);
    msg_flags_[6] = find_Closest_Frame(selected_time,pointcloud_sp_msgs1_);
    msg_flags_[7] = find_Closest_Frame(selected_time,pointcloud_sp_msgs2_);
    msg_flags_[8] = find_Closest_Frame(selected_time,pointcloud_sp_msgs3_);
    msg_flags_[9] = find_Closest_Frame(selected_time,pointcloud_sp_msgs4_);

    msg_flags_[10] = find_Closest_Frame(selected_time,camera_msgs0_);
    msg_flags_[11] = find_Closest_Frame(selected_time,camera_msgs1_);
    msg_flags_[12] = find_Closest_Frame(selected_time,camera_msgs2_);
    msg_flags_[13] = find_Closest_Frame(selected_time,camera_msgs3_);
    msg_flags_[14] = find_Closest_Frame(selected_time,camera_msgs4_);
    msg_flags_[15] = find_Closest_Frame(selected_time,camera_msgs5_);

    msg_flags_[16]= find_Closest_Frame(selected_time,car_msgs_);

    msg_flags_[17]= find_Closest_Frame(selected_time,cube_data_msgs0_);
    msg_flags_[18]= find_Closest_Frame(selected_time,cube_data_msgs1_);
    msg_flags_[19]= find_Closest_Frame(selected_time,cube_data_msgs2_);
    msg_flags_[20]= find_Closest_Frame(selected_time,cube_data_msgs3_);
    msg_flags_[21]= find_Closest_Frame(selected_time,cube_data_msgs4_);

    packetCallbackMsg();
    finishProcessFlag_ = false;
    message_callback_(frame_msgs_,current_frame_,msg_flags_);
    ROS_INFO("!-Playing frame %d", current_frame_);
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
