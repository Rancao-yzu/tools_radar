#include "my_rviz_plugin/bag_reader.h"
#include <rosbag/view.h>
#include <chrono>
#include <thread>

namespace my_rviz_plugin
{

BagReader::BagReader() : current_frame_(0), play_rate_(1.0), playing_(false), finishProcessFlag_(true)
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

void BagReader::readBagFile(const std::string& file_path, int& radarFrameCount)
{
  if(update_progress_bar_callback_) update_progress_bar_callback_(0.05f);

  stopBag();

  if (bag_.isOpen()) {
    ROS_INFO("Closing previous bag file.");
    bag_.close();
  }

  if(update_progress_bar_callback_) update_progress_bar_callback_(0.1f);

  radar_msgs_.clear();
  motor_msgs_.clear();
  camera_msgs0_.clear();
  camera_msgs1_.clear();
  camera_msgs2_.clear();
  camera_msgs3_.clear();
  camera_msgs4_.clear();
  camera_msgs5_.clear();
  msg_flags_.clear();

  bag_.open(file_path, rosbag::bagmode::Read);

  if(update_progress_bar_callback_) update_progress_bar_callback_(0.15f);

  std::vector<std::string> topics = {
    "/wf/radar/sofa_0",
    "/wf/motor/motor_pub",
    "/cv_camera_0/image_raw/compressed",
    "/cv_camera_1/image_raw/compressed",
    "/cv_camera_2/image_raw/compressed",
    "/cv_camera_3/image_raw/compressed",
    "/cv_camera_4/image_raw/compressed",
    "/cv_camera_5/image_raw/compressed",
  };

  rosbag::View view(bag_, rosbag::TopicQuery(topics));

  if(update_progress_bar_callback_)
    update_progress_bar_callback_(0.2f);

  size_t count = 0;
  size_t totalCount = view.size();

  for (const auto& msg : view)
  {
    if (msg.getTopic() == "/wf/radar/sofa_0")                radar_msgs_.push_back(msg);
    else if (msg.getTopic() == "/wf/motor/motor_pub")        motor_msgs_.push_back(msg);
    else if (msg.getTopic() == "/cv_camera_0/image_raw/compressed") camera_msgs0_.push_back(msg);
    else if (msg.getTopic() == "/cv_camera_1/image_raw/compressed") camera_msgs1_.push_back(msg);
    else if (msg.getTopic() == "/cv_camera_2/image_raw/compressed") camera_msgs2_.push_back(msg);
    else if (msg.getTopic() == "/cv_camera_3/image_raw/compressed") camera_msgs3_.push_back(msg);
    else if (msg.getTopic() == "/cv_camera_4/image_raw/compressed") camera_msgs4_.push_back(msg);
    else if (msg.getTopic() == "/cv_camera_5/image_raw/compressed") camera_msgs5_.push_back(msg);

    count++;
    if(update_progress_bar_callback_){
      update_progress_bar_callback_(((float)count)/((float)totalCount) + 0.2);
    }
  }

  current_frame_ = 0;
  radarFrameCount = radar_msgs_.size();

  // 若无有效雷达数据，无法播放
  if(radarFrameCount == 0)
  {
    ROS_WARN("No radar messages found in bag file.");
  }

  for(int i = 0; i < MAX_TOPIC_NUM; i++)  msg_flags_.push_back(-1);
}

int BagReader::getMainRadarPclSize()
{
  return radar_msgs_.size();
}

ros::Time BagReader::getMainRadarTime(int curIdx)
{
  return radar_msgs_[curIdx].getTime();
}

void BagReader::packetCallbackMsg()
{
  // frame_msgs_ 布局: [0]雷达 [1]电机 [2-7]相机
  frame_msgs_.clear();
  for(int i = 0; i < MAX_TOPIC_NUM; i++)
  {
    if(msg_flags_[i] < 0)
    {
      // 无数据时用雷达首条消息占位，避免空引用
      frame_msgs_.push_back(radar_msgs_[0]);
    }
    else
    {
      switch (i)
      {
        case 0: frame_msgs_.push_back(radar_msgs_[msg_flags_[i]]);     break;
        case 1: frame_msgs_.push_back(motor_msgs_[msg_flags_[i]]);     break;
        case 2: frame_msgs_.push_back(camera_msgs0_[msg_flags_[i]]);   break;
        case 3: frame_msgs_.push_back(camera_msgs1_[msg_flags_[i]]);   break;
        case 4: frame_msgs_.push_back(camera_msgs2_[msg_flags_[i]]);   break;
        case 5: frame_msgs_.push_back(camera_msgs3_[msg_flags_[i]]);   break;
        case 6: frame_msgs_.push_back(camera_msgs4_[msg_flags_[i]]);   break;
        case 7: frame_msgs_.push_back(camera_msgs5_[msg_flags_[i]]);   break;
        default: break;
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

      msg_flags_[0] = find_Closest_Frame(selected_time, radar_msgs_);
      msg_flags_[1] = find_Closest_Frame(selected_time, motor_msgs_);
      msg_flags_[2] = find_Closest_Frame(selected_time, camera_msgs0_);
      msg_flags_[3] = find_Closest_Frame(selected_time, camera_msgs1_);
      msg_flags_[4] = find_Closest_Frame(selected_time, camera_msgs2_);
      msg_flags_[5] = find_Closest_Frame(selected_time, camera_msgs3_);
      msg_flags_[6] = find_Closest_Frame(selected_time, camera_msgs4_);
      msg_flags_[7] = find_Closest_Frame(selected_time, camera_msgs5_);

      packetCallbackMsg();
      message_callback_(frame_msgs_, current_frame_, msg_flags_);
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

void BagReader::playLoop()
{
  int mainRadarSize = getMainRadarPclSize();
  if (current_frame_ != 0)  current_frame_++;

  ROS_INFO("=========== playLoop  ==========");
  ROS_INFO("play_maxindex: %d", mainRadarSize);
  for (size_t i = current_frame_; i < mainRadarSize; ++i)
  {
    while(!finishProcessFlag_)
      std::this_thread::sleep_for(std::chrono::milliseconds(1));

    std::lock_guard<std::mutex> lock(mutex_);

    if (!playing_)
      break;

    current_frame_ = i;

    ros::Time selected_time = getMainRadarTime(current_frame_);
    ROS_INFO("Set current_frame_ to %d, Selected time: %f", current_frame_, selected_time.toSec());

    msg_flags_[0] = find_Closest_Frame(selected_time, radar_msgs_);
    msg_flags_[1] = find_Closest_Frame(selected_time, motor_msgs_);
    msg_flags_[2] = find_Closest_Frame(selected_time, camera_msgs0_);
    msg_flags_[3] = find_Closest_Frame(selected_time, camera_msgs1_);
    msg_flags_[4] = find_Closest_Frame(selected_time, camera_msgs2_);
    msg_flags_[5] = find_Closest_Frame(selected_time, camera_msgs3_);
    msg_flags_[6] = find_Closest_Frame(selected_time, camera_msgs4_);
    msg_flags_[7] = find_Closest_Frame(selected_time, camera_msgs5_);

    packetCallbackMsg();
    finishProcessFlag_ = false;
    message_callback_(frame_msgs_, current_frame_, msg_flags_);
    ROS_INFO("!-Playing frame %d", current_frame_);
    std::this_thread::sleep_for(std::chrono::milliseconds((int)(50 / play_rate_)));
  }
}

void BagReader::setFinishProcessFlag(bool flag)
{
  finishProcessFlag_ = flag;
}

} // namespace my_rviz_plugin
