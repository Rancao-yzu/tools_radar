#include "my_rviz_plugin/bag_reader.h"
#include <rosbag/view.h>
#include <chrono>
#include <thread>

namespace my_rviz_plugin
{

// INTERVAL_SEC/区间(雷达66ms±2ms,IMU10ms±2ms),区间内各topic的所有msg全部发布
const double BagReader::INTERVAL_SEC = 0.008;

BagReader::BagReader() : current_frame_(0), play_rate_(1.0), playing_(false), finishProcessFlag_(true), bPlaySPFlag_(false), mainRadarIndex_(3), interval_count_(0)
{   }

BagReader::~BagReader()// 析构函数，确保播放线程安全退出，并关闭bag文件
{
  stopBag();  // 若正在播放则停止并join;若已结束则playing_已为false
  // 兜底:播放自然结束后 playing_=false 但 play_thread_ 仍 joinable ,必须join否则std::thread析构会std::terminate
  if (play_thread_.joinable())  play_thread_.join();
  bag_.close();
}

void BagReader::readBagFile// 读取bag文件，并按话题分类存储消息；同时更新进度条
(const std::string& file_path, int& frameCount0,int& frameCount1, int& frameCount2,int& frameCount3,int& frameCount4)
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

  IMU_msgs_.clear();
  corner_radar_warning_msgs_.clear();

  bag_.open(file_path, rosbag::bagmode::Read);// 打开新的bag文件

  if(update_progress_bar_callback_) update_progress_bar_callback_(0.15f);

  // 只读取指定的话题
  std::vector<std::string> topics = {"/wf/corner_radar/lgu_data_0","/wf/corner_radar/lgu_data_1", "/wf/corner_radar/lgu_data_2", 
    "/wf/corner_radar/lgu_data_3", "/wf/corner_radar/lgu_data_4", 
    "/cv_camera_0/image_raw/compressed","/cv_camera_1/image_raw/compressed", "/cv_camera_2/image_raw/compressed",
    "/cv_camera_3/image_raw/compressed", "/cv_camera_4/image_raw/compressed","/cv_camera_5/image_raw/compressed",
    "/wf/car_id6/parsed2","/corner_radar/warning_status",
    "/wf/imu_data/parsed"
  };

  rosbag::View view(bag_, rosbag::TopicQuery(topics));//创建消息视图

  if(update_progress_bar_callback_)
    update_progress_bar_callback_(0.2f);
  
  size_t count = 0;
  size_t totalCount = view.size();

  for (const auto& msg : view)
  {
    if (msg.getTopic() == "/wf/car_id6/parsed2"){car_msgs_.push_back(msg);}
    else if (msg.getTopic() == "/wf/corner_radar/lgu_data_0"){pointcloud_msgs0_.push_back(msg);}
    else if (msg.getTopic() == "/wf/corner_radar/lgu_data_1"){pointcloud_msgs1_.push_back(msg);}
    else if (msg.getTopic() == "/wf/corner_radar/lgu_data_2"){pointcloud_msgs2_.push_back(msg);}
    else if (msg.getTopic() == "/wf/corner_radar/lgu_data_3"){pointcloud_msgs3_.push_back(msg);}
    else if (msg.getTopic() == "/wf/corner_radar/lgu_data_4"){pointcloud_msgs4_.push_back(msg);}
    else if (msg.getTopic() == "/corner_radar/warning_status"){corner_radar_warning_msgs_.push_back(msg);}
    else if (msg.getTopic() == "/cv_camera_0/image_raw/compressed"){camera_msgs0_.push_back(msg);}
    else if (msg.getTopic() == "/cv_camera_1/image_raw/compressed"){camera_msgs1_.push_back(msg);}
    else if (msg.getTopic() == "/cv_camera_2/image_raw/compressed"){camera_msgs2_.push_back(msg);}
    else if (msg.getTopic() == "/cv_camera_3/image_raw/compressed"){camera_msgs3_.push_back(msg);}
    else if (msg.getTopic() == "/cv_camera_4/image_raw/compressed"){camera_msgs4_.push_back(msg);}
    else if (msg.getTopic() == "/cv_camera_5/image_raw/compressed"){camera_msgs5_.push_back(msg);}
    else if(msg.getTopic() == "/wf/imu_data/parsed") {IMU_msgs_.push_back(msg);}

    count++;
    if(update_progress_bar_callback_){
      update_progress_bar_callback_(((float)count)/((float)totalCount) + 0.2);
    }
  }
 
  current_frame_ = 0; // 初始化当前区间为 0

  frameCount0 = pointcloud_msgs0_.size();
  frameCount1 = pointcloud_msgs1_.size();
  frameCount2 = pointcloud_msgs2_.size();
  frameCount3 = pointcloud_msgs3_.size();
  frameCount4 = pointcloud_msgs4_.size();

  // 添加调试信息，检查GT数据是否存在
  ROS_INFO(" pointcloud_msgs: %u", frameCount3);

  // 以主雷达首帧时间为T0,按INTERVAL_SEC切片,计算区间总数
  recomputeIntervals();
}

// 获取主雷达(由mainRadarIndex_/bPlaySPFlag_决定)的消息数量
int BagReader::getMainRadarSize() const
{
  switch (mainRadarIndex_)
  {
  case 0:  return pointcloud_msgs0_.size();
  case 1:  return pointcloud_msgs1_.size();
  case 2:  return pointcloud_msgs2_.size();
  case 3:  return pointcloud_msgs3_.size();
  default: return pointcloud_msgs4_.size();
  }
}

// 主雷达某帧时间(用于确定T0和区间边界)。越界时返回无效时间。
ros::Time BagReader::getMainRadarTime(int curIdx)
{
  ros::Time result;  // 默认无效时间
  int size = getMainRadarSize();
  if (curIdx < 0 || curIdx >= size)
    return result;

  switch (mainRadarIndex_)
  {
    case 0:result = pointcloud_msgs0_[curIdx].getTime();break;
    case 1:result = pointcloud_msgs1_[curIdx].getTime();break;
    case 2: result = pointcloud_msgs2_[curIdx].getTime();break;
    case 3:result = pointcloud_msgs3_[curIdx].getTime();break;
    default:result = pointcloud_msgs4_[curIdx].getTime();break;
  }
  return result;
}

// 以主雷达首帧时间为T0,按INTERVAL_SEC切片,重算区间总数
void BagReader::recomputeIntervals()
{
  int size = getMainRadarSize();
  interval_count_ = 0;
  t0_ = ros::Time(); // 默认无效
  if (size <= 0)
    return;
  t0_ = getMainRadarTime(0);
  if (!t0_.isValid())
    return;
  ros::Time t_end = getMainRadarTime(size - 1);
  double total_sec = (t_end - t0_).toSec();
  if (total_sec > 0)
    interval_count_ = (int)std::ceil(total_sec / INTERVAL_SEC) + 1;
  ROS_INFO("recomputeIntervals: T0=%f, interval_count=%d", t0_.toSec(), interval_count_);
}

int BagReader::getIntervalCount() const
{
  return interval_count_;
}

// 从src中收集时间在[start,end)范围内的所有msg(rosbag view按时间有序,二分定位起点后顺序扫描)
static void collectFromRange(std::vector<rosbag::MessageInstance>& dst,
                             const ros::Time& start, const ros::Time& end,
                             const std::vector<rosbag::MessageInstance>& src)
{
  if (src.empty()) return;
  auto it = std::lower_bound(src.begin(), src.end(), start,
      [](const rosbag::MessageInstance& m, const ros::Time& t){ return m.getTime() < t; });
  while (it != src.end() && it->getTime() < end)
  {
    dst.push_back(*it);
    ++it;
  }
}

// 收集[start,end)时间区间内各topic的所有msg(按topic索引分组,与packetCallbackMsg索引一致)
void BagReader::collectIntervalMessages(const ros::Time& start, const ros::Time& end, IntervalMessages& out)
{
  // rosbag::MessageInstance不可CopyAssignable,故用resize+clear而非assign
  out.resize(MAX_TOPIC_NUM);
  for (auto& v : out) v.clear();
  // idx0-4: 由bPlaySPFlag_决定源(sgu或sp)
  collectFromRange(out[0],  start, end, pointcloud_msgs0_);
  collectFromRange(out[1],  start, end, pointcloud_msgs1_);
  collectFromRange(out[2],  start, end, pointcloud_msgs2_);
  collectFromRange(out[3],  start, end, pointcloud_msgs3_);
  collectFromRange(out[4],  start, end, pointcloud_msgs4_);
  // idx5: warning_status
  collectFromRange(out[5],  start, end, corner_radar_warning_msgs_);
  // idx6-11: cameras 0-5
  collectFromRange(out[6],  start, end, camera_msgs0_);
  collectFromRange(out[7],  start, end, camera_msgs1_);
  collectFromRange(out[8],  start, end, camera_msgs2_);
  collectFromRange(out[9],  start, end, camera_msgs3_);
  collectFromRange(out[10], start, end, camera_msgs4_);
  collectFromRange(out[11], start, end, camera_msgs5_);
  // idx12: car
  collectFromRange(out[12], start, end, car_msgs_);

  collectFromRange(out[13], start, end, IMU_msgs_);
}

void BagReader::jumpToFrame(int interval_index)// 跳转到指定时间区间并发布该区间消息
{
  int total = getIntervalCount();
  if (interval_index < 0 || interval_index >= total)
    return;

  current_frame_ = interval_index;
  if (!t0_.isValid())
    return;

  // 区间[t0+idx*15ms, t0+(idx+1)*15ms)
  ros::Time iv_start = t0_ + ros::Duration((double)interval_index * INTERVAL_SEC);
  ros::Time iv_end   = t0_ + ros::Duration((double)(interval_index + 1) * INTERVAL_SEC);
  collectIntervalMessages(iv_start, iv_end, interval_msgs_);
  if (message_callback_)
  {
    message_callback_(interval_msgs_, interval_index);
    ROS_INFO("Playing interval %d [%f, %f)", interval_index, iv_start.toSec(), iv_end.toSec());
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
      finishProcessFlag_ = true;  // 唤醒步进阻塞等待,使playLoop退出while循环
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

void BagReader::playLoop()
{
  int total = getIntervalCount();
  if (total <= 0 || !t0_.isValid())
  {
    ROS_WARN("playLoop: no intervals to play (total=%d)", total);
    playing_ = false;
    return;
  }

  // 从当前区间的下一个开始播放(保持原"播放从下一帧开始"的行为)
  if (current_frame_ != 0)  current_frame_++;

  ROS_INFO("=========== playLoop (interval-driven) =========== total=%d start=%d", total, current_frame_);

  for (int iv = current_frame_; iv < total; ++iv)
  {
    std::lock_guard<std::mutex> lock(mutex_);

    if (!playing_)
        break;

    current_frame_ = iv;

    // 区间[t0+iv*INTERVAL_SEC, t0+(iv+1)*INTERVAL_SEC)
    ros::Time iv_start = t0_ + ros::Duration((double)iv * INTERVAL_SEC);
    ros::Time iv_end   = t0_ + ros::Duration((double)(iv + 1) * INTERVAL_SEC);
    collectIntervalMessages(iv_start, iv_end, interval_msgs_);
    finishProcessFlag_ = false;  // 设等待态:仅当本区间含sgu/sp时才真正阻塞
    message_callback_(interval_msgs_, iv);
    //ROS_INFO("!-Playing interval %d [%f, %f)", iv, iv_start.toSec(), iv_end.toSec());

    // 无sgu区间不阻塞,自动推进,避免因算法不调service而卡死
    if (!interval_msgs_[3].empty())
    {
      while(!finishProcessFlag_ && playing_)
        std::this_thread::sleep_for(std::chrono::milliseconds(1));
    }

    std::this_thread::sleep_for(
        std::chrono::milliseconds(static_cast<int>(INTERVAL_SEC * 1000.0 / play_rate_))
    );
  }
  playing_ = false;
}

void BagReader::setFinishProcessFlag(bool flag)
{
  finishProcessFlag_ = flag;
}

void BagReader::setSPFlag(bool flag)
{
  bPlaySPFlag_ = flag;
  // 切换SP源后,T0与区间数会变,需重算并重置当前区间位置
  recomputeIntervals();
  current_frame_ = 0;
}

void BagReader::selectMainRadar(int index)
{
  mainRadarIndex_ = index;
  // 切换主雷达后,T0与区间数会变,需重算并重置当前区间位置
  recomputeIntervals();
  current_frame_ = 0;
}

} // namespace my_rviz_plugin
