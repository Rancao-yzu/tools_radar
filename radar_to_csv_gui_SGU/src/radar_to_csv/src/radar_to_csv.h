#ifndef RADAR_TO_CSV_H
#define RADAR_TO_CSV_H

#include <ros/ros.h>
#include <rosbag/bag.h>
#include <rosbag/view.h>
#include <arbe_msgs/wfSguRawData.h>
#include <fstream>
#include <sstream>
#include <iomanip>
#include <cmath>
#include <map>
#include <vector>
#include <boost/foreach.hpp>
#include <filesystem>
#include <regex>
#include <iostream>

// Qt GUI 头文件
#include <QApplication>
#include <QWidget>
#include <QVBoxLayout>
#include <QHBoxLayout>
#include <QLabel>
#include <QLineEdit>
#include <QPushButton>
#include <QFileDialog>
#include <QMessageBox>
#include <QProgressBar>
#include <QTextEdit>
#include <QCheckBox>
#include <QDateTime>
#include <QDir>
#include <QTimer>  // 添加QTimer头文件
#include <QMetaObject>

namespace fs = std::filesystem;

class RadarToCSV : public QWidget
{
    Q_OBJECT

public:
    RadarToCSV(QWidget *parent = nullptr);

private slots:
    void selectBagFolder();
    void selectOutputFolder();
    void processBags();
    void enableProcessButton();
    void logMessage(const QString &message);

private:
    void setupUI();
    void processBagFolder();  // 添加这个函数声明
    std::string getRadarName(const std::string& topic_name);
    void initCSV(const std::string& topic_name, std::ofstream& csv_file, bool& header_written, const std::string& bag_name);
    void writeToCSV(const arbe_msgs::wfSguRawData::ConstPtr& msg, std::ofstream& csv_file, 
                   bool& header_written, uint32_t& object_counter, const std::string& topic_name);
    void processBag(const std::string& bag_path);

    // GUI 组件  arbe_msgs
    QLineEdit *bagFolderEdit;
    QLineEdit *vehicleIdEdit;
    QLineEdit *outputFolderEdit;
    QLineEdit *outputFolderEdit_Folder;
    QPushButton *processButton;
    QProgressBar *progressBar;
    QTextEdit *logText;

    // ROS 相关
    ros::NodeHandle nh_;
    std::string bag_folder_;
    std::string vehicle_id_;
    bool initialized_;
    std::string output_folder_;
};

#endif // RADAR_TO_CSV_H