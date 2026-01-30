/**
 * 
Radar To CSV 工具
├── 1. 用户界面 (UI)
│   ├── GUI 控件（输入框、按钮、日志框、进度条）
│   └── 用户操作：选择文件夹、输入参数、点击处理、查看日志
│
├── 2. 事件与控制逻辑
│   ├── 参数输入校验（Bag路径、车辆编号、输出目录）
│   ├── 处理流程控制（启动、进度、完成、异常处理）
│   └── 日志输出与UI状态管理（如启用/禁用按钮、显示进度）
│
├── 3. Bag 文件处理
│   ├── 扫描指定文件夹中的所有 .bag 文件
│   ├── 遍历每个 Bag 文件，查找雷达相关话题
│   └── 对每个话题，逐条读取雷达数据消息
│
├── 4. 数据筛选与转换
│   ├── 从消息中提取目标对象（如车辆、障碍物）
│   ├── 筛选有效目标（ objID > 0，dynFlg != 0）
│   └── 格式化目标数据（时间、ID、速度、位置等）
│
├── 5. CSV 文件输出
│   ├── 生成带时间戳/车辆/雷达信息的文件名
│   ├── 创建输出目录（ GTs/GT/...）
│   └── 将筛选后的目标数据写入 CSV 文件（每个话题一个文件）
│
└── 6. 程序入口
    └── 启动 Qt 应用与 ROS 节点，显示主窗口
 * 
 */
#include "radar_to_csv.h"

RadarToCSV::RadarToCSV(QWidget *parent) : QWidget(parent), nh_("~"), initialized_(false) {
    setupUI(); // 初始化用户界面
}

void RadarToCSV::setupUI() {
    QVBoxLayout *mainLayout = new QVBoxLayout(this);// 主垂直布局
    
    QHBoxLayout *bagFolderLayout = new QHBoxLayout();
    bagFolderLayout->addWidget(new QLabel("Bag文件文件夹:"));
    bagFolderEdit = new QLineEdit();
    bagFolderLayout->addWidget(bagFolderEdit);
    QPushButton *bagFolderButton = new QPushButton("浏览...");
    connect(bagFolderButton, &QPushButton::clicked, this, &RadarToCSV::selectBagFolder);
    bagFolderLayout->addWidget(bagFolderButton);
    mainLayout->addLayout(bagFolderLayout);
    
    QHBoxLayout *vehicleIdLayout = new QHBoxLayout();
    vehicleIdLayout->addWidget(new QLabel("车辆编号:"));
    vehicleIdEdit = new QLineEdit("F520MR");
    vehicleIdLayout->addWidget(vehicleIdEdit);
    mainLayout->addLayout(vehicleIdLayout);
    
    QHBoxLayout *outputFolderLayout = new QHBoxLayout();
    outputFolderLayout->addWidget(new QLabel("后缀:"));
    outputFolderEdit = new QLineEdit("GT");
    outputFolderLayout->addWidget(outputFolderEdit);
    mainLayout->addLayout(outputFolderLayout);

    QHBoxLayout *outputFolderLayout_Folder = new QHBoxLayout();
    outputFolderLayout_Folder->addWidget(new QLabel("输出文件夹:"));
    outputFolderEdit_Folder = new QLineEdit("2025_10_05");
    outputFolderLayout_Folder->addWidget(outputFolderEdit_Folder);
    mainLayout->addLayout(outputFolderLayout_Folder);
    
    processButton = new QPushButton("开始处理");
    connect(processButton, &QPushButton::clicked, this, &RadarToCSV::processBags);
    mainLayout->addWidget(processButton);
    
    // 进度条
    progressBar = new QProgressBar();
    progressBar->setVisible(false);
    mainLayout->addWidget(progressBar);
    
    logText = new QTextEdit();
    logText->setReadOnly(true);
    logText->setMaximumHeight(1000);
    mainLayout->addWidget(new QLabel("处理日志:"));
    mainLayout->addWidget(logText);
    
    setWindowTitle("雷达数据转CSV工具");
    setMinimumSize(600, 500);
}

void RadarToCSV::selectBagFolder() {
    QString folder = QFileDialog::getExistingDirectory(this, "选择Bag文件文件夹", QDir::homePath());
    if (!folder.isEmpty()) {
        bagFolderEdit->setText(folder);     // 更新路径显示
    }
}

void RadarToCSV::selectOutputFolder() {
    QString folder = QFileDialog::getExistingDirectory(this, "选择输出文件夹", QDir::homePath());
    if (!folder.isEmpty()) {
        outputFolderEdit_Folder->setText(folder);
    }
}

void RadarToCSV::processBags() {        // 点击“开始处理”按钮后的逻辑
    bag_folder_ = bagFolderEdit->text().toStdString();
    vehicle_id_ = vehicleIdEdit->text().toStdString();
    output_folder_ = outputFolderEdit_Folder->text().toStdString();    // 从UI获取输入参数
    
    if (bag_folder_.empty()) {
        QMessageBox::warning(this, "警告", "请选择Bag文件文件夹!");
        return;
    }
    
    if (vehicle_id_.empty()) {
        QMessageBox::warning(this, "警告", "请输入车辆编号！");
        return;
    }
    
    if (output_folder_.empty()) 
        output_folder_ = "GT";
    
    processButton->setEnabled(false); // 禁用处理按钮，防止重复点击
    progressBar->setVisible(true);
    
    QTimer::singleShot(0, this, &RadarToCSV::processBagFolder);// 在主线程异步调用，避免阻塞UI
}

void RadarToCSV::logMessage(const QString &message) {
    logText->append(QDateTime::currentDateTime().toString("hh:mm:ss") + " - " + message);
    QApplication::processEvents(); // 确保界面更新
}

void RadarToCSV::enableProcessButton() {// 处理完成后重新启用按钮，隐藏进度条
    processButton->setEnabled(true);
    progressBar->setVisible(false);
}

void RadarToCSV::processBagFolder() {
    if (!fs::exists(bag_folder_) || !fs::is_directory(bag_folder_)) {
        logMessage(QString("错误: 文件夹路径 %1 不存在或不是目录").arg(QString::fromStdString(bag_folder_)));
        enableProcessButton();
        return;
    }
    
    std::vector<std::string> bag_files;
    for (const auto& entry : fs::directory_iterator(bag_folder_)) {
        if (entry.path().extension() == ".bag") {
            bag_files.push_back(entry.path().string());// 找出所有 .bag 文件
        }
    }
    
    if (bag_files.empty()) {
        logMessage(QString("警告: 在文件夹 %1 中未找到.bag文件").arg(QString::fromStdString(bag_folder_)));
        enableProcessButton();
        return;
    }
    
    progressBar->setRange(0, bag_files.size());
    progressBar->setValue(0);
    
    int processed = 0;
    for (const auto& bag_file : bag_files) {
        logMessage(QString("处理文件: %1").arg(QString::fromStdString(bag_file)));
        
        processBag(bag_file);
        progressBar->setValue(++processed);
        
        QApplication::processEvents(); // 处理UI事件，保持界面响应
    }
    
    logMessage(QString("完成! 共处理 %1 个bag文件").arg(processed));
    enableProcessButton();
}

std::string RadarToCSV::getRadarName(const std::string& topic_name) {
    if (topic_name == "/wf/corner_radar/sgu_data_1") return "WFRAFL";
    if (topic_name == "/wf/corner_radar/sgu_data_2") return "WFRAFR";
    if (topic_name == "/wf/corner_radar/sgu_data_3") return "WFRARL";
    if (topic_name == "/wf/corner_radar/sgu_data_4") return "WFRARR";
    return "UNKNOWN";
}

// 初始化CSV文件，包括生成文件名和创建目录
void RadarToCSV::initCSV(const std::string& topic_name, std::ofstream& csv_file, bool& header_written, const std::string& bag_name) {
    std::string date, time;
    std::regex bag_name_regex(".*_(\\d{4}-\\d{2}-\\d{2})-(\\d{2}-\\d{2}-\\d{2})_\\d+");
    std::smatch match;// 从bag文件名中通过正则提取日期和时间
    if (std::regex_match(bag_name, match, bag_name_regex)) {
        date = match[1].str();
        time = match[2].str();
        date.erase(std::remove(date.begin(), date.end(), '-'), date.end());
        time.erase(std::remove(time.begin(), time.end(), '-'), time.end());
    } else {
        logMessage(QString("警告: 无法从文件名提取日期时间: %1").arg(QString::fromStdString(bag_name)));
        date = "unknown_date";
        time = "unknown_time";
    }

    std::string radar_name = getRadarName(topic_name);
    std::string custom_field = outputFolderEdit->text().toStdString();

    std::string gt_dir = "GTs";                        
    std::string full_output_dir = gt_dir + "/" + output_folder_;  // 文件存储位置

    std::string csv_filename = vehicle_id_ + "_" + radar_name + "_" + date + "_" + time + "_" + custom_field + ".csv";

    if (!output_folder_.empty()) {
        std::error_code ec;
        fs::create_directories(full_output_dir, ec);
        if (ec) {
            logMessage(QString("无法创建目录 '%1': %2").arg(QString::fromStdString(output_folder_)).arg(QString::fromStdString(ec.message())));
            return;
        }
        csv_filename = full_output_dir + "/" + csv_filename;
    }
    
    csv_file.open(csv_filename, std::ios::out);
    if (!csv_file.is_open()) {
        logMessage(QString("无法打开CSV文件: %1").arg(QString::fromStdString(csv_filename)));
        return;
    }
    
    logMessage(QString("创建CSV文件: %1").arg(QString::fromStdString(csv_filename)));
    header_written = false;
}

void RadarToCSV::writeToCSV(const arbe_msgs::wfSguRawData::ConstPtr& msg, std::ofstream& csv_file, 
                bool& header_written, uint32_t& object_counter, const std::string& topic_name) {
    if (!csv_file.is_open()) {
        return;
    }

    if (!header_written) {
        csv_file << "dut_timestamp,dut_id,dut_Unqid,dut_dx,dut_dy,dut_dz,dut_vx,dut_vy,dut_vz,dut_roll,dut_pitch,dut_yaw,dut_ax,dut_ay,dut_az,"
                 << "dut_length,dut_width,dut_height,dut_object_type,dut_reference_point,dut_source_path,dut_existProb,dut_probMoving,dut_probHasBeenObservedMoving,remark,dynFlg,center_dx,center_dy\n";
        header_written = true;
    }

    char timestamp_buf[32];
    snprintf(timestamp_buf, sizeof(timestamp_buf), "%.6f", msg->header.stamp.toSec());
    std::string radar_name = getRadarName(topic_name);

    for (const auto& obj : msg->objects) {
         if (obj.dynFlg != 0 && obj.objID != 0) {
            double heading_in_radians = obj.yawAng * M_PI / 180.0;
            csv_file << timestamp_buf << ","
                    <<static_cast<int>( obj.objID) << ","           // dut_id   
                    <<static_cast<int>( obj.objID) << ","           //dut_Unqid
                    << obj.distX << ","                             // dut_dx             
                    << obj.distY << ","                             // dut_dy             
                    << "-" << ","                                   // dut_dz   
                    << obj.velX << ","                              //dut_vx
                    << obj.velY << ","                              //dut_vy
                    << "-" << ","                                   // dut_vz     
                    << "-" << ","                                   // dut_roll 
                    << "-" << ","                                   // dut_pitch 
                    << heading_in_radians << ","                    //dut_yaw
                    << "-" << ","                                   // dut_ax        
                    << "-" << ","                                   // dut_ay         
                    << "-" << ","                                   // dut_az
                    << obj.length << ","
                    << obj.width << ","
                    << obj.height << ","                            // dut_height
                    << static_cast<int>(obj.objType) << ","
                    << "-" << ","                                   // dut_reference_point 
                    << radar_name << ","
                    << "-" << ","                                   // dut_existProb
                    << "-" << ","                                   // dut_probMoving            
                    << "-" << ","                                   // dut_probHasBeenObservedMoving
                    << "-" << ","                                   // remark
                    << static_cast<int>(obj.dynFlg) << ","          //dynFlg
                    << obj.distX<< ","                              //center_dx
                    << obj.distY << "\n";                           //center_dy
        }
        object_counter++;
    }
}

void RadarToCSV::processBag(const std::string& bag_path) {
    rosbag::Bag bag;
    try {
        bag.open(bag_path, rosbag::bagmode::Read);
    } catch (const rosbag::BagException& e) {
        logMessage(QString("错误: 无法打开bag文件 %1: %2").arg(QString::fromStdString(bag_path)).arg(QString::fromStdString(e.what())));
        return;
    }

    std::string bag_name = fs::path(bag_path).stem().string();// 获取不带扩展名的文件名

    rosbag::View view_topics(bag);
    std::vector<std::string> radar_topics;
    for (const rosbag::ConnectionInfo* info : view_topics.getConnections()) {
        if (info->topic.find("/wf/corner_radar/sgu_data_") == 0 &&
            info->datatype == "arbe_msgs/wfSguRawData") {
            radar_topics.push_back(info->topic);
        }
    }

    if (radar_topics.empty()) {
        logMessage(QString("警告: 在bag文件 %1 中未找到雷达话题或Msg不匹配").arg(QString::fromStdString(bag_path)));
        bag.close();
        return;
    }

    for (const auto& topic : radar_topics) {
        logMessage(QString("处理话题: %1").arg(QString::fromStdString(topic)));

        std::ofstream csv_file;
        bool header_written = false;
        uint32_t object_counter = 0;
        initCSV(topic, csv_file, header_written, bag_name);

        rosbag::View view(bag, rosbag::TopicQuery(topic));
        for (const rosbag::MessageInstance& m : view) {
            arbe_msgs::wfSguRawData::ConstPtr msg = m.instantiate<arbe_msgs::wfSguRawData>();
            if (msg != nullptr) 
                writeToCSV(msg, csv_file, header_written, object_counter, topic);
            else if (m.getMD5Sum() != ros::message_traits::MD5Sum<arbe_msgs::wfSguRawData>::value()) 
                logMessage(QString("错误: ROS消息MD5值不一致,可能存在消息定义冲突"));
            else 
                logMessage(QString("错误: 无法实例化消息"));
            
        }

        if (csv_file.is_open()) {
            csv_file.close();
             logMessage(QString("完成话题: %1,成功写入 %2 个目标对象").arg(QString::fromStdString(topic)).arg(object_counter));
        }
    }

    bag.close();
    logMessage(QString("完成文件: %1").arg(QString::fromStdString(bag_path)));
}

int main(int argc, char** argv) {
    setlocale(LC_ALL, "en_US.UTF-8");
    ros::init(argc, argv, "radar_to_csv_node");
    
    QApplication app(argc, argv);
    
    RadarToCSV window;
    window.show();
    
    return app.exec();
}

