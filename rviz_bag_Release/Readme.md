# RVIZ插件的 Release 发行版本
> - 在使用前，需要检查默认上位机主节点是否有srv/PlaySingleFrame.srv服务。
> - 要确保对应msg结构一致，否则会有MD5 value 不一致问题。

所有插件默认 publish 的topic：
- /wf/car_id6/parsed
- /cv_camera_/image_raw/compressed

启动:`bash start.sh` 或  `python3 start.py`

> # **非开发者不允许更改Readme!**

## 对应版本

### 8T8R 项目
- lib/8T8R_rviz_bag_Folder.so
  - /wf/front_radar/parsed/float_data_
  - /wf/frame_rd_data/ti/radar_
  - /wf/front_radar/ti/cube_data_

### SDK 项目
- lib/rviz_bag_2e44lc_geely.so
  - /wf/corner_radar/postprocess_data_
  - /wf/frame_rd_data/ti/radar_

- lib/rviz_bag_2e44lc_SDK_V1.0.so
  - /wf/corner_radar/parsed/float_data_
  - /wf/frame_rd_data/ti/radar_
  
- lib/rviz_bag_2e44lc_SDK_V1.2.so **(该版本可选择播放原始objects)**
  - /wf/corner_radar/parsed/float_data_
  - /wf/frame_rd_data/ti/radar_
  - /wf/imu_data/parsed
  - /wf/rviz/objects_tags_
  - /wf/rviz/objects_

- lib/rviz_bag_2e44lc_BLD_ALL_Radar_Folder_V2.3.so **(批量回灌版本)**
  - /wf/corner_radar/parsed/float_data_
  - /wf/frame_rd_data/ti/radar_
  - /wf/corner_radar/rd_data_

- lib/rviz_bag_simulate.so
  - /wf/corner_radar/parsed/float_data_
  - /wf/frame_rd_data/ti/radar_
  - /wf/corner_radar/simulate_data_


### AtoSar_LGU 项目
- lib/rviz_bag_AutoSar_LGU_90.so
  - /wf/corner_radar/lgu_data_
  - /wf/corner_radar/sgu_data_
  
- lib/rviz_bag_AutoSar_LGU_V1.1.so
  - /wf/corner_radar/lgu_data_
  - /wf/imu_data/parsed
  - /corner_radar/warning_status

- lib/rviz_bag_AutoSar_LGU_V2.1.so **(该版本不是逐帧，其播放按照8ms逐个递进)**
  - /wf/corner_radar/lgu_data_
  - /wf/imu_data/parsed
  - /corner_radar/warning_status
  
- lib/rviz_bag_AutoSar_LGU_winter.so **(冬测版本，唯一与1.1版本区别为摄像头增加为10个)**

- lib/rviz_bag_AtoSar_LGU_Folder.so **(批量回灌版本)**
  - /wf/corner_radar/lgu_data_

### AtoSar 项目
- lib/rviz_bag_sgu_autosar_V1.1.so
  - /wf/frame_rd_data/ti/radar_
  - /wf/corner_radar/sgu_data_
  - /wf/imu_data/parsed
  - /wf/corner_radar/adas_input_data_
  - /corner_radar/warning_status

- lib/rviz_bag_sgu_autosar_Folder_V1.0.so **(批量回灌版本)**
  - /wf/frame_rd_data/ti/radar_
  - /wf/corner_radar/sgu_data_
  - /wf/imu_data/parsed
  - /wf/corner_radar/adas_input_data_
  - /corner_radar/warning_status

- lib/rviz_bag_sgu_autosar_VW.so **(此版本的msg结构变化为VW结构)**
  - /wf/frame_rd_data/ti/radar_
  - /wf/corner_radar/sgu_data_
  - /wf/imu_data/parsed
  - /wf/corner_radar/adas_input_data_
  - /corner_radar/warning_status

- lib/rviz_bag_sgu_autosar_VW_Time_new.so **(该版本不是逐帧，其播放按照8ms逐个递进)**
  - /wf/frame_rd_data/ti/radar_
  - /wf/corner_radar/sgu_data_
  - /wf/imu_data/parsed
  - /wf/corner_radar/adas_input_data_
  - /corner_radar/warning_status