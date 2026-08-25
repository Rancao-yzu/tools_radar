1、编译
catkin_make

2、运行(提前打开roscore)
bash start.sh

3、添加插件
Panels->Add New Panel->my_rviz_plugin

4、使用
首先：Select Bag File选择bag包，Read Bag File计算bag中的msg大小
然后：play会显示第一帧并暂停，再次点击play进行播放。点击Pause暂停，暂停之后点击Step Backward和Step Forward进行逐帧的前进和后退。
其中：step可以调节逐帧的步长；play rate可以调节播放速率（默认50ms）发布一帧；使用滑轮和指定帧数之前需先暂停。

5、修改
目前版本只支持/wf/corner_radar/parsed/float_data_0", "/cv_camera_0/image_raw/compressed", "/wf/car_id6/parsed三组话题，如需添加，在Bag_reader.cpp中添加：





