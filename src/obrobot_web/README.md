<!--
 * @Author: your name
 * @Date: 2021-04-25 17:02:46
 * @LastEditTime: 2021-07-06 20:55:21
 * @LastEditors: Please set LastEditors
 * @Description: In User Settings Edit
 * @FilePath: /ob_web/README.md
-->


```
git clone https://github.com/jbeder/yaml-cpp.git
cd yaml-cpp
mkdir build && cd build
cmake -DYAML_BUILD_SHARED_LIBS=ON ..
make -j4
sudo make install
```
如果报错
Could NOT find SDL (missing: SDL_LIBRARY SDL_INCLUDE_DIR)
```
 sudo apt-get install libsdl1.2-dev
```
Could NOT find SDL_image (missing:SDL_IMAGE_LIBRARIES SDL_IMAGE_INCLUDE_DIRS)
```
sudo apt-get install libsdl-image1.2-dev
```
## 注意事项
由于开发过程中并没有cartographer的launch文件，所以暂时使用cartograph.launch代替的，launch文件有误，需要修改ob_callback_re.cc文件的221行（ob_callback::callbackStateChange函数）内容，即将
```
else if (msg->data == 3)
{
    char *argv[] = {"roslaunch", "obrobot_navigation", "cartograph.launch", NULL};
    pid_model = Exec("roslaunch", argv);
}
```
修改为
```
else if (msg->data == 3)
{
    char *argv[] = {"roslaunch", "launch文件所在包名", "launch文件名", NULL};
    pid_model = Exec("roslaunch", argv);
}
```
### 参数说明
主要的参数配置都写在了launch文件中了，根据实际可以自己做一些修改
```
<launch>
    <arg name="pkgname" default="ob_web" />
    <node name="ob_callback" pkg="$(arg pkgname)" type="ob_callback" output="screen">
        <param name="host_path" type="string" value="/home/ob/" />
        <param name="map_path" type="string" value=".ob/map/" />
        <param name="pose_path" type="string" value=".ob/pos/" />
        <param name="cfg_path" type="string" value=".ob/setting/" />
        <param name="occupied_thresh" type="double" value="0.65" />
        <param name="free_thresh" type="double" value="0.196" />
        <param name="name_pose_json" type="string" value="OB_pose.json" />
        <param name="name_cfg_json" type="string" value="external_cfg.json" />
        <param name="name_speed_json" type="string" value="limitConfig.json" />
        <param name="upgrade_script" type="string" value="/home/ob/.ob/upgrade.sh" />
    </node>
</launch>
```
- host_path ， 主要是用来区分主机，格式为/home/主机名/
- map_path ，  map文件的存放地址，从根目录开始
- pose_path， 导航点存放地址
- cfg_path， 配置文件的存放地址
- occupied_thresh， 地图点占据概率阈值，占据概率大于这个值则认为这个点是存在障碍物的，占据概率ooc的计算方式为  (255 - 像素点rgb通道的平均值) / 255.0
- free_thresh， 可通行概率阈值， 占据概率ooc小于这个阈值则认为这个点可通行，对于ooc>free_thresh&&ooc<occupied_thresh的就是未知区域
- name_pose_json， 导航点文件名
- name_cfg_json， 配置文件名
- name_speed_json， 限速文件名
- upgrade_script， 更新脚本路径

### 接口说明
#### pub
- debug/current_mode， 当前系统模式值，系统一共5个状态，-1为未连接，0为连接成功，1为gmapping，2为navigation，3为cartographer
- debug/map_list， 地图列表
- debug/pgm123， 要修改的地图内容
- debug/upgrade_log，更新日志行

#### sub
- debug/change_mode， 模式转换请求，内容为要转换的系统模式
- debug/save_map， 地图保存请求，内容为要保存地图的楼层号码
- debug/map_list_request， 地图列表请求，内容随意
- debug/load_pgm， 加载地图请求，内容为要加载的楼层号，其中正数为navigation用地图，负数为location用地图
- debug/save_pgm， 保存修改的地图，内容为要保存的地图
- debug/delete_pgm， 删除地图，内容为要删除的楼层号
- debug/rename_pgm， 重命名地图，0号元素是源操作数，1号元素是目的操作数
- debug/copy_pgm， 复制地图，0号元素是源操作数，1号元素是目的操作数
- debug/upgrade，下载更新包并更新。参数为更新包的URL

#### srv
- debug/getCfgJson， 获取配置文件内容
- debug/getPosJson， 获取导航点文件内容
- debug/saveCfgJso， 写入配置文件
- debug/getSpeedJson， 获取限速配置文件内容
- debug/saveSpeedJson， 保存限速配置文件内容