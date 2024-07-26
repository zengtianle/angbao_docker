/*
 * @Author: your name
 * @Date: 2021-06-30 10:25:15
 * @LastEditTime: 2021-07-12 20:02:22
 * @LastEditors: Please set LastEditors
 * @Description: In User Settings Edit
 * @FilePath: /ob_web/src/ob_callback_re.cc
 */
#include "ob_web/ioutil.h"
#include "ob_web/pgm_map_util.h"
#include <ob_web/ob_callback_re.h>

#include <dirent.h>
#include <iostream>
#include <sys/types.h>
#include <sys/wait.h>
using namespace std;

std::string ob_callback::getMapPath(int floor)
{
    return floor > 0 ? getNavigationMapPath(floor) : getLocationMapPath(-floor);
}
std::string ob_callback::getLocationMapPath(int floor)
{
    return host_path + map_path + "OB_floor_location_" + std::to_string(floor);
}
std::string ob_callback::getNavigationMapPath(int floor)
{
    return host_path + map_path + "OB_floor_" + std::to_string(floor);
}

bool ob_callback::getPosJsonSrv(ob_web::GetJson::Request &req, ob_web::GetJson::Response &res)
{
    std::string path = host_path + pose_path + name_pose_json;
    res.json = ioutil::ReadFile(path);
    return true;
}

bool ob_callback::getCfgJsonSrv(ob_web::GetJson::Request &req, ob_web::GetJson::Response &res)
{
    std::string path = host_path + cfg_path + name_cfg_json;
    res.json = ioutil::ReadFile(path);
    return true;
}

bool ob_callback::saveCfgJsonSrv(ob_web::SaveJson::Request &req, ob_web::SaveJson::Response &res)
{
    std::string path = host_path + cfg_path + name_cfg_json;
    ioutil::WriteFile(path, req.json);
    return true;
}

bool ob_callback::getSpeedJsonSrv(ob_web::GetJson::Request &req, ob_web::GetJson::Response &res)
{
    std::string path = host_path + cfg_path + name_speed_json;
    res.json = ioutil::ReadFile(path);
    return true;
}

bool ob_callback::saveSpeedJsonSrv(ob_web::SaveJson::Request &req, ob_web::SaveJson::Response &res)
{
    std::string path = host_path + cfg_path + name_speed_json;
    ioutil::WriteFile(path, req.json);
    return true;
}

std::vector<int64_t> ob_callback::getMaplist(std::string path)
{
    std::map<int64_t, int> navigation_map;
    std::map<int64_t, int> location_map;
    std::map<int64_t, int> map_list_map;
    std::vector<int64_t> list;

    list.clear();

    DIR *directory_pointer;
    struct dirent *entry;
    int map_num;
    if ((directory_pointer = opendir(path.c_str())) != NULL)
    {
        while ((entry = readdir(directory_pointer)) != NULL)
        {
            if (entry->d_name[0] == '.')
            {
                continue;
            }

            if (strlen(entry->d_name) < 10)
            {
                continue;
            }
            // cout << entry->d_name << endl;

            //这里只校验了第一个和第十个字母，请不要插入奇怪的文件
            if (entry->d_name[0] == 'O' && entry->d_name[9] >= '0' && entry->d_name[9] <= '9')
            {
                map_num = atoi(entry->d_name + 9);
                navigation_map[map_num]++;
            }
            else if (entry->d_name[0] == 'O' && entry->d_name[9] == 'l')
            {
                map_num = atoi(entry->d_name + 18);
                location_map[map_num]++;
            }
            if (navigation_map[map_num] >= 2 && location_map[map_num] >= 2 && map_list_map[map_num] == 0)
            {
                list.push_back(map_num);
                map_list_map[map_num] = 1;
            }
        }
    }

    return list;
}

void ob_callback::callbackStateChange(const std_msgs::Int64::ConstPtr &msg)
{
    cout << "准备状态转换：" << state << " -> " << msg->data << endl;
    int tmp_state = state;
    //所有的状态都可以转到0
    if (msg->data == 0)
    {
        //当前状态为-1，关闭自启动服务
        if (tmp_state == -1)
        {
            pid_t tmp_pid = ioutil::Exec("sudo", "sudo", stop_script);
        }
        else if (tmp_state == 1 || tmp_state == 2 || tmp_state == 3) //否则kill掉正在运行的launch
        {
            kill(pid_model, SIGTERM);
            pid_model = 0;
        }

        //更新状态，发送
        state = msg->data;
        std_msgs::Int64 pub_tmp;
        pub_tmp.data = state;
        currentState_pub.publish(pub_tmp);
        return;
    }

    //0可以转到-1以外的状态
    if (tmp_state == 0)
    {
        if (msg->data == -1)
        {
            printf("从状态0转到状态-1是不合法的\n");
            return;
        }
        else if (msg->data == 1)
        {
            pid_model = ioutil::Exec("roslaunch", "roslaunch", "obrobot_navigation", "gmapping.launch");
        }
        else if (msg->data == 2)
        {
            pid_model = ioutil::Exec("roslaunch", "roslaunch", "obrobot_navigation", "navigation_web_tool.launch");
        }
        else if (msg->data == 3)
        {
            pid_model = ioutil::Exec("roslaunch", "roslaunch", "obrobot_navigation", "cartographer.launch");
        }

        state = msg->data;
        std_msgs::Int64 pub_tmp;
        pub_tmp.data = state;
        currentState_pub.publish(pub_tmp);
        return;
    }
    //其他状态转化不合法
    printf("从状态%d转到状态%ld是不合法的\n", state, msg->data);
}

void ob_callback::callbackSaveMap(const std_msgs::Int64::ConstPtr &msg)
{
    std::string navigation_path = getNavigationMapPath(msg->data);
    std::string location_path = getLocationMapPath(msg->data);
    cout << "保存导航地图至: " << navigation_path << endl;
    cout << "保存定位地图至: " << location_path << endl;

    pid_t exec_pid = ioutil::Exec("rosrun", "rosrun", "map_server", "map_saver", "-f", navigation_path);
    waitpid(exec_pid, NULL, 0);
    exec_pid = ioutil::Exec("rosrun", "rosrun", "map_server", "map_saver", "-f", location_path);
    waitpid(exec_pid, NULL, 0);
}

void ob_callback::callbackPubmaplist(const std_msgs::Int64::ConstPtr &msg)
{
    cout << "正在获取地图列表" << endl;
    std_msgs::Int64MultiArray map_list;
    std::string path = host_path + map_path;

    std::vector<int64_t> list = getMaplist(path);
    for (int i = 0; i < list.size(); i++)
    {
        map_list.data.push_back(list[i]);
    }

    map_list_pub.publish(map_list);
}

void ob_callback::callbackLoadpmg(const std_msgs::Int64::ConstPtr &msg)
{
    pgm_num = int(msg->data);
    auto path = getMapPath(msg->data);
    nav_msgs::OccupancyGrid grid = PgmUtil::ReadMap(path, occ_th, free_th);
    map_pub.publish(grid);
}

void ob_callback::callbackSavepmg(const nav_msgs::OccupancyGrid::ConstPtr &msg)
{
    auto path = getMapPath(pgm_num);
    PgmUtil::EditMap(path, msg);
    std::cout << "地图(" << pgm_num << ")修改成功 " << path << std::endl;
}

void ob_callback::callbackDelete(const std_msgs::Int64::ConstPtr &msg)
{
    std::string navigation_path = getNavigationMapPath(msg->data);
    std::string location_path = getLocationMapPath(msg->data);

    PgmUtil::Remove(location_path);
    PgmUtil::Remove(navigation_path);
}

void ob_callback::callbackRename(const std_msgs::Int64MultiArray::ConstPtr &msg)
{
    if (msg->data[0] == msg->data[1])
    {
        cerr << "地图源文件与目标文件相同" << endl;
        return;
    }

    std::string location_path_origin = getLocationMapPath(msg->data[0]);
    std::string location_path_purpose = getLocationMapPath(msg->data[1]);
    std::string navigation_path_origin = getNavigationMapPath(msg->data[0]);
    std::string navigation_path_purpose = getNavigationMapPath(msg->data[1]);

    cout << "地图 " << location_path_origin << " 重命名至 " << location_path_purpose << endl;
    cout << "地图 " << navigation_path_origin << " 重命名至 " << navigation_path_purpose << endl;

    // 禁止覆盖已经存在的地图文件
    if (PgmUtil::Exists(navigation_path_purpose))
    {
        cerr << "地图文件已存在，跳过" << endl;
        return;
    }

    PgmUtil::Rename(location_path_origin, location_path_purpose);
    PgmUtil::Rename(navigation_path_origin, navigation_path_purpose);
}

void ob_callback::callbackCopy(const std_msgs::Int64MultiArray::ConstPtr &msg)
{
    std::string location_path_origin = getLocationMapPath(msg->data[0]);
    std::string location_path_purpose = getLocationMapPath(msg->data[1]);
    std::string navigation_path_origin = getNavigationMapPath(msg->data[0]);
    std::string navigation_path_purpose = getNavigationMapPath(msg->data[1]);

    cout << "地图 " << location_path_origin << " 复制至 " << location_path_purpose << endl;
    cout << "地图 " << navigation_path_origin << " 复制至 " << navigation_path_purpose << endl;

    // 禁止覆盖已经存在的地图文件
    if (PgmUtil::Exists(navigation_path_purpose))
    {
        cerr << "地图文件已存在，跳过" << endl;
        return;
    }

    PgmUtil::Copy(location_path_origin, location_path_purpose);
    PgmUtil::Copy(navigation_path_origin, navigation_path_purpose);
}

void ob_callback::callbackUpgradeUrl(const std_msgs::StringConstPtr &msg)
{
    cout << "Upgrade package url: " << msg->data << std::endl;
    doUpgrade(msg->data);
}

void ob_callback::callbackUpgradePackage(const std_msgs::StringConstPtr &msg)
{
    const std::string filename = "/tmp/ob_upgrade_pkg.tar.gz";
    cout << "Receive upgrade package, write to " << filename << std::endl;
    ioutil::WriteBase64File(filename, msg->data);
    doUpgrade(filename);
}

void ob_callback::doUpgrade(const std::string &package)
{
    auto cmd = upgrade_script + " " + package + " 2>&1";
    FILE *make_ptr;
    char buf_ps[1024];
    if (make_ptr = popen(cmd.c_str(), "r"))
    {
        while (fgets(buf_ps, 1024, make_ptr))
        {
            std_msgs::String build_log;
            build_log.data = buf_ps;

            upgrade_pub.publish(build_log);
        }
        pclose(make_ptr);
        make_ptr = NULL;
    }
    else
    {
        cerr << "Execute " << cmd << " error." << endl;
    }
}

int main(int argc, char **argv)
{
    // Initialize ROS
    ros::init(argc, argv, "ob_callback");
    ros::NodeHandle nh;

    ob_callback obc(nh);

    while (ros::ok())
        ros::spin();
}