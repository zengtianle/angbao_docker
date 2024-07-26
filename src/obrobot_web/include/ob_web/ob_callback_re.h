/*
 * @Author: your name
 * @Date: 2021-06-30 10:26:00
 * @LastEditTime: 2021-07-05 11:34:19
 * @LastEditors: Please set LastEditors
 * @Description: In User Settings Edit
 * @FilePath: /ob_web/include/ob_web/ob_callback_re.h
 */
/*
 * @Author: your name
 * @Date: 2021-06-30 10:26:00
 * @LastEditTime: 2021-07-05 10:30:28
 * @LastEditors: Please set LastEditors
 * @Description: In User Settings Edit
 * @FilePath: /ob_web/include/ob_callback_re.h
 */

#include <vector>
#include <ros/ros.h>

#include <std_msgs/String.h>
#include <std_msgs/Int64.h>
#include <std_msgs/Int64MultiArray.h>
#include <nav_msgs/OccupancyGrid.h>

#include <ob_web/GetJson.h>
#include <ob_web/SaveJson.h>

class ob_callback
{
private:
    /* data */
    ros::NodeHandle m_nh;
    int state = -1;        //系统状态值，-1 待机， 0 启动， 1 建图， 2 导航
    pid_t pid_model = 0;   //正在启动的模式对于launch的pid
    std::string host_path; //主要是区分主机名
    std::string map_path;  //地图保存地址
    std::string pose_path; //位姿文件保存地址
    std::string cfg_path;  //配置文件的保存地址
    std::string stop_script; // 服务停止脚本
    std::string upgrade_script; // 更新脚本
    int pgm_num;           //当前正在修改的地图的编号
    double occ_th;         //占据概率阈值
    double free_th;        //可通行概率阈值
    //std::string name_navigation; //导航用地图名前缀
    //std::string name_laocation;  //定位用地图名前缀
    std::string name_pose_json; // 位置Json的文件名
    std::string name_cfg_json; // 配置Json的文件名
    std::string name_speed_json; // 速度配置Json的文件名

    ros::Publisher currentState_pub; //发送当前状态到前端
    ros::Publisher map_list_pub;     //发送地图列表
    ros::Publisher map_pub;          //发送地图（修改用
    ros::Publisher upgrade_pub;      //发送升级日志

    ros::Subscriber stateChange_sub;
    ros::Subscriber saveMap_sub;
    ros::Subscriber pubPoseJson_sub;
    ros::Subscriber pubCfgJson_sub;
    ros::Subscriber pubMaplist_sub;
    ros::Subscriber pubMap_sub;
    ros::Subscriber savePgm_sub;
    ros::Subscriber deleteMap_sub;
    ros::Subscriber renameMap_sub;
    ros::Subscriber copyMap_sub;
    ros::Subscriber upgrade_sub;
    ros::Subscriber upgrade_body_sub;

    ros::ServiceServer save_cfg_json;
    ros::ServiceServer get_cfg_json;
    ros::ServiceServer get_pos_json;
    ros::ServiceServer get_speed_json;
    ros::ServiceServer save_speed_json;

    std::string getMapPath(int floor);
    std::string getLocationMapPath(int floor);
    std::string getNavigationMapPath(int floor);

    /**
     * @description: 获取json的服务
     * @param {Response} &req
     * @param {Response} &res
     * @return {*}
     */
    bool getPosJsonSrv(ob_web::GetJson::Request &req, ob_web::GetJson::Response &res);

    /**
     * @description: 
     * @param {Response} &req
     * @param {Response} &res
     * @return {*}
     */
    bool getCfgJsonSrv(ob_web::GetJson::Request &req, ob_web::GetJson::Response &res);

    /**
     * @description: 保存json的服务
     * @param {Response} &req
     * @param {Response} &res
     * @return {*}
     */
    bool saveCfgJsonSrv(ob_web::SaveJson::Request &req, ob_web::SaveJson::Response &res);

    /**
     * @brief 读取限速文件
     * 
     * @param req 
     * @param res 
     * @return true 
     * @return false 
     */
    bool getSpeedJsonSrv(ob_web::GetJson::Request &req, ob_web::GetJson::Response &res);
    /**
     * @brief 保存限速文件
     * 
     * @param req 
     * @param res 
     * @return true 
     * @return false 
     */
    bool saveSpeedJsonSrv(ob_web::SaveJson::Request &req, ob_web::SaveJson::Response &res);

    /**
     * @description: 用于改变系统状态
     * @param {const} std_msgs
     * @param {int} *state
     * @param {pid_t} *pid_
     * @param {Publisher} &currentState_pub
     * @return {*}
     */
    void callbackStateChange(const std_msgs::Int64::ConstPtr &msg);

    /**
     * @description: 保存地图
     * @param {const} std_msgs
     * @return {*}
     */
    void callbackSaveMap(const std_msgs::Int64::ConstPtr &msg);

    /**
     * @description: 扫描地图，发送过去
     * @param {const} std_msgs
     * @param {int} *state
     * @param {Publisher} &map_list_pub
     * @return {*}
     */
    void callbackPubmaplist(const std_msgs::Int64::ConstPtr &msg);

    /** 
     * @description:    
     * @param {string} path
     * @return {*}
     */
    std::vector<int64_t> getMaplist(std::string path);

    /**
     * @description: 读取pgm发送出去
     * @param {const} std_msgs
     * @return {*}
     */
    void callbackLoadpmg(const std_msgs::Int64::ConstPtr &msg);

    /**
     * @description: 
     * @param {const} nav_msgs
     * @return {*}
     */
    void callbackSavepmg(const nav_msgs::OccupancyGrid::ConstPtr &msg);

    /**
     * @description: 
     * @param {const} std_msgs
     * @return {*}
     */
    void callbackDelete(const std_msgs::Int64::ConstPtr &msg);

    /**
     * @description: 
     * @param {const} std_msgs
     * @return {*}
     */
    void callbackRename(const std_msgs::Int64MultiArray::ConstPtr &msg);

    /**
     * @description: 
     * @param {const} std_msgs
     * @return {*}
     */
    void callbackCopy(const std_msgs::Int64MultiArray::ConstPtr &msg);

    /**
     * @brief 更新回调
     * 
     * @param msg 
     */
    void callbackUpgradeUrl(const std_msgs::String::ConstPtr &msg);

    /**
     * @brief 更新回调
     * 
     * @param msg 
     */
    void callbackUpgradePackage(const std_msgs::String::ConstPtr &msg);

    /**
     * @brief 执行更新脚本
     * 
     * @param package 更新包路径或URL
     */
    void doUpgrade(const std::string& package);

public:
    ob_callback(ros::NodeHandle nh) : m_nh(nh)
    {
        m_nh.param<std::string>("/ob_callback/host_path", host_path, "home/ob/");
        m_nh.param<std::string>("/ob_callback/map_path", map_path, ".ob/map/");
        m_nh.param<std::string>("/ob_callback/pose_path", pose_path, ".ob/pose/");
        m_nh.param<std::string>("/ob_callback/cfg_path", cfg_path, ".ob/setting/");
        m_nh.param<double>("/ob_callback/occupied_thresh", occ_th, 0.65);
        m_nh.param<double>("/ob_callback/free_thresh", free_th, 0.196);
        m_nh.param<std::string>("/ob_callback/name_pose_json", name_pose_json, "OB_pose.json");
        m_nh.param<std::string>("/ob_callback/name_cfg_json", name_cfg_json, "external_cfg.json");
        m_nh.param<std::string>("/ob_callback/name_speed_json", name_speed_json, "limitConfig.json");
        m_nh.param<std::string>("/ob_callback/stop_script", stop_script, "/home/ob/.ob/autostart/ros_stop.sh");
        m_nh.param<std::string>("/ob_callback/upgrade_script", upgrade_script, "/home/ob/.ob/upgrade.sh");

        currentState_pub = nh.advertise<std_msgs::Int64>("debug/current_mode", 1);
        map_list_pub = nh.advertise<std_msgs::Int64MultiArray>("debug/map_list", 1);
        map_pub = nh.advertise<nav_msgs::OccupancyGrid>("debug/pgm123", 1);
        upgrade_pub = nh.advertise<std_msgs::String>("debug/upgrade_log", 1);

        stateChange_sub = nh.subscribe<std_msgs::Int64>("debug/change_mode", 1, &ob_callback::callbackStateChange, this);
        saveMap_sub = nh.subscribe<std_msgs::Int64>("debug/save_map", 1, &ob_callback::callbackSaveMap, this);
        pubMaplist_sub = nh.subscribe<std_msgs::Int64>("debug/map_list_request", 1, &ob_callback::callbackPubmaplist, this);
        pubMap_sub = nh.subscribe<std_msgs::Int64>("debug/load_pgm", 1, &ob_callback::callbackLoadpmg, this);
        savePgm_sub = nh.subscribe<nav_msgs::OccupancyGrid>("debug/save_pgm", 1, &ob_callback::callbackSavepmg, this);
        deleteMap_sub = nh.subscribe<std_msgs::Int64>("debug/delete_pgm", 1, &ob_callback::callbackDelete, this);
        renameMap_sub = nh.subscribe<std_msgs::Int64MultiArray>("debug/rename_pgm", 1, &ob_callback::callbackRename, this);
        copyMap_sub = nh.subscribe<std_msgs::Int64MultiArray>("debug/copy_pgm", 1, &ob_callback::callbackCopy, this);
        upgrade_sub = nh.subscribe<std_msgs::String>("debug/upgrade", 1, &ob_callback::callbackUpgradeUrl, this);
        upgrade_body_sub = nh.subscribe<std_msgs::String>("debug/upgrade_pkg", 1, &ob_callback::callbackUpgradePackage, this);

        //get_json = nh.advertiseService();
        get_cfg_json = nh.advertiseService("debug/getCfgJson", &ob_callback::getCfgJsonSrv, this);
        get_pos_json = nh.advertiseService("debug/getPosJson", &ob_callback::getPosJsonSrv, this);
        save_cfg_json = nh.advertiseService("debug/saveCfgJson", &ob_callback::saveCfgJsonSrv, this);
        get_speed_json = nh.advertiseService("debug/getSpeedJson", &ob_callback::getSpeedJsonSrv, this);
        save_speed_json = nh.advertiseService("debug/saveSpeedJson", &ob_callback::saveSpeedJsonSrv, this);
    
    }
    ~ob_callback()
    {
    }
};
