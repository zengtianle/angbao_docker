/**
 * @file pgm_map_util.h
 * @author Jiang Ming (jim@lotlab.org)
 * @brief 
 * @version 0.1
 * @date 2021-11-04
 * 
 * @copyright Copyright (c) 2021 ESAC
 * 
 */
#include <string>
#include <nav_msgs/OccupancyGrid.h>

class PgmUtil {
public:
    /**
     * @brief 复制一个地图文件到指定位置
     * 
     * @param from 地图源文件（不带扩展名）
     * @param to 地图目标文件（不带扩展名）
     */
    static void Copy(std::string from, std::string to);
    /**
     * @brief 重命名一个地图文件
     * 
     * @param from 地图源文件（不带扩展名）
     * @param to 地图目标文件（不带扩展名）
     */
    static void Rename(std::string from, std::string to);
    /**
     * @brief 删除一个地图文件
     * 
     * @param path 地图文件（不带扩展名）
     */
    static void Remove(std::string path);
    /**
     * @brief 判断地图文件是否存在
     * 
     * @param path 地图文件（不带扩展名）
     * @return true 
     * @return false 
     */
    static bool Exists(std::string path);

    /**
     * @brief 读取指定的地图文件，并将其转换为ROS消息
     * 
     * @param path 地图路径
     * @param occTh 占据概率阈值
     * @param freeTh 可通行概率阈值
     * @return nav_msgs::OccupancyGrid 
     */
    static nav_msgs::OccupancyGrid ReadMap(std::string path, double occTh = 0.65, double freeTh = 0.196);

    /**
     * @brief 使用指定的ROS消息编辑指定的地图文件
     * 
     * @param path 地图路径
     * @param map 地图消息
     * @return true 
     * @return false 
     */
    static bool EditMap(std::string path, nav_msgs::OccupancyGridConstPtr map);
    /**
     * @brief 修改地图文件的yaml信息
     * 
     * @param path 地图文件（不带扩展名）
     */
    static void ModifyPgmYaml(std::string path);
private:
    /**
     * @brief 获取指定文件的文件名
     * 
     * @param path 文件路径
     * @return std::string 文件名
     */
    static std::string Filename(std::string path);

    /**
     * @brief 获取地图对应的yaml文件的文件名
     * 
     * @param path 地图文件（不带扩展名）
     * @return std::string 
     */
    static std::string getYamlName(std::string path);
    /**
     * @brief 获取地图对应的Pgm文件的文件名
     * 
     * @param path 地图文件（不带扩展名）
     * @return std::string 
     */
    static std::string getPgmName(std::string path);
};
