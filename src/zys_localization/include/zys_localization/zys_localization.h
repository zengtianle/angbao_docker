#include <ros/ros.h>
#include <iostream>
#include <string>
#include <math.h>
#include <thread>
#include <mutex>
#include <vector>
#include <map>
#include <nav_msgs/OccupancyGrid.h>
#include <sensor_msgs/LaserScan.h>
#include <sensor_msgs/Imu.h>
#include <geometry_msgs/Twist.h>
#include <geometry_msgs/PoseWithCovarianceStamped.h>//init_pos
#include "std_msgs/String.h"

#include <tf/transform_listener.h>
#include"tf/transform_broadcaster.h"


#include "zys_localization/localization_srv.h"
#include "map.h"
#include "field_map.h"
#include <time.h>//引入头文件
#include<opencv2/opencv.hpp>

using namespace std;

struct scan_t{
    scan_t(const int& origin_x, const int& origin_y, const int& origin_theta, const int& theta_scale , const int& map_scale) 
        :   origin_x_(origin_x)
        ,   origin_y_(origin_y)
        ,   origin_theta_(origin_theta)
        ,   theta_scale_(theta_scale)
        ,   map_scale_(map_scale)
        {}
    // Map origin; the map is a viewport onto a conceptual larger map.
    int origin_x_, origin_y_;
    int origin_theta_;
    // Map scale (m/cell)
    int  theta_scale_, map_scale_;
    float score_;

    bool operator<(const scan_t& other) const { return score_ < other.score_; }
    bool operator>(const scan_t& other) const { return score_ > other.score_; }

};

class SlidingWindowMaximum {
 public:
  void AddValue(const double value) {
    while (!non_ascending_maxima_.empty() &&
           value < non_ascending_maxima_.back()) {
      non_ascending_maxima_.pop_back();
    }
    non_ascending_maxima_.push_back(value);
  }

  void RemoveValue(const double value) {
    // DCHECK for performance, since this is done for every value in the
    // precomputation grid.
    // DCHECK(!non_ascending_maxima_.empty());
    assert(!non_ascending_maxima_.empty());
    // DCHECK_LE(value, non_ascending_maxima_.front());
    if (value == non_ascending_maxima_.front()) {
      non_ascending_maxima_.pop_front();
    }
  }

  double GetMaximum() const {
    // DCHECK for performance, since this is done for every value in the
    // precomputation grid.
    // DCHECK_GT(non_ascending_maxima_.size(), 0);
    assert(!non_ascending_maxima_.empty());
    return non_ascending_maxima_.front();
  }
  std::deque<double> non_ascending_maxima_;
//   void CheckIsEmpty() const { CHECK_EQ(non_ascending_maxima_.size(), 0); }

 private:
  // Maximum of the current sliding window at the front. Then the maximum of the
  // remaining window that came after this values first occurrence, and so on.

};

class ZysLocalization
{
    public:
        ZysLocalization();
        ~ZysLocalization();
        bool run(double const & axis_x_, double const & axis_y_);
        void runone(double const & axis_x_, double const & axis_y_, double const & axis_yaw_, bool &result, float &score);
        void score(double const & axis_x_, double const & axis_y_, double const & axis_yaw_, bool &result, float &score);

    private:
        ros::NodeHandle nh_private_;
        ros::NodeHandle nh_;

        map_t* map_;
        multimap<double, scan_t> map_scan_t;
        std::vector<LikelyHoodFieldMap> fieldMaps;
        std::vector<tf::Vector3> ranges;
        std::vector<std::vector<tf::Vector3> > rotatedPoses;
        // std::vector<scan_t> scan_ts;
            
        ros::Publisher initial_pos_pub_ , pub_twist_;
        ros::Subscriber sub_map, sub_scan, sub_imu, sub_map_name;

        std::string sub_scan_topic_name_, sub_tf_tree_name_, output_twist_topic_name_, sub_imu_topic_name_, sub_map_name_topic_name_;
        boost::shared_ptr<tf::TransformListener> listener;

        int matching_size;
        double sigma_hit, z_rand, z_hit, cloud_range_max_;
        double range_max, range_min;
        double score_,score_one_;
        bool run_mutex,getmap_,map_ok,imu_ok;
        double imu_yaw_;
        int num_;
        mutex mutex_it_, mutex_base_;
        std::string mapName_ = "";

        sensor_msgs::LaserScan::ConstPtr scan_;
        void mapCallback(const nav_msgs::OccupancyGrid& map_msg);
        void mapNameCallback(const std_msgs::String& mapName_msg);
        void scanCallback(const sensor_msgs::LaserScan::ConstPtr& scan_msg);
        void imuCallback(const sensor_msgs::ImuConstPtr& imu_msg);
        void constructCoarserMap(LikelyHoodFieldMap const& source);
        vector<tf::Vector3> projectData(double const& theta, std::vector<tf::Vector3> const& data, size_t const& p_x, size_t const& p_y) const;
        void addMapScanT(const scan_t& scan_t_);
        double scan_matching(size_t const& p_x, size_t const& p_y, double const &axis_yaw_, geometry_msgs::PoseWithCovarianceStamped& msg);
        void scoreScanT(std::vector<scan_t> &scan_ts_);
        scan_t BranchAndBound(std::vector<scan_t> &scan_ts_, float const min_score);
        void calculate_score(std::vector<scan_t>::iterator& it_, scan_t& best_scan_t_,std::vector<scan_t> &scan_ts_, float const min_score);
};