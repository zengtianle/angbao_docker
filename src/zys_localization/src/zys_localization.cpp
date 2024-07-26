#include <zys_localization/zys_localization.h>

#include <pcl/point_types.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/filters/radius_outlier_removal.h>
#include <pcl/filters/statistical_outlier_removal.h>
#include <pcl/conversions.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/common/transforms.h>
#include <pcl/PCLPointCloud2.h>
#include <pcl_ros/transforms.h>
#include <pcl_ros/point_cloud.h>
using namespace std;


ZysLocalization::ZysLocalization()
    : nh_private_("~")
    , cloud_range_max_(50)
    , run_mutex(false)
    , map_ok(false)
{
    
    nh_private_.param<std::string>("sub_scan_topic_name_", sub_scan_topic_name_, "scan");
    nh_private_.param<std::string>("sub_tf_tree_name_", sub_tf_tree_name_, "base_footprint");
    nh_private_.param<std::string>("output_twist_topic_name_", output_twist_topic_name_, "/cmd_vel");
    nh_private_.param<std::string>("sub_imu_topic_name_", sub_imu_topic_name_, "/imu");
    nh_private_.param<std::string>("sub_map_name_topic_name_", sub_map_name_topic_name_, "/reset_map_name");

    nh_private_.param("laser_sigma_hit", sigma_hit, 0.2);
	nh_private_.param("laser_z_rand", z_rand, 0.05);
	nh_private_.param("laser_z_hit", z_hit, 0.95);
    nh_private_.param("matching_size", matching_size, 2);
    nh_private_.param("cloud_range_max", cloud_range_max_, 50.0);
    nh_private_.param("score", score_, 0.85);
    nh_private_.param("score_one", score_one_, 0.85);

    nh_private_.param("getmap", getmap_, false);

    setlocale(LC_CTYPE,"zh_CN.utf8");
	listener.reset(new tf::TransformListener());
    
    initial_pos_pub_=nh_.advertise<geometry_msgs::PoseWithCovarianceStamped>("/initialpose",1, true);
    pub_twist_ = nh_.advertise<geometry_msgs::Twist>(output_twist_topic_name_, 1, true);

    sub_map_name = nh_.subscribe(sub_map_name_topic_name_, 1,  &ZysLocalization::mapNameCallback, this);
    sub_map = nh_.subscribe("map", 1, &ZysLocalization::mapCallback, this);
    sub_scan = nh_.subscribe(sub_scan_topic_name_, 1,  &ZysLocalization::scanCallback, this);
    sub_imu = nh_.subscribe(sub_imu_topic_name_, 1,  &ZysLocalization::imuCallback, this);
    

}


ZysLocalization::~ZysLocalization()
{
    ROS_INFO("close icp Matching Estimate");
}

// bool ZysLocalization::run(double const & axis_x_, double const & axis_y_)
// {
//     cout<<endl<<endl<<endl;
//     if (run_mutex){
//         ROS_INFO("ICP Matching Already Running!!!!!");
//         return 0;
//     }
//     else run_mutex = true;
//     if (scan_ == NULL || !map_ok) {
//         cout << "scan_: "<< (scan_ == NULL)<<endl;
//         cout << "map_: "<< map_ok<<endl;
//         run_mutex = false;
//         return 0 ;
//     }
//     std::string error_msg;
// 	if (!listener->canTransform(sub_tf_tree_name_, scan_->header.frame_id, ros::Time(0), &error_msg))
// 	{
// 		// ROS_ERROR_STREAM_THROTTLE(1.0, "can not transform: "<<sub_tf_tree_name_<<" to "<<scan_->header.frame_id);
//         ROS_ERROR("ZysLocalization can not transform:%s  to %s", sub_tf_tree_name_, scan_->header.frame_id);
//         run_mutex = false;
// 		return 0 ;
// 		// continue;
		
// 	}

//     size_t p_x = MAP_GXWX(map_, axis_x_);
//     size_t p_y = MAP_GYWY(map_, axis_y_);
//     // printf("zero p_x:%lf, zero p_y:%lf position.x:%f position.y:%f\n",MAP_GXWX(map_, 0),MAP_GYWY(map_, 0),map_->origin_x,map_->origin_y);
//     bool result;
//     geometry_msgs::Twist ts;
//     ros::Rate loop_rate(100);
//     for(int i = 1;i <= 4;i++){
//         geometry_msgs::PoseWithCovarianceStamped msg_;
//         double _score = scan_matching(p_x, p_y, msg_);
//         if (_score > score_){
//             initial_pos_pub_.publish(msg_);
//             result = true;
//         }else{
//             result = false;
//         }
//         if(result) break;
//         double imu_now = imu_yaw_ + M_PI_2;
//         imu_now = imu_now > M_PI ? imu_now - (2 * M_PI) : imu_now;
//         // cout<<"imu_now:"<<imu_now<<endl;
//         while (fabs(imu_yaw_ - imu_now) > 0.1)
//         {
//             ros::spinOnce();
//             ts.angular.z = 0.5;
//             pub_twist_.publish(ts);
//             loop_rate.sleep();
//         }
//         cout<<"stop"<<endl;
//         ts.angular.z = 0.0;
//         pub_twist_.publish(ts);
//         loop_rate.sleep();
//     }
//     run_mutex = false;
//     return result;
// }

void ZysLocalization::score(double const & axis_x_, double const & axis_y_, double const & axis_yaw_, bool &result, float &score){
    cout<<endl<<endl<<endl;
    if (run_mutex){
        ROS_INFO("ICP Matching Already Running!!!!!");
        result = false;
        return ;
    }
    else run_mutex = true;
    if (scan_ == NULL || !map_ok) {
        cout << "scan_: "<< (scan_ == NULL)<<endl;
        cout << "map_: "<< map_ok<<endl;
        run_mutex = false;
        result = false;
        return ;
    }
    std::string error_msg;
	if (!listener->canTransform(sub_tf_tree_name_, scan_->header.frame_id, ros::Time(0), &error_msg))
	{
		// ROS_ERROR_STREAM_THROTTLE(1.0, "can not transform: "<<sub_tf_tree_name_<<" to "<<scan_->header.frame_id);
        ROS_ERROR("ZysLocalization not transform:%s  to %s", sub_tf_tree_name_, scan_->header.frame_id);
        run_mutex = false;
        result = false;
		return ;
		// continue;
		
	}
    rotatedPoses.clear();
    ranges.clear();

    if (cloud_range_max_ > 0.0) range_max = std::min(static_cast<double>(scan_->range_max), cloud_range_max_);
    else range_max = scan_->range_max;
    range_min = scan_->range_min;
    // range_min = ranges_agv/double(num)*0.33333;
    // range_min = 1.0;
    for (size_t i = 0; i < scan_->ranges.size(); ++i) {
        if (std::isinf(scan_->ranges[i]) || std::isnan(scan_->ranges[i])) continue;

        if (scan_->ranges[i] < range_min || scan_->ranges[i] >= range_max) continue;

        double angle = scan_->angle_min + i * scan_->angle_increment;
        double x = cos(angle) * scan_->ranges[i];
        double y = sin(angle) * scan_->ranges[i];
        // tf::Vector3 range();
        ranges.emplace_back(x,y,0);
    }


    size_t p_x = MAP_GXWX(map_, axis_x_);
    size_t p_y = MAP_GYWY(map_, axis_y_);
    double axis_yaw__ = axis_yaw_;
    if(axis_yaw__< 0) axis_yaw__+=180.0;

    // std::vector<scan_t> scan_ts;
    // scan_ts.emplace_back(0, 0, int(axis_yaw__+0.5555), 1, 1);
    // rotatedPoses.emplace_back(projectData(axis_yaw_, ranges, p_x, p_y));
    for (double theta = 0; theta < 360; theta ++) {
        rotatedPoses.emplace_back(projectData(theta, ranges, p_x, p_y));
    }
    // scoreScanT(scan_ts);



    scan_t scan_t_(0, 0, int(axis_yaw__+0.5555), 1, 1);
    double score_ = 0.0;
    int const mapindex = log2(scan_t_.map_scale_);
    // size_t const maxX = fieldMaps[mapindex].xSize();
    // size_t const maxY = fieldMaps[mapindex].ySize();
    int x_offset = scan_t_.origin_x_*scan_t_.map_scale_;
    int y_offset = scan_t_.origin_y_*scan_t_.map_scale_;

    for (const tf::Vector3& pose :projectData(axis_yaw__, ranges, p_x, p_y)){
        size_t decimated_x = pose.getX() + x_offset;
        size_t decimated_y = pose.getY() + y_offset;
        // printf(" %f,",fieldMaps[mapindex].getEntry(decimated_x, decimated_y));
        score_ += fieldMaps[mapindex].getEntry(decimated_x, decimated_y);
        
    }
    // printf("\nscore_:%f,rotatedPose.size:%d, max :%f, min :%f",score_, rotatedPoses[scan_t_.origin_theta_].size(), fieldMaps[0].max_entry,fieldMaps[0].min_entry);
    scan_t_.score_ = 1 - (score_ / rotatedPoses[0].size() / (fieldMaps[0].max_entry - fieldMaps[0].min_entry)); 
    // scan_t_.score_ = 0.1 + (1 - (score / rotatedPoses[0].size() / fieldMaps[0].max_entry)) * 0.8; 
    // printf(" %f,",scan_t_.score_);
    score = scan_t_.score_;





    // score = scan_ts.begin()->score_;
    run_mutex = false;

    result = true;
}

void ZysLocalization::runone(double const & axis_x_, double const & axis_y_, double const &axis_yaw_, bool &result, float &score)
{
    cout<<endl<<endl<<endl;
    if (run_mutex){
        ROS_INFO("ICP Matching Already Running!!!!!");
        result = false;
        return;
    }
    else run_mutex = true;
    if (scan_ == NULL || !map_ok) {
        cout << "scan_: "<< (scan_ == NULL)<<endl;
        cout << "map_: "<< map_ok<<endl;
        run_mutex = false;
        result = false;
        return ;
    }
    std::string error_msg;
	if (!listener->canTransform(sub_tf_tree_name_, scan_->header.frame_id, ros::Time(0), &error_msg))
	{
		// ROS_ERROR_STREAM_THROTTLE(1.0, "can not transform: "<<sub_tf_tree_name_<<" to "<<scan_->header.frame_id);
        ROS_ERROR("ZysLocalization can not transform:%s  to %s", sub_tf_tree_name_, scan_->header.frame_id);
        run_mutex = false;
        result = false;
		return ;
		// continue;
		
	}
    size_t p_x = MAP_GXWX(map_, axis_x_);
    size_t p_y = MAP_GYWY(map_, axis_y_);
    printf("zero p_x:%lf, zero p_y:%lf position.x:%f position.y:%f\n",MAP_GXWX(map_, 0),MAP_GYWY(map_, 0),map_->origin_x,map_->origin_y);
    geometry_msgs::Twist ts;
    // result = scan_matching(p_x,p_y,score_one_);
    geometry_msgs::PoseWithCovarianceStamped msg_;
    score = scan_matching(p_x, p_y, axis_yaw_, msg_);
    initial_pos_pub_.publish(msg_);
    cout << "matching OK! score: "<<score<<endl;
    // if (_score > score_one_){
    //     initial_pos_pub_.publish(msg_);
    //     result = true;
    //     cout << "matching OK!"<<endl;
    // }else{
    //     result = false;
    //     cout << "无法判定成功"<<endl;
    // }
    run_mutex = false;
    result = true;
}



double ZysLocalization::scan_matching(size_t const& p_x, size_t const& p_y, double const &axis_yaw_, geometry_msgs::PoseWithCovarianceStamped& msg)
{
    ranges.clear();
    map_scan_t.clear();
    // scan_ts.clear();
    rotatedPoses.clear();

    int num = 0;
    double ranges_agv = 0.0;
    for(double range:scan_->ranges){
        if (std::isinf(range) || std::isnan(range)) continue;
        ranges_agv += range;
        num++;
    }
    // cout<<"ranges_agv:"<<(ranges_agv/scan_->ranges.size())<<" "<< (ranges_agv/num)<<endl;



    if (cloud_range_max_ > 0.0) range_max = std::min(static_cast<double>(scan_->range_max), cloud_range_max_);
    else range_max = scan_->range_max;
    range_min = scan_->range_min;
    // range_min = ranges_agv/double(num)*0.33333;
    // range_min = 1.0;
    pcl::PointCloud<pcl::PointXYZ>::Ptr scan_ptr_(new pcl::PointCloud<pcl::PointXYZ>());
    pcl::PointCloud<pcl::PointXYZ>::Ptr scan_filter_ptr_(new pcl::PointCloud<pcl::PointXYZ>());
    pcl::PointXYZ new_point;
    new_point.z = 0;
    for (size_t i = 0; i < scan_->ranges.size(); ++i) {
        if (std::isinf(scan_->ranges[i]) || std::isnan(scan_->ranges[i])) continue;

        if (scan_->ranges[i] < range_min || scan_->ranges[i] >= range_max) continue;

        double angle = scan_->angle_min + i * scan_->angle_increment;
        double x = cos(angle) * scan_->ranges[i];
        double y = sin(angle) * scan_->ranges[i];
        // tf::Vector3 range();
        // ranges.emplace_back(x,y,0);
        new_point.x = x;
        new_point.y = y;
        scan_ptr_->push_back(new_point);
    }
    // printf("ranges size:%d\n",ranges.size());
    //滤波
    pcl::VoxelGrid<pcl::PointXYZ> voxel_grid_filter;
    float voxel_leaf_size = 0.05;
    voxel_grid_filter.setLeafSize(voxel_leaf_size, voxel_leaf_size, voxel_leaf_size);
    voxel_grid_filter.setInputCloud(scan_ptr_);
    voxel_grid_filter.filter(*scan_filter_ptr_);

    for(pcl::PointXYZ point: *scan_filter_ptr_){
        ranges.emplace_back(point.x,point.y,0);
    }

    for (double theta = 0; theta < 360; theta ++) {
        // vector<tf::Vector3> projectedPoses = projectData(theta, ranges);
        rotatedPoses.emplace_back(projectData(theta, ranges, p_x, p_y));
    }

    std::vector<scan_t> scan_ts;
    int theta_steps = 2;
    for(int x = -matching_size;x < matching_size;x++){
        for(int y = -matching_size;y < matching_size;y++){
            for(int theta = 0;theta < 360; theta += theta_steps){
                scan_ts.emplace_back(x,y,theta,theta_steps,fieldMaps.back().getResolution());
            }
        }
    }
    printf("scan_ts size:%d\n",scan_ts.size());
    // clock_t now = clock();
    scoreScanT(scan_ts);
    // cout<<"time1 = "<<double(clock()-now)/CLOCKS_PER_SEC<<"s"<<endl;
    float score_threshold = 0.75;
    
    std::vector<scan_t>::iterator it_ = scan_ts.begin();
    scan_t best_scan_t(0,0,0,1,1);
    best_scan_t.score_ = score_threshold;
    thread t1(&ZysLocalization::calculate_score, this, std::ref(it_), std::ref(best_scan_t), std::ref(scan_ts), score_threshold);
    // thread t2(&ZysLocalization::calculate_score, this, std::ref(it_), std::ref(best_scan_t), std::ref(scan_ts), score_threshold);
    // thread t3(&ZysLocalization::calculate_score, this, std::ref(it_), std::ref(best_scan_t), std::ref(scan_ts), score_threshold);
    calculate_score(it_, best_scan_t, scan_ts, score_threshold);
    t1.join();
    // t2.join();
    // t3.join();
    // const scan_t best_scan_t = BranchAndBound(scan_ts, score_threshold);





    // cout<<"time2 = "<<double(clock()-now)/CLOCKS_PER_SEC<<"s"<<endl;


    // cout<< "fieldMaps[0].getEntry(0, 0):"<<fieldMaps[0].getEntry(0, 0)<<endl;
    // double result_score = 1 - (map_scan_t.begin()->first/(fieldMaps[0].getEntry(0, 0) * rotatedPoses[0].size())) ;
    // cout <<map_scan_t.begin()->first <<" 去掉一个最高分，去掉一个最低分，最终的评分是！！！！！    " <<result_score<<"  分"<<endl;
    // cout << map_scan_t.begin()->first <<" -- "<<map_scan_t.bsegin()->second.origin_x_<<", "<<map_scan_t.begin()->second.origin_y_<<endl;
    // double result_x = MAP_WXGX(map_, int(best_scan_t.origin_x_ + p_x));
    // double result_y = MAP_WYGY(map_, int(best_scan_t.origin_y_ + p_y));
    double result_x = MAP_WXGX(map_, int(best_scan_t.origin_x_ + p_x));
    double result_y = MAP_WYGY(map_, int(best_scan_t.origin_y_ + p_y));
    double result_yaw;
    if(best_scan_t.score_ <= score_threshold) result_yaw = best_scan_t.origin_theta_ + axis_yaw_ / 180.00 * M_PI;
    else result_yaw = best_scan_t.origin_theta_ / 180.00 * M_PI;
     
    std::cout<<best_scan_t.origin_x_<<", "<<best_scan_t.origin_y_<<", "<<p_x<<", "<<p_y<<endl;
    std::cout<<result_x<<", "<<result_y<<", "<<result_yaw<<endl;
    tf::Vector3 position;
    position.setX(result_x);
    position.setY(result_y);
    tf::Quaternion orientation = tf::createQuaternionFromYaw(result_yaw);
    tf::Pose result_pose(orientation, position);

    tf::Stamped<tf::Pose> laser_pose(result_pose, scan_->header.stamp, "map");
    tf::Stamped<tf::Pose> identity(tf::Pose(), scan_->header.stamp, scan_->header.frame_id);
    identity.setIdentity();
    tf::Stamped<tf::Pose> robot_to_laser;
    listener->transformPose(sub_tf_tree_name_, identity, robot_to_laser);

    // geometry_msgs::PoseWithCovarianceStamped msg;
    msg.header.stamp = laser_pose.stamp_;
    msg.header.frame_id = laser_pose.frame_id_;
    tf::poseTFToMsg(laser_pose * robot_to_laser.inverse(), msg.pose.pose);
    msg.pose.covariance.elems[0] = 0.05;
    msg.pose.covariance.elems[7] = 0.05;
    msg.pose.covariance.elems[35] = 0.0025;
    // initial_pos_pub_.publish(msg);
    scan_ = NULL;
    return best_scan_t.score_;
    
}
void ZysLocalization::calculate_score(std::vector<scan_t>::iterator& it_, scan_t& best_scan_t_,std::vector<scan_t> &scan_ts_, float const min_score){
    while(1){
        mutex_it_.lock();
        if(it_ == scan_ts_.end()){
            mutex_it_.unlock();
            return ;
        }
        scan_t scan_t_ = *it_;
        it_++;
        mutex_it_.unlock();
        if (scan_t_.score_ <= min_score) break;
        // printf("\n");
        std::vector<scan_t> higher_scan_ts;
        for (int x_offset : {0, 1}) {
            for (int y_offset : {0, 1}) {
                higher_scan_ts.emplace_back(scan_t_.origin_x_ * 2 + x_offset, scan_t_.origin_y_ * 2 + y_offset,
                                            scan_t_.origin_theta_, scan_t_.theta_scale_, scan_t_.map_scale_/2);
            }
        } 
        scoreScanT(higher_scan_ts);
        scan_t scan_t__ = BranchAndBound(higher_scan_ts, best_scan_t_.score_);

        mutex_base_.lock();
        best_scan_t_ = std::max(best_scan_t_, scan_t__);
        mutex_base_.unlock();
    }
}


void ZysLocalization::scoreScanT(std::vector<scan_t> &scan_ts_){
    for(scan_t& scan_t_:scan_ts_){
        double score = 0.0;
        int const mapindex = log2(scan_t_.map_scale_);
        // size_t const maxX = fieldMaps[mapindex].xSize();
        // size_t const maxY = fieldMaps[mapindex].ySize();
        int x_offset = scan_t_.origin_x_*scan_t_.map_scale_;
        int y_offset = scan_t_.origin_y_*scan_t_.map_scale_;

        for (const tf::Vector3& pose :rotatedPoses[scan_t_.origin_theta_]){
            size_t decimated_x = pose.getX() + x_offset;
            size_t decimated_y = pose.getY() + y_offset;
            score += fieldMaps[mapindex].getEntry(decimated_x, decimated_y);
            
        }
        scan_t_.score_ = 1 - (score / rotatedPoses[0].size() / (fieldMaps[0].max_entry - fieldMaps[0].min_entry)); 
        // scan_t_.score_ = 0.1 + (1 - (score / rotatedPoses[0].size() / fieldMaps[0].max_entry)) * 0.8; 
        // printf(" %f,",scan_t_.score_);
        
    }
    std::sort(scan_ts_.begin(), scan_ts_.end(),
            std::greater<scan_t>());
    
}

scan_t ZysLocalization::BranchAndBound(std::vector<scan_t> &scan_ts_, float const min_score){
    if(scan_ts_.begin()->map_scale_ == 1) return *scan_ts_.begin();
    scan_t best_scan_t(0,0,0,1,1);
    best_scan_t.score_ = min_score;
    for(scan_t& scan_t_:scan_ts_){
        if (scan_t_.score_ <= min_score) break;
        // if (scan_t_.score_ <= best_scan_t.score_) break;
        // if (scan_t_.map_scale_ == fieldMaps.back().getResolution()) printf("\n");
        std::vector<scan_t> higher_scan_ts;
        for (int x_offset : {0, 1}) {
            for (int y_offset : {0, 1}) {
                higher_scan_ts.emplace_back(scan_t_.origin_x_ * 2 + x_offset, scan_t_.origin_y_ * 2 + y_offset,
                                            scan_t_.origin_theta_, scan_t_.theta_scale_, scan_t_.map_scale_/2);
            }
        } 
        // printf("map_scale_:%d, candidate:%f, min:%f, best_score:%f",scan_t_.map_scale_, scan_t_.score_, min_score, best_scan_t.score_);
        // printf("map_scale_:%d, candidate:%f, min:%f, best_score:%f",scan_t_.map_scale_, scan_t_.score_, min_score, best_scan_t.score_);
        scoreScanT(higher_scan_ts);
        best_scan_t = std::max(
            best_scan_t,
            BranchAndBound(higher_scan_ts, best_scan_t.score_));
    }

    return best_scan_t;
}
vector<tf::Vector3> ZysLocalization::projectData(double const& theta, std::vector<tf::Vector3> const& data, size_t const& p_x, size_t const& p_y) const 
{
    vector<tf::Vector3> projectedPoses;
    tf::Vector3 position;
    position.setX(MAP_WXGX(map_, 0));
    position.setY(MAP_WYGY(map_, 0));
    // Since we save the poses for each theta we can compute theta by dividing
    // the poseindex by #theta_steps
    tf::Quaternion orientation = tf::createQuaternionFromYaw(theta / 180 * M_PI);
    tf::Pose pose(orientation, position);

    for (tf::Vector3 point: data) {
        tf::Vector3 projectedPoint = pose * point;
        // printf("projectedPoint:%f, %f    ",projectedPoint.getX(),projectedPoint.getY());
        // projectedPoint.setX(MAP_GXWX(map_, projectedPoint.getX()));
        // projectedPoint.setY(MAP_GYWY(map_, projectedPoint.getY()));
        projectedPoint.setX(MAP_GXWX(map_, projectedPoint.getX())+p_x);
        projectedPoint.setY(MAP_GYWY(map_, projectedPoint.getY())+p_y);
        // printf("projectedPoint:%f, %f    point:%f, %f  \n",projectedPoint.getX(),projectedPoint.getY(),point.getX(),point.getY());
        projectedPoses.push_back(projectedPoint);
    }
    return projectedPoses;
}

// void ZysLocalization::addMapScanT(const scan_t& scan_t_)
// {
//     for (int y = 0; y <= 1; y++) {
//         for (int x = 0; x <= 1; x++) {
//             for (int t = 0; t<=(scan_t_.theta_scale_/2); t+=(scan_t_.theta_scale_/2)){
//                 int theta_ = scan_t_.origin_theta_ + t >= 360 ? scan_t_.origin_theta_ + t -360 :scan_t_.origin_theta_ + t;
//                 scan_ts.push_back(
//                     scan_t(scan_t_.origin_x_ * 2 + x, scan_t_.origin_y_ * 2+ y, theta_, scan_t_.theta_scale_/2, scan_t_.map_scale_/2));
//                 // scan_ts.push_back(
//                 //     scan_t(scan_t_.origin_x_ * 2 + x, scan_t_.origin_y_ * 2+ y, scan_t_.origin_theta_, scan_t_.theta_scale_/2, scan_t_.map_scale_/2));
//             }
//         }
//     }
// }

void ZysLocalization::mapCallback(const nav_msgs::OccupancyGrid& map_msg)
{
    map_ = (map_t*) malloc(sizeof(map_t));

    // Allocate storage for main map
    map_->cells = (map_cell_t*) NULL;
    ROS_ASSERT(map_);
    map_->size_x = map_msg.info.width;
    map_->size_y = map_msg.info.height;
    map_->scale = map_msg.info.resolution;
    map_->origin_x = map_msg.info.origin.position.x +
        (map_->size_x / 2) * map_->scale;
    map_->origin_y = map_msg.info.origin.position.y +
        (map_->size_y / 2) * map_->scale;

    ROS_INFO("Map service data processed.");


    // Convert to amcl map format
    // map_->cells = (map_cell_t*)malloc(
    //         (sizeof(map_cell_t)) * map_->size_x * map_->size_y);
    // ROS_ASSERT(map_->cells);
    
    // for (int i = 0; i < map_->size_x * map_->size_y; ++i) {
    //     if (map_msg.data[i] == 0) {
    //         map_->cells[i].occ_state = -1; // free
    //     } else if (map_msg.data[i] == 100) {
    //         map_->cells[i].occ_state = +1; // occ
    //     } else {
    //         map_->cells[i].occ_state = 0; // unknown
    //     }
    // }
    // double max_occ_dist = 2.0;
    // map_update_cspace(map_, max_occ_dist);

    // ROS_INFO("Map received. Creating coarser likelyhood maps.");

    // LikelyHoodFieldMap fieldmap(map_->size_x, map_->size_y);
    
    // double const z_hit_denom = 2.0 * sigma_hit * sigma_hit;
    // double const z_hit_mult = 1.0 / sqrt(2 * M_PI * sigma_hit);
    // double const randomNoise =  z_rand / cloud_range_max_;

    // cout<<"z_hit_denom: "<<z_hit_denom<<endl;
    // cout<<"z_hit_mult: "<<z_hit_mult<<endl;
    // cout<<"randomNoise: "<<randomNoise<<endl;
    // for (int y = 0; y < fieldmap.ySize(); ++y) {
    //     for (int x = 0; x < fieldmap.xSize(); ++x) {
    //         int pose_x = fieldmap.valueWithoutResolution(x);
    //         int pose_y = fieldmap.valueWithoutResolution(y);
    //         double distToObstacle = map_->cells[MAP_INDEX(map_,pose_x,pose_y)].occ_dist;
    //         double gaussNoise = z_hit * z_hit_mult *
    //             exp(-(distToObstacle * distToObstacle) / z_hit_denom);
    //         fieldmap.setEntry(-log(gaussNoise + randomNoise), x, y);
                
    //         // if (y == 500) printf("y: %f, %f, %f, %f\n",-log(gaussNoise + randomNoise),gaussNoise,exp(-(distToObstacle * distToObstacle) / z_hit_denom),distToObstacle);
    //     }
    // }

    // vector<double>::iterator max = std::max_element(fieldmap.grid.begin(), fieldmap.grid.end());
    // vector<double>::iterator min = std::min_element(fieldmap.grid.begin(), fieldmap.grid.end());
    // cout<<"max: "<<*max<<"  min: "<<*min<<endl;
    // fieldmap.max_entry = *max;
    // fieldmap.min_entry = *min;
    // fieldMaps.push_back(fieldmap);

    // while (fieldMaps.size() < 5) {
    //     // printf("getResolution:%d\n",fieldMaps.back().getResolution());
    //     constructCoarserMap(fieldMaps.back());
        
    // } 
    // if(getmap_){
    //     for (auto const fieldMap__:fieldMaps) {
    //         fieldMap__.toPGM("/home/zhang/map/akmap_"+boost::lexical_cast<std::string>(fieldMap__.getResolution()) + ".pgm",*max,*min);
    //     }
    // }
    // map_ok = true;
    // ROS_INFO("Map data processed.");
}
void ZysLocalization::mapNameCallback(const std_msgs::String& mapName_msg){

    // printf("map_name1:%s\n",mapName_msg.data.c_str());
    if(mapName_ != mapName_msg.data){
        mapName_ = mapName_msg.data;
        printf("map_name:%s\n",mapName_.c_str());
        fieldMaps.clear();
        while (fieldMaps.size() < 5) {
            cv::Mat image;
            string mapName__;
            // string string_num = '_' + to_string(pow(2,fieldMaps.size()));
            for(int i = 0;i<mapName_.size()-5;i++){
                mapName__.push_back(mapName_[i]);
            }
            // string string_num = to_string(int(pow(2,fieldMaps.size())));
            mapName__ = mapName__ + "_" + to_string(int(pow(2,fieldMaps.size()))) + ".pgm";
            image = cv::imread(mapName__,cv::IMREAD_GRAYSCALE);
            if(image.data== nullptr)//nullptr是c++11新出现的空指针常量
            {

                printf("map_name:%s\n",mapName__.c_str());
                ROS_ERROR("图片文件不存在");
                return ;
            }
            LikelyHoodFieldMap map(image.cols, image.rows, pow(2,fieldMaps.size()));
            map.max_entry = 255;
            map.min_entry = 0;
            // for (int y = max_y; y > 0; --y) {
            //     for (int x = 0; x < max_x; ++x) {
            //         int output = (getEntry(x, y-1)-min_) * maxGray;
            //         fputc(output, out);
            //     }
            // }
            // cout <<"image.rows:"<<image.rows<<" image.cols:"<<image.cols<<endl;
            int i = 0;
            for (int y = image.rows; y > 0; --y) {
                for (int x = 0; x < image.cols; ++x) {
                    map.setEntry(double(image.data[i]), x, y-1);
                    i++;
                }
            }
            // for(int i = 0; i < image.rows*image.cols; ++i) {
            //     map.grid[i] = double(image.data[i]);
            // }
            fieldMaps.push_back(map);
        } 
        map_ok = true;
        if(getmap_){
            for (auto const fieldMap__:fieldMaps) {
                fieldMap__.toPGM("/home/zhang/map/akmap_"+boost::lexical_cast<std::string>(fieldMap__.getResolution()) + "_.pgm",255,0);
            }
        }
        ROS_INFO("Map data processed.");
    }

}

void ZysLocalization::constructCoarserMap(LikelyHoodFieldMap const& source) {
    const int width = 2 * source.getResolution();
    LikelyHoodFieldMap map(fieldMaps[0].xSize(), fieldMaps[0].ySize(), 2 * source.getResolution());
    // LikelyHoodFieldMap map(fieldMaps[0].xSize() + width - 1, fieldMaps[0].ySize() + width - 1, width);
    map.max_entry = fieldMaps[0].max_entry;
    map.min_entry = fieldMaps[0].min_entry;
    LikelyHoodFieldMap map1(fieldMaps[0].xSize(), fieldMaps[0].ySize(), 2 * source.getResolution());
    map1.max_entry = fieldMaps[0].max_entry;
    map1.min_entry = fieldMaps[0].min_entry;

    // vector<size_t> xCoords;
    // // for(size_t i = -width; i < width;i++){
    // //     xCoords.push_back(i);
    // // }
    // xCoords.push_back(-1);
    // xCoords.push_back(0);
    // xCoords.push_back(1);
    // vector<size_t> yCoords;
    // // for(size_t i = -width; i < width;i++){
    // //     yCoords.push_back(i);
    // // }
    // yCoords.push_back(-1);
    // yCoords.push_back(0);
    // yCoords.push_back(1);
    // for (size_t y = 1; y < source.ySize(); y++) {
    //     for (size_t x = 1; x < source.xSize(); x++) {
    //         // Convolution kernel of 3: Project a 3x3 grid around the original
    //         // point and compute the maximum value
    //         vector<double> values;
    //         for (size_t const& x_offset: xCoords) {
    //         	for (size_t const& y_offset: yCoords) {
    //                 // Note that values < 0 overflow to max size_t value
    //         		size_t x_coord = x + x_offset*map.getResolution();
    //         		size_t y_coord = y + y_offset*map.getResolution();
    //                 if (x_coord < source.xSize() && y_coord < source.ySize()) {
    //                     values.push_back(source.getEntry(x_coord,y_coord));
    //                 }
    //             }
    //         }
    //         // Note that the max element has the lowest log probability, thus
    //         // we check for the smallest element
    //         vector<double>::iterator max = std::min_element(values.begin(), values.end());
    //         map.setEntry(*max, x, y);
    //         // map.setEntry(*max, x-1, y);
    //         // map.setEntry(*max, x, y-1);
    //         // map.setEntry(*max, x-1, y-1);
    //         // map.setEntry(*max, x+1, y);
    //     }
    // }

    // 

    for (int y = 0; y < fieldMaps[0].ySize(); ++y) {
        SlidingWindowMaximum current_values;
        for (int x = 0; x < width; ++x) {
            current_values.AddValue(fieldMaps[0].getEntry(x,y));
            // for(double i:current_values.non_ascending_maxima_){
            //     cout <<i<<" ";
            // }
            // cout<<endl;
        }

        for (int x = 0; x < map.xSize() - width; ++x) {
            map1.setEntry(current_values.GetMaximum(), x, y);
            current_values.RemoveValue(fieldMaps[0].getEntry(x - width,y));
            current_values.AddValue(fieldMaps[0].getEntry(x + width,y));
            // for(double i:current_values.non_ascending_maxima_){
            //     cout <<i<<" ";
            // }
            // cout<<endl;
        }
        for (int x = map.xSize() - width; x < map.xSize(); ++x) {
            map1.setEntry(current_values.GetMaximum(), x, y);
            current_values.RemoveValue(fieldMaps[0].getEntry(x - width,y));
            // for(double i:current_values.non_ascending_maxima_){
            //     cout <<i<<" ";
            // }
            // cout<<endl;
        }
        // printf("current_values%d.size:%d\n",y,current_values.non_ascending_maxima_.size());
        // if(current_values.non_ascending_maxima_.size()>0){
        //     while (!current_values.non_ascending_maxima_.empty()){
        //         cout <<current_values.non_ascending_maxima_.front()<<" ";
        //         current_values.non_ascending_maxima_.pop_front();
        //     }

        //     cout<<endl;
        // }
    }

    for (int x = 0; x < fieldMaps[0].xSize(); x++) {
        SlidingWindowMaximum current_values;
        for (int y = 0; y < width; y++) {
            current_values.AddValue(map1.getEntry(x,y));
            // cout <<map.getEntry(x,y - width)<<"||";
            // for(double i:current_values.non_ascending_maxima_){
            //     cout <<i<<" ";
            // }
            // cout<<endl;
        }
        for (int y = 0; y < map.ySize() - width; y++) {
            map.setEntry(current_values.GetMaximum(), x, y);
            current_values.RemoveValue(map1.getEntry(x,y - width));
            current_values.AddValue(map1.getEntry(x, y + width));
            // cout <<map.getEntry(x,y - width)<<"--"<<map.getEntry(x, y + width)<<"||";
            // for(double i:current_values.non_ascending_maxima_){
            //     cout <<i<<" ";
            // }
            // cout<<endl;
        }
        for (int y = map.ySize() - width; y < map.ySize(); y++) {
            map.setEntry(current_values.GetMaximum(), x, y);
            current_values.RemoveValue(map1.getEntry(x,y - width));
        //     cout <<map.getEntry(x,y - width)<<"--";
        //     for(double i:current_values.non_ascending_maxima_){
        //         cout <<i<<" ";
        //     }
        //     cout<<endl;
        }
        // printf("current_values%d.size:%d\n",x,current_values.non_ascending_maxima_.size());
    }

    // for (int y = 0; y !=  map.ySize(); ++y) {
    //     SlidingWindowMaximum current_values;
    //     current_values.AddValue(fieldMaps[0].getEntry(0,y));
    //     for (int x = -width + 1; x != 0; ++x) {
    //         map.setEntry(current_values.GetMaximum(), x + width - 1, y);
    //         if (x + width < fieldMaps[0].xSize()) {
    //             current_values.AddValue(fieldMaps[0].getEntry(x + width,y)/10.0);
    //             // cout <<fieldMaps[0].getEntry(x + width - 1, y)<<"||";
    //             // for(double i:current_values.non_ascending_maxima_){
    //             //     cout <<i<<" ";
    //             // }
    //             // cout<<endl;
    //         }
    //     }
    //     for (int x = 0; x < fieldMaps[0].xSize() - width; ++x) {
    //         // intermediate[x + width - 1 + y * stride] = current_values.GetMaximum();
    //         map.setEntry(current_values.GetMaximum(), x + width - 1, y);
    //         // current_values.RemoveValue(1.f - std::abs(grid.GetCorrespondenceCost(Eigen::Array2i(x, y))));
    //         // current_values.AddValue(1.f - std::abs(grid.GetCorrespondenceCost(Eigen::Array2i(x + width, y))));
    //         current_values.RemoveValue(fieldMaps[0].getEntry(x,y)*10.0);
    //         current_values.AddValue(fieldMaps[0].getEntry(x + width, y)*10.0);

    //         // cout <<fieldMaps[0].getEntry(x,y)<<"--"<<fieldMaps[0].getEntry(x + width, y)<<"||";
    //         // for(double i:current_values.non_ascending_maxima_){
    //         //     cout <<i<<" ";
    //         // }
    //         // cout<<endl;
    //     }
    //     for (int x = std::max(int(fieldMaps[0].xSize()) - width, 0); x != fieldMaps[0].xSize(); ++x) {
    //         // intermediate[x + width - 1 + y * stride] = current_values.GetMaximum();
    //         map.setEntry(current_values.GetMaximum(), x + width - 1, y);
    //         // current_values.RemoveValue(1.f - std::abs(grid.GetCorrespondenceCost(Eigen::Array2i(x, y))));
    //         current_values.RemoveValue(fieldMaps[0].getEntry(x,y)*10.0);
    //         // cout <<fieldMaps[0].getEntry(x,y)<<"--";
    //         // for(double i:current_values.non_ascending_maxima_){
    //         //     cout <<i<<" ";
    //         // }
    //         // cout<<endl;
    //     }
    // }
    // // For each (x, y), we compute the maximum probability in the width x width
    // // region starting at each (x, y) and precompute the resulting bound on the
    // // score.
    // for (int x = 0; x != map.xSize(); ++x) {
    //     SlidingWindowMaximum current_values;
    //     // current_values.AddValue(intermediate[x]);
    //     current_values.AddValue(map.getEntry(x, 0));
    //     for (int y = -width + 1; y != 0; ++y) {
    //         // cells_[x + (y + width - 1) * stride] =ComputeCellValue(current_values.GetMaximum());
    //         map.setEntry(current_values.GetMaximum(), x, y + width - 1);
    //         if (y + width < fieldMaps[0].ySize()) {
    //             // current_values.AddValue(intermediate[x + (y + width) * stride]);
    //             current_values.AddValue(map.getEntry(x, y + width)/10.0);
    //         }
    //     }
    //     for (int y = 0; y < fieldMaps[0].xSize() - width; ++y) {
    //         // cells_[x + (y + width - 1) * stride] = ComputeCellValue(current_values.GetMaximum());
    //         map.setEntry(current_values.GetMaximum(), x, y + width - 1);
    //         // current_values.RemoveValue(intermediate[x + y * stride]);
    //         // current_values.AddValue(intermediate[x + (y + width) * stride]);
    //         current_values.RemoveValue(map.getEntry(x,y)/10.0);
    //         current_values.AddValue(map.getEntry(x, y + width)/10.0);
    //     }
    //     for (int y = std::max(int(fieldMaps[0].xSize()) - width, 0); y != fieldMaps[0].xSize(); ++y) {
    //         // cells_[x + (y + width - 1) * stride] =ComputeCellValue(current_values.GetMaximum());
    //         map.setEntry(current_values.GetMaximum(), x, y + width - 1);
    //         // current_values.RemoveValue(intermediate[x + y * stride]);
    //         current_values.RemoveValue(map.getEntry(x,y)/10.0);
    //     }
    // }




    fieldMaps.push_back(map);
}

void ZysLocalization::scanCallback(const sensor_msgs::LaserScan::ConstPtr& scan_msg)
{
    scan_ = scan_msg;
}

void ZysLocalization::imuCallback(const sensor_msgs::ImuConstPtr& imu_msg)
{
    imu_yaw_ = tf::getYaw(imu_msg->orientation);
    imu_ok = true;
}