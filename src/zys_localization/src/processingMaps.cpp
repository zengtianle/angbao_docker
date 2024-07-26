#include <ros/ros.h>
#include <iostream>
#include <fstream>

#include <sys/types.h>
#include <dirent.h>
#include <vector>
#include <string.h>

#include "zys_localization/map.h"
#include "zys_localization/field_map.h"
#include<opencv2/opencv.hpp>
#include "yaml-cpp/yaml.h"
#include "std_msgs/Int8.h"

#ifdef HAVE_NEW_YAMLCPP
// The >> operator disappeared in yaml-cpp 0.5, so this function is
// added to provide support for code written under the yaml-cpp 0.3 API.
template<typename T>
void operator >> (const YAML::Node& node, T& i)
{
  i = node.as<T>();
}
#endif

using namespace std;


double sigma_hit, z_rand, z_hit, cloud_range_max_;
int processing_size;
std::vector<LikelyHoodFieldMap> fieldMaps;
void constructCoarserMap(LikelyHoodFieldMap const& source);

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
//   void CheckIsEmpty() const { CHECK_EQ(non_ascending_maxima_.size(), 0); }

 private:
  std::deque<double> non_ascending_maxima_;
  // Maximum of the current sliding window at the front. Then the maximum of the
  // remaining window that came after this values first occurrence, and so on.

};


bool getMultiresolutionMaps(string filename){
    fieldMaps.clear();
    std::ifstream fin((filename + ".yaml").c_str());
    if (fin.fail()) {
        ROS_ERROR("Map_server could not open %s", filename.c_str());
        exit(-1);
    }
    #ifdef HAVE_NEW_YAMLCPP
        // The document loading process changed in yaml-cpp 0.5.
            YAML::Node doc = YAML::Load(fin);
    #else
            YAML::Parser parser(fin);
            YAML::Node doc;
            parser.GetNextDocument(doc);
    #endif
    // doc["resolution"] >> res;
    // cout<<"resolution: "<<doc["resolution"]<<endl;

    cv::Mat image;
    image = cv::imread(filename+".pgm",cv::IMREAD_GRAYSCALE);
    if(image.data== nullptr)//nullptr是c++11新出现的空指针常量
    {
        ROS_INFO("图片文件不存在");
        return 0;
    }
    // cout<<image<<endl;
    map_t* map_ = (map_t*) malloc(sizeof(map_t));

    // // Allocate storage for main map
    map_->cells = (map_cell_t*) NULL;
    ROS_ASSERT(map_);
    map_->size_x = image.cols;
    map_->size_y = image.rows;
    // map_->scale = doc["resolution"];
    doc["resolution"] >> map_->scale;

    // Convert to amcl map format
    map_->cells = (map_cell_t*)malloc(
            (sizeof(map_cell_t)) * map_->size_x * map_->size_y);
    ROS_ASSERT(map_->cells);
    
    for (int i = 0; i < map_->size_x * map_->size_y; ++i) {
        // if (image.data[i] <= 25) {
        //     map_->cells[i].occ_state = -1; // free
        // } else if (image.data[i] > 210) {
        //     map_->cells[i].occ_state = +1; // occ
        // } else {
        //     map_->cells[i].occ_state = 0; // unknown
        // }
        if (image.data[i] > 210) {
            map_->cells[i].occ_state = -1; // free
        } else if (image.data[i] <= 165) {
            map_->cells[i].occ_state = +1; // occ
        } else {
            map_->cells[i].occ_state = 0; // unknown
        }
    }
    double max_occ_dist = 2.0;
    map_update_cspace(map_, max_occ_dist);

    ROS_INFO("Map received. Creating coarser likelyhood maps.");



    LikelyHoodFieldMap fieldmap(image.cols, image.rows);
    
    double const z_hit_denom = 2.0 * sigma_hit * sigma_hit;
    double const z_hit_mult = 1.0 / sqrt(2 * M_PI * sigma_hit);
    double const randomNoise =  z_rand / cloud_range_max_;

    cout<<"z_hit_denom: "<<z_hit_denom<<endl;
    cout<<"z_hit_mult: "<<z_hit_mult<<endl;
    cout<<"randomNoise: "<<randomNoise<<endl;
    for (int y = 0; y < fieldmap.ySize(); ++y) {
        for (int x = 0; x < fieldmap.xSize(); ++x) {
            int pose_x = fieldmap.valueWithoutResolution(x);
            int pose_y = fieldmap.valueWithoutResolution(y);
            double distToObstacle = map_->cells[MAP_INDEX(map_,pose_x,pose_y)].occ_dist;
            double gaussNoise = z_hit * z_hit_mult *
                exp(-(distToObstacle * distToObstacle) / z_hit_denom);
            fieldmap.setEntry(-log(gaussNoise + randomNoise), x, y);
                
            // if (y == 500) printf("y: %f, %f, %f, %f\n",-log(gaussNoise + randomNoise),gaussNoise,exp(-(distToObstacle * distToObstacle) / z_hit_denom),distToObstacle);
        }
    }
    vector<double>::iterator max = std::max_element(fieldmap.grid.begin(), fieldmap.grid.end());
    vector<double>::iterator min = std::min_element(fieldmap.grid.begin(), fieldmap.grid.end());
    cout<<"max: "<<*max<<"  min: "<<*min<<endl;
    fieldmap.max_entry = *max;
    fieldmap.min_entry = *min;
    fieldMaps.push_back(fieldmap);


    while (fieldMaps.size() < processing_size)  constructCoarserMap(fieldMaps.back());
    
    for (auto const fieldMap__:fieldMaps) {
        fieldMap__.toPGM(filename +"_"+ to_string(fieldMap__.getResolution()) + ".pgm",*max,*min);
    }


    delete map_;
    return 1;
    
}
void constructCoarserMap(LikelyHoodFieldMap const& source) {
    const int width = 2 * source.getResolution();
    LikelyHoodFieldMap map(fieldMaps[0].xSize(), fieldMaps[0].ySize(), 2 * source.getResolution());
    map.max_entry = fieldMaps[0].max_entry;
    map.min_entry = fieldMaps[0].min_entry;
    LikelyHoodFieldMap map1(fieldMaps[0].xSize(), fieldMaps[0].ySize(), 2 * source.getResolution());
    map1.max_entry = fieldMaps[0].max_entry;
    map1.min_entry = fieldMaps[0].min_entry;

    for (int y = 0; y < fieldMaps[0].ySize(); ++y) {
        SlidingWindowMaximum current_values;
        for (int x = 0; x < width; ++x) {
            current_values.AddValue(fieldMaps[0].getEntry(x,y));
        }

        for (int x = 0; x < map.xSize() - width; ++x) {
            map1.setEntry(current_values.GetMaximum(), x, y);
            current_values.RemoveValue(fieldMaps[0].getEntry(x - width,y));
            current_values.AddValue(fieldMaps[0].getEntry(x + width,y));
        }
        for (int x = map.xSize() - width; x < map.xSize(); ++x) {
            map1.setEntry(current_values.GetMaximum(), x, y);
            current_values.RemoveValue(fieldMaps[0].getEntry(x - width,y));
        }
    }

    for (int x = 0; x < fieldMaps[0].xSize(); x++) {
        SlidingWindowMaximum current_values;
        for (int y = 0; y < width; y++) {
            current_values.AddValue(map1.getEntry(x,y));
        }
        for (int y = 0; y < map.ySize() - width; y++) {
            map.setEntry(current_values.GetMaximum(), x, y);
            current_values.RemoveValue(map1.getEntry(x,y - width));
            current_values.AddValue(map1.getEntry(x, y + width));
        }
        for (int y = map.ySize() - width; y < map.ySize(); y++) {
            map.setEntry(current_values.GetMaximum(), x, y);
            current_values.RemoveValue(map1.getEntry(x,y - width));
        }
    }
    fieldMaps.push_back(map);
}

void GetFileNames(string path,vector<string>& filenames)
{
    DIR *pDir;
    struct dirent* ptr;
    if(!(pDir = opendir(path.c_str())))
        return;
    while((ptr = readdir(pDir))!=0) {
        if (strcmp(ptr->d_name, ".") != 0 && strcmp(ptr->d_name, "..") != 0)
            //filenames.push_back(path + "/" + ptr->d_name);
            filenames.push_back(ptr->d_name);
    }
    closedir(pDir);
}


int main(int argc, char **argv)
{
    ros::init(argc, argv, "processingMaps");
    ros::NodeHandle nh_, nh_private_("~");
    ROS_INFO("processingMaps start!!!");

    string path;
    nh_private_.param("laser_sigma_hit", sigma_hit, 0.2);
	nh_private_.param("laser_z_rand", z_rand, 0.05);
	nh_private_.param("laser_z_hit", z_hit, 0.95);
    nh_private_.param("cloud_range_max", cloud_range_max_, 50.0);
    nh_private_.param("processing_size", processing_size, 5);
    nh_private_.param<std::string>("file_path", path, "/home/ob/.ob/map/");
    // getMultiresolutionMaps("/home/zhang/teb_ws/src/robot_launch/worlds/willow");

    //0: 开始 1： 成功 2：失败
    ros::Publisher pub_state_ = nh_.advertise<std_msgs::Int8>("processingMaps_state", 1, true);



    std_msgs::Int8 state_;
    state_.data = 0;
    pub_state_.publish(state_);
    pub_state_.publish(state_);
    pub_state_.publish(state_);
    ROS_INFO("processingMaps state data:%d", state_.data);
    ros::Rate rate(20);

    vector<string> file_name;
    // string path = "../folder";
    // string path = "/home/zhang/fsdownload/map/";
    string re_name_font = "OB_floor_location_";
    string re_name_end = ".yaml";
    ofstream outfile;
    // outfile.open("sum.txt");
    GetFileNames(path, file_name); 
    for(string s:file_name){
        if(s.size() < 18) continue;
        if(s.substr(0,18) != re_name_font or s.substr(s.size()-5,s.size()) != re_name_end) continue;
        cout<<path+s.substr(0,s.size()-5)<<endl;
        if(!getMultiresolutionMaps(path+s.substr(0,s.size()-5))){
            ROS_INFO("processingMaps state data:%d", state_.data);
            state_.data = 2;
            while(ros::ok()){
                pub_state_.publish(state_);
                rate.sleep();
                ros::spinOnce();
            }
        };
        cout<<s<<endl<<endl;
    }

    ROS_INFO("processingMaps state data:%d", state_.data);
    state_.data = 1;
    while(ros::ok()){
        pub_state_.publish(state_);
        rate.sleep();
        ros::spinOnce();
    }


    return 0;
}
