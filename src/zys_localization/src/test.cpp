/*
 * @Description: 
 * @Version: 1.0
 * @Autor: Yongsheng_Zhang
 * @Date: 2021-06-29 14:11:58
 * @LastEditors: Yongsheng_Zhang
 * @LastEditTime: 2021-06-29 14:11:59
 */
#include<stdio.h>
#include<iostream>
#include<sys/time.h>　　　//引入头文件
#include<opencv2/opencv.hpp>
#include<string>
#include<vector>
// #include <opencv2/core/core.hpp>
// #include <opencv2/highgui/highgui.hpp>
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

void time_test(){
    struct timeval t1,t2;
    double timeuse;
    int map_scale_ = 4, origin_x_ = 102, origin_y_ = 302;
    int resolution = 32,x_num = 17000, y_num = 529;
    size_t p_x = 1, p_y = 1;
    gettimeofday(&t1,NULL);

    for(int x = 0;x<x_num;x++){
        for(int y = 0;y<y_num;y++){
            size_t decimated_x = (x + p_x)/ map_scale_ + origin_x_;
            size_t decimated_y = (y + p_y)/ map_scale_ + origin_y_;
        }
    }

    //不加除法能节省90%的计算量


    gettimeofday(&t2,NULL);
    timeuse = (t2.tv_sec - t1.tv_sec) + (double)(t2.tv_usec - t1.tv_usec)/1000000.0;
    cout<<"time = "<<timeuse<<endl;  //输出时间（单位：ｓ）




    gettimeofday(&t1,NULL);

    for(int x = 0;x<x_num;x++){
        for(int y = 0;y<y_num;y++){
            size_t decimated_x = (x + p_x) + origin_x_;
            size_t decimated_y = (y + p_y) + origin_y_;
        }
    }

    //不加除法能节省90%的计算量


    gettimeofday(&t2,NULL);
    timeuse = (t2.tv_sec - t1.tv_sec) + (double)(t2.tv_usec - t1.tv_usec)/1000000.0;
    cout<<"time = "<<timeuse<<endl;  //输出时间（单位：ｓ）


    gettimeofday(&t1,NULL);

    for(int x = 0;x<x_num;x++){
        for(int y = 0;y<y_num;y++){
            size_t decimated_x = (x + p_x) + origin_x_*resolution;
            size_t decimated_y = (y + p_y) + origin_y_*resolution;
        }
    }

    //不加除法能节省90%的计算量


    gettimeofday(&t2,NULL);
    timeuse = (t2.tv_sec - t1.tv_sec) + (double)(t2.tv_usec - t1.tv_usec)/1000000.0;
    cout<<"time = "<<timeuse<<endl;  //输出时间（单位：ｓ）






    gettimeofday(&t1,NULL);

    for(int x = 0;x<x_num;x++){
        for(int y = 0;y<y_num;y++){
            size_t decimated_x = (x) + origin_x_*resolution;
            size_t decimated_y = (y) + origin_y_*resolution;
        }
    }

    //不加除法能节省90%的计算量


    gettimeofday(&t2,NULL);
    timeuse = (t2.tv_sec - t1.tv_sec) + (double)(t2.tv_usec - t1.tv_usec)/1000000.0;
    cout<<"time = "<<timeuse<<endl;  //输出时间（单位：ｓ）

    gettimeofday(&t1,NULL);

    for(int x = 0;x<x_num;x++){
        for(int y = 0;y<y_num;y++){
            int decimated_x = (x) + origin_x_*resolution;
            int decimated_y = (y) + origin_y_*resolution;
        }
    }

    //不加除法能节省90%的计算量


    gettimeofday(&t2,NULL);
    timeuse = (t2.tv_sec - t1.tv_sec) + (double)(t2.tv_usec - t1.tv_usec)/1000000.0;
    cout<<"time = "<<timeuse<<endl;  //输出时间（单位：ｓ）


    gettimeofday(&t1,NULL);

    for(int x = 0;x<x_num;x++){
        for(int y = 0;y<y_num;y++){
            int decimated_x = (x) + origin_x_<<5;
            int decimated_y = (y) + origin_y_<<5;
        }
    }

    //不加除法能节省90%的计算量


    gettimeofday(&t2,NULL);
    timeuse = (t2.tv_sec - t1.tv_sec) + (double)(t2.tv_usec - t1.tv_usec)/1000000.0;
    cout<<"time = "<<timeuse<<endl;  //输出时间（单位：ｓ）



    gettimeofday(&t1,NULL);

    for(int x = 0;x<x_num;x++){
        for(int y = 0;y<y_num;y++){
            int decimated_x = (x) + origin_x_*0.05;
            int decimated_y = (y) + origin_y_*0.05;
        }
    }

    //不加除法能节省90%的计算量


    gettimeofday(&t2,NULL);
    timeuse = (t2.tv_sec - t1.tv_sec) + (double)(t2.tv_usec - t1.tv_usec)/1000000.0;
    cout<<"time = "<<timeuse<<endl;  //输出时间（单位：ｓ）




        gettimeofday(&t1,NULL);

    for(int x = 0;x<x_num;x++){
        for(int y = 0;y<y_num;y++){
            int decimated_x = (x) + origin_x_/20;
            int decimated_y = (y) + origin_y_/20;
        }
    }

    //不加除法能节省90%的计算量


    gettimeofday(&t2,NULL);
    timeuse = (t2.tv_sec - t1.tv_sec) + (double)(t2.tv_usec - t1.tv_usec)/1000000.0;
    cout<<"time = "<<timeuse<<endl;  //输出时间（单位：ｓ）
}

void maps_test(){
     cv::Mat image;
    // image = cv::imread("/home/zhang/teb_ws/src/teb_local_planner_tutorials-kinetic-devel/maps/maze.png");
    image = cv::imread("/home/zhang/map/akmap4.pgm",cv::IMREAD_GRAYSCALE);
    if(image.data== nullptr)//nullptr是c++11新出现的空指针常量
    {
        cerr<<"图片文件不存在"<<endl;
        return ;
    }else
        cout<<image<<endl;//你会发现图片就是一个矩阵
        // 遍历每个像素
    //之所以用y这个名字表示行是因为图片的坐标系中行号就是y
    // for (size_t y = 0; y < image.rows; ++y) {

    //     // 获取行指针，之所以用char的原因是因为颜色值是1-256用char能放得下
    //     // ptr是pointer的缩写
    //     unsigned char* row_ptr= image.ptr<unsigned char>(y);
    //     for (size_t x = 0; x < image.cols; ++x) {
    //         //这是获得像素数据数组的头指针，注意像素数据可能会有多个通道所以才需要用数组存储
    //         unsigned char* data_ptr = &row_ptr[x*image.channels()];
    //         //对当前像素逐个通道输出颜色值
    //         for (int i = 0; i < image.channels(); ++i) {
    //             cout<< int(data_ptr[i]);
    //         }
    //     }
    // }
    for(int i = 0; i < image.rows*image.cols; ++i) {
        cout<< int(image.data[i])<<" ";
    }

    string filename = "/home/zhang/map/akmap4_.pgm";
    cout << "Writing Likelihood Field Map to " << filename << "..." <<endl;
    FILE* out = fopen(filename.c_str(), "w");
    if (!out) { 
        std::cout << "Could not save likelihood field map." << endl;
        return ;
    }
    // int maxGray = 255/(max_-min_);

    fprintf(out, "P5\n%d %d\n%d\n", (int) image.cols, (int) image.rows, 255);
    std::cout<<"--------------------"<<endl;
    // for (int y = max_y; y > 0; --y) {

    //     for (int x = 0; x < max_x; ++x) {
    //         int output = (getEntry(x, y-1)-min_) * maxGray;
    //         fputc(output, out);
    //     }
    // }
    for(int i = 0; i < image.rows*image.cols; ++i) {
        fputc(int(image.data[i]), out);
    }
    // for (int y = max_y; y > 0; --y) {vector_test()
    //     for (int x = 0; x < max_x; ++x) {
    //         int output = (getEntry(x, y-1)-min_) * maxGray;
    //         fputc(output, out);
    //     }
    // }
    fclose(out);
    cout << "... Finished!" << endl;
    int num = 1;
    string map_name = "/home/zhang/map/akmap4.pgm";
    string string_num = '_' + to_string(num);
    map_name.insert(map_name.size()-4, string_num.c_str());
    
    cout << map_name << endl;


}
void vector_test(){
    vector<scan_t> a;
    a.emplace_back(0,0,0,1,1);
    a.emplace_back(0,1,0,1,1);
    a.emplace_back(1,0,0,1,1);
    a.emplace_back(1,1,0,1,1);
    // std::vector<scan_t>::iterator it_ = a.begin();
    int i =0;
    while(1){
        cout<<(a.begin()+i)->origin_x_<<" "<<(a.begin()+i)->origin_y_<<" "<<i<<endl;
        
        if(i >= a.size()-1) return ;
        i++;
    }
}

int main(int argc, char **argv)
{
    

    // time_test();
    // maps_test();
    vector_test();




    return 0;

}