// Copyright (C) 2018 Dongsheng Yang <ydsf16@buaa.edu.cn>
//(Biologically Inspired Mobile Robot Laboratory, Robotics Institute, Beihang University)

#ifndef GRID_MAP_H
#define GRID_MAP_H

#include <Eigen/Geometry>
#include <Eigen/Dense>
#include <opencv2/opencv.hpp>
#include <opencv2/core/eigen.hpp>

using namespace Eigen;
using namespace std;

class GridMap{
public:
    GridMap(const int& size_x, const int& size_y, const int& init_x, const int& init_y, const double& cell_size );
    bool setGridBel(const double& x, const double& y, const double& bel);
    bool getGridBel ( const double& x, const double& y, double& bel);
    bool setGridLogBel(const double& x, const double& y, const double& log_bel);
    bool getGridLogBel(const double& x, const double& y, double& log_bel);
    double getCellSize();
   
    //void toRosOccGridMap(const std::string& frame_id, nav_msgs::OccupancyGrid& occ_grid); //转换到Ros栅格地图的消息
    cv::Mat toCvMat(); //转换到Opencv的图片格式
    void saveMap(const std::string& img_dir, const std::string& cfg_dir); // 保存地图，图片加配置文件的形式
    void loadMap(const std::string& img_dir, const std::string& cfg_dir); 
    
private:
    bool getIdx(const double& x, const double& y, Eigen::Vector2i& idx);
    
private:
    int size_x_, size_y_, init_x_, init_y_;
    double cell_size_;
    Eigen::MatrixXd bel_data_;
    Eigen::MatrixXd m_one_, m_show_;
    
};// class GridMap


#endif