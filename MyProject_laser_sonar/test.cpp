#include <iostream>
#include <sstream>
#include <fstream>
#include <string>

#include <Eigen/Core>
#include <Eigen/StdVector>
#include <Eigen/Geometry>

#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>

#include <pcl/registration/ndt.h>
#include <pcl/filters/approximate_voxel_grid.h>

#include <pcl/visualization/pcl_visualizer.h>
#include <boost/thread/thread.hpp>

#include <pcl/filters/extract_indices.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/filters/conditional_removal.h>

#include <pcl/segmentation/sac_segmentation.h>
#include <pcl/segmentation/extract_clusters.h>
#include <pcl/segmentation/conditional_euclidean_clustering.h>

using namespace std;

//只和前10帧直接拼起来的地图做匹配
//#define FILE_NUMBER 441
#define FILE_NUMBER 1348

//std::string pcd_name("/media/xjh/69ccdb5f-8ea4-45ba-8ad1-d07865577b2b/xjh/sonar_laser_date/20170615_02/laser");
//std::string sonar_name("/media/xjh/69ccdb5f-8ea4-45ba-8ad1-d07865577b2b/xjh/sonar_laser_date/20170615_02/sonar");

std::string pcd_name("/home/pygmalionchen/SLAM_Prj/SLAM_Data/sonar_laser_date/20170615_02/laser");
////std::string sonar_name("/home/pygmalionchen/SLAM_Prj/SLAM_Data/sonar_laser_date/20170615_02/sonar");

//std::string pcd_name("/home/pygmalionchen/SLAM_Prj/SLAM_Data/Bi_output_data/Bi_pcd_data_");

vector<pcl::PointCloud<pcl::PointXYZ>> point_buffer;
vector<Eigen::Matrix4f> pose_buffer;    //R.T.
pcl::PointCloud<pcl::PointXYZ> PointMap;
pcl::PointCloud<pcl::PointXYZ> PointCurrent;
pcl::PointCloud<pcl::PointXYZ> PointLast;

Eigen::Matrix4f TransformCurrent;
Eigen::Matrix4f TransformLast;

// simply see the error level of the system.
float get_Z_ave()
{
    float z=0;
    for(int i=0; i<pose_buffer.size(); i++)
    {
        z += fabs(pose_buffer[i](2,3));//sum the z and get the average value.
    }
    z /= pose_buffer.size();
    return z;
}

void downsampleCloud(const pcl::PointCloud<pcl::PointXYZ>::Ptr in_cloud_ptr, \
                     pcl::PointCloud<pcl::PointXYZ>::Ptr out_cloud_ptr, \
                     float in_leaf_size=0.5)
{
    pcl::VoxelGrid<pcl::PointXYZ> sor;
    sor.setInputCloud(in_cloud_ptr);
    sor.setLeafSize((float)in_leaf_size, (float)in_leaf_size, (float)in_leaf_size);
    sor.filter(*out_cloud_ptr);
}

void downsampleCloud_approximate(const pcl::PointCloud<pcl::PointXYZ>::Ptr in_cloud_ptr, \
                     pcl::PointCloud<pcl::PointXYZ>::Ptr out_cloud_ptr, \
                     float in_leaf_size=0.5)
{
    pcl::ApproximateVoxelGrid<pcl::PointXYZ> sor;
    sor.setInputCloud(in_cloud_ptr);
    sor.setLeafSize((float)in_leaf_size, (float)in_leaf_size, (float)in_leaf_size);
    sor.filter(*out_cloud_ptr);
}



int main()
{
    // 轴角初始化 (angle,unit vector of x) why use the unitx ???
    // 系统参考坐标系建立在哪里？？？
    Eigen::AngleAxisf init_rotation(0, Eigen::Vector3f::UnitX());
    // 初始化t矩阵
    Eigen::Translation3f init_translation (0.0, 0.0, 0.0);
    TransformCurrent = (init_translation * init_rotation).matrix ();
    TransformLast = TransformCurrent;

    // 之和前10帧拼接而成的点云地图进行匹配
    // 仿真文件i从1开始,原始文件从0开始 ???!!!
    for(int i=0 ;i<10; i++)
    {
        // strstream类同时可以支持C风格的串流的输入输出操作。需要包含<sstream>头文件
        std::stringstream ss_pcd;
        ss_pcd << i;
        // 把前10帧数据作为全局地图
        std::string s_pcd = pcd_name + ss_pcd.str() + ".pcd";

        pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_ptr (new pcl::PointCloud<pcl::PointXYZ>);

        // Load any PCD file into a templated PointCloud type
        if (pcl::io::loadPCDFile<pcl::PointXYZ> (s_pcd, *cloud_ptr) == -1) //* load the file
        {
          PCL_ERROR ("Couldn't read file test_pcd.pcd \n");
        }

        PointMap += *cloud_ptr; // get the NDT initial map
        PointCurrent = *cloud_ptr;

        point_buffer.push_back(PointCurrent);
        pose_buffer.push_back(TransformCurrent);
    }

    // Init the NDT
    pcl::PointCloud<pcl::PointXYZ>::Ptr PointMap_ptr(new pcl::PointCloud<pcl::PointXYZ>(PointMap));
    pcl::NormalDistributionsTransform<pcl::PointXYZ, pcl::PointXYZ> ndt;
    ndt.setTransformationEpsilon (0.01);//设置迭代结束的条件
    ndt.setStepSize (0.1);//
    ndt.setResolution (0.5);//对map的降采样 室内 设置的小一些
    ndt.setMaximumIterations (30);

    ndt.setInputTarget(PointMap_ptr);//前10帧数据作为地图

    // show the first 10 map and save it.
    pcl::PointCloud<pcl::PointXYZ>::Ptr show_cloud (new pcl::PointCloud<pcl::PointXYZ>);
    show_cloud = PointMap_ptr;
    // why only get the point cloud initial value. ???
    pcl::io::savePCDFileASCII ("new_first10_cloud_init.pcd", *show_cloud);

    for(int i=10 ;i<FILE_NUMBER; i++)
    {
        std::stringstream ss_pcd;
        ss_pcd << i;
        std::string s_pcd = pcd_name + ss_pcd.str() + ".pcd";

        //this part don't use sonar data
//        std::stringstream ss_sonar;
//        ss_sonar << i;
//        std::string s_sonar = sonar_name + ss_sonar.str() + ".bin";


        pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_ptr (new pcl::PointCloud<pcl::PointXYZ>);
        if (pcl::io::loadPCDFile<pcl::PointXYZ> (s_pcd, *cloud_ptr) == -1) //* load the file
        {
          PCL_ERROR ("Couldn't read file test_pcd.pcd \n");
        }

        PointLast.clear();
        PointLast = PointCurrent;
        PointCurrent.clear();
        PointCurrent = *cloud_ptr;

        pcl::PointCloud<pcl::PointXYZ>::Ptr filtered_cloud (new pcl::PointCloud<pcl::PointXYZ>);
        //  0.2 m
        downsampleCloud(cloud_ptr,filtered_cloud, 0.2); //这个参数设置为0.2才能实时，0.5不能实时
        //downsampleCloud_approximate(cloud_ptr,filtered_cloud, 0.2); //这个参数设置为0.2才能实时，0.5不能实时

        ndt.setInputSource (filtered_cloud);
        pcl::PointCloud<pcl::PointXYZ>::Ptr output_cloud (new pcl::PointCloud<pcl::PointXYZ>);
        // Call the registration algorithm which estimates the transformation and returns the transformed source
        // API: align (PointCloudSource &output, const Matrix4& guess);
        // T_i * T_(i-1) = T_(i-1) * T_(i-2)
        //加上帧间预测估计作为配准初始值

        // 送入的估计值有误,旧写法出错原理不明
        ndt.align (*output_cloud, pose_buffer[i-1]*pose_buffer[i-2].inverse()*pose_buffer[i-1]);
        //ndt.align (*output_cloud, pose_buffer[i-1]);
        //ndt.align (*output_cloud);
        TransformCurrent = ndt.getFinalTransformation();     // 每次都相对于之前的全局地图，求RT。

        cout << "i " << i <<endl;
        cout << "pose" << endl << TransformCurrent << endl;
        cout << "score " << ndt.getFitnessScore() <<endl;
        cout << "z ave"  << get_Z_ave() << endl;

    /*
        if(ndt.getFitnessScore() > 0.02)//匹配程度不够时将上一帧数据加进来,再进行匹配
        {
            pcl::PointCloud<pcl::PointXYZ> PointCurrent_map;
            pcl::transformPointCloud (point_buffer[i-1], PointCurrent_map, pose_buffer[i-1]);//将当前帧变换到map坐标系下
            *PointMap_ptr += PointCurrent_map;

            ndt.setInputTarget(PointMap_ptr);
            ndt.align (*output_cloud, TransformCurrent*TransformLast.inverse()*TransformCurrent);//加上帧间预测估计

        }
    */

        // 当z坐标偏差过大时（正常情况下z值稳定不变），对坐标的估计值进行补偿，每一帧在对全局的地图进行匹配。
        if(TransformCurrent(2,3)>0.01 || TransformCurrent(2,3)<-0.01)//z产生了偏差就将z为0的数据放入map中
        {
            pcl::PointCloud<pcl::PointXYZ> PointCurrent_map;
            pose_buffer[i-1](2,3) = 0;//z强行給0            // Apply a rigid transform defined by a 3D offset and a quaternion
            // 变换到map坐标系下有何用？此函数需要继续研究
            pcl::transformPointCloud (point_buffer[i-1], PointCurrent_map, pose_buffer[i-1]);//将上一帧变换到map坐标系下
            // z偏差变大以后，利用前一帧对全局的地图进行更新,因为相邻帧匹配差值小，全局地图加入前一帧信息能够补偿当前帧与全局地图匹配的偏差
            *PointMap_ptr += PointCurrent_map;

            // update the reference map.
            ndt.setInputTarget(PointMap_ptr);//重新设置ndt地图
            // 假设机器人匀速运动，对其估计值进行初始化，通过前一帧的位姿数据来近似当前帧的初始值。
            // 此处近似的数学依据与优化无需证明，因为NDT对预测值的要求并不那么严苛
            // 加上帧间预测估计作为配准初始值
            ndt.align (*output_cloud, pose_buffer[i-1]*pose_buffer[i-2].inverse()*pose_buffer[i-1]);
            //ndt.align (*output_cloud, pose_buffer[i-1]);
            // Get the final transformation matrix estimated by the registration method.
            TransformCurrent = ndt.getFinalTransformation();//获得全局位置

            cout << "fix z" << endl;
            cout << "i " << i <<endl;
            cout << "pose" << endl << TransformCurrent << endl;
            cout << "score " << ndt.getFitnessScore() <<endl;
            cout << "z ave"  << get_Z_ave() << endl;

        }
        point_buffer.push_back(PointCurrent);
        pose_buffer.push_back(TransformCurrent);

    }
    //vector 写入
    ofstream os ("new_pose.bin", ios::binary);
    int size1 = pose_buffer.size();
    os.write((const char*)&size1, 4);
    os.write((const char*)&pose_buffer[0], size1 * sizeof(Eigen::Matrix4f));
    os.close();

    /*
    vector<Eigen::Matrix4f> pose_buffer;
    // 读取
    ifstream is("pose.bin", ios::binary);
    int size1;
    is.read((char*)&size1, 4);
    pose_buffer.resize(size1);
    is.read((char*)&pose_buffer[0], size1 * sizeof(Eigen::Matrix4f));
  */
/////////////////////////////////////////////////////////////////////////////
    // Initializing point cloud visualizer
    boost::shared_ptr<pcl::visualization::PCLVisualizer>
    viewer_final (new pcl::visualization::PCLVisualizer ("3D Viewer"));
    viewer_final->setBackgroundColor (0, 0, 0);

    // Coloring and visualizing transformed input cloud (green).
    pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZ>
    output_color (show_cloud, 0, 255, 0);
    viewer_final->addPointCloud<pcl::PointXYZ> (show_cloud, output_color, "output cloud");
    viewer_final->setPointCloudRenderingProperties (pcl::visualization::PCL_VISUALIZER_POINT_SIZE,
                                                    1, "output cloud");

    // Starting visualizer
    viewer_final->addCoordinateSystem (1.0, "global");
    viewer_final->initCameraParameters ();

    // get the point cloud final value.
    pcl::io::savePCDFileASCII ("new_first10_cloud_final.pcd", *show_cloud);
    // Wait until visualizer window is closed.
    while (!viewer_final->wasStopped ())
    {
      viewer_final->spinOnce (100);
      boost::this_thread::sleep (boost::posix_time::microseconds (100000));
    }

    return 1;
}
