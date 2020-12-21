#include <sstream>
#include <iostream>
#include <fstream>
#include <iterator>
#include <string>
#include <vector>
#include <string>

#include <Eigen/Core>
#include <Eigen/StdVector>
#include <Eigen/Geometry>

#include <ceres/ceres.h>

using namespace std;

// 25_pose_opt has 2313 files.
// 25_pose_opt_z has 9334 files.
// 25_opt_sn has 2313 files.

//#define BATCH_NUM  2313
#define BATCH_NUM  9334

//根据pose和超声波数据，构建优化问题 进行解析

//超声波数据块
struct ReceivedUnitBaseDate
{
//    float distance[30];
    float distance[25];
};

//位姿数据块
struct PoseData
{
    float x;
    float y;
    float z;
};

vector<PoseData> poseLists;

// pose通过读取NDT结果获得
// Lidar Sonar information structure.
struct measure_date
{
    PoseData pose;
    float length;
};

//vector<measure_date> receive_sonar[30];  //30个接受端，每个接受端会有很多的测量数据
vector<measure_date> receive_sonar[25];  //25个接受端，每个接受端会有很多的测量数据

//　超声波batch路径
// std::string sonar_name("/home/pygmalionchen/PycharmProjects/SLAM_script/sim_data_split/25_opt_sn/snk_opt_01_0_1_bin/SonarBatch_bin_");

// std::string sonar_name("/home/pygmalionchen/PycharmProjects/SLAM_script/sim_data_split/pose_opt/pose_opt_sonar_bin/SonarBatch_bin_");
//std::string sonar_name("/home/pygmalionchen/PycharmProjects/SLAM_script/sim_data_split/25_opt/25_opt_sonar_bin/SonarBatch_bin_");


//// sonar z opt without noise
//std::string sonar_name("/home/pygmalionchen/PycharmProjects/SLAM_script/sim_data_split/25_opt_z/25_opt_z_sonar_bin/SonarBatch_bin_");

//// sonar z opt with 4 batch
//  std::string sonar_name("/home/pygmalionchen/PycharmProjects/SLAM_script/sim_data_split/25_opt_z/snk_opt124_35373941_01_0_1_bin/SonarBatch_bin_");
//  std::string sonar_name("/home/pygmalionchen/PycharmProjects/SLAM_script/sim_data_split/25_opt_z/snk_opt124_35373941_001_0_01_bin/SonarBatch_bin_");

//    std::string sonar_name("/home/pygmalionchen/PycharmProjects/SLAM_script/sim_data_split/25_opt_z/snk_opt124_35373941_02_0_1_bin/SonarBatch_bin_");
    std::string sonar_name("/home/pygmalionchen/PycharmProjects/SLAM_script/sim_data_split/25_opt_z/snk_opt124_35373941_03_0_1_bin/SonarBatch_bin_");

struct POSE_FITTING_COST
{
    //组合类构造函数
    POSE_FITTING_COST(PoseData pose, float length): _pose(pose), _length(length){}
    //残差计算25_opt_z
    template <typename T>
    bool operator() (
            const T* const xyz, //模型参数 //xyzvb
            T* residual) const //残差
    {
       // length - sqrt( (pose(0,3)-x)^2 +  (pose(1,3)-y)^2 + (pose(2,3)-z)^2)
        residual[0] = _length - T(ceres::sqrt(ceres::pow(T(_pose.x)-xyz[0], 2) +
                                               ceres::pow(T(_pose.y)-xyz[1], 2) +
                                               ceres::pow(T(_pose.z)-xyz[2], 2) ) );
    // Part1:add the optimization variable
//        residual[0] = _t*xyzvb[3]+xyzvb[4] -  T(ceres::sqrt(ceres::pow(T(_pose(0,3))-xyz[0], 2) +
//                                                ceres::pow(T(_pose(1,3))-xyz[1], 2) +
//                                                ceres::pow(T(_pose(2,3))-xyz[2], 2) ) );
        return true;
    }

    const PoseData _pose;
    const double _length;
};


int main()
{
    // Pose data get
    string Data;
//    int line = 0;
    int line_count = 0;
//    ifstream posefile("/home/pygmalionchen/PycharmProjects/SLAM_script/sim_data_split/25_opt_sn/25_opt_sn_data_org/pose_opt1129.txt");

// //4 batch union to z opt ,z value is 2.1
//    ifstream posefile("/home/pygmalionchen/PycharmProjects/SLAM_script/sim_data_split/25_opt_z/25_opt_z_data_split/pose_opt124_35373941.txt");

// //4 batch union to z opt,pose noise add
//    ifstream posefile("/home/pygmalionchen/PycharmProjects/SLAM_script/sim_data_split/25_opt_z/25_opt_z_pnk_split/pose_opt124_35373941_pnk_01_0_1.txt");

// //4 batch union to z opt,pose noise add with bias
//    ifstream posefile("/home/pygmalionchen/PycharmProjects/SLAM_script/sim_data_split/25_opt_z/25_opt_z_pnk_split/pose_opt124_35373941_pnk_02_0_1_004.txt");

//  //4 batch union to z opt,pose noise add with binomial distribution
//    ifstream posefile("/home/pygmalionchen/PycharmProjects/SLAM_script/sim_data_split/25_opt_z/25_opt_z_pnk_split/pose_opt124_35373941_pnk_01_0_1_bi.txt");
//    ifstream posefile("/home/pygmalionchen/PycharmProjects/SLAM_script/sim_data_split/25_opt_z/25_opt_z_pnk_split/pose_opt124_35373941_pnk_001_0_01_bi.txt");
    ifstream posefile("/home/pygmalionchen/PycharmProjects/SLAM_script/sim_data_split/25_opt_z/25_opt_z_pnk_split/pose_opt124_35373941_pnk_02_0_1_bi.txt");

//// single noise add
//    ifstream posefile("/home/pygmalionchen/PycharmProjects/SLAM_script/sim_data_split/25_opt/25_opt_data_split/noise_add/pose_opt_pnk_02_0_1_004.txt");
    // failed in 0.35z
//    ifstream posefile("/home/pygmalionchen/PycharmProjects/SLAM_script/sim_data_split/25_opt/25_opt_data_split/noise_add/pose_opt_pnk_02_0_1.txt");
    // failed in 0.35z
//    ifstream posefile("/home/pygmalionchen/PycharmProjects/SLAM_script/sim_data_split/25_opt/25_opt_data_split/noise_add/pose_opt_pnk_01_0_1.txt");



    //// the wrong noise
//    ifstream posefile("/home/pygmalionchen/PycharmProjects/SLAM_script/sim_data_split/25_opt_z/25_opt_z_data_split/noise_add_z/pose_opt124_35373941_0_001.txt");
//    ifstream posefile("/home/pygmalionchen/PycharmProjects/SLAM_script/sim_data_split/25_opt_z/25_opt_z_data_split/noise_add_z/pose_opt124_35373941_0_005.txt");
//    ifstream posefile("/home/pygmalionchen/PycharmProjects/SLAM_script/sim_data_split/25_opt_z/25_opt_z_data_split/noise_add_z/pose_opt124_35373941_0_01.txt");

//    ifstream posefile("/home/pygmalionchen/PycharmProjects/SLAM_script/sim_data_split/25_opt/25_opt_data_split/noise_add/Npose_opt1129_0_1.txt");
//    ifstream posefile("/home/pygmalionchen/PycharmProjects/SLAM_script/sim_data_split/25_opt/25_opt_data_split/noise_add/Npose_opt1129_0_01.txt");
//    ifstream posefile("/home/pygmalionchen/PycharmProjects/SLAM_script/sim_data_split/25_opt/25_opt_data_split/noise_add/Npose_opt1129_0_005.txt");
//    ifstream posefile("/home/pygmalionchen/PycharmProjects/SLAM_script/sim_data_split/25_opt/25_opt_data_split/noise_add/Npose_opt1129_0_001.txt");
//    ifstream posefile("/home/pygmalionchen/PycharmProjects/SLAM_script/sim_data_split/25_opt/25_opt_data_split/pose_opt1129.txt");
//    ifstream posefile("/home/pygmalionchen/PycharmProjects/SLAM_script/sim_data_split/pose_opt/pose_opt_Tdata_split/pose_opt1122.txt");
//    ifstream posefile("/home/pygmalionchen/PycharmProjects/SLAM_script/sim_data_split/pose_opt/pose_opt_Tdata_split/add_noise/hhpose_opt1122_n.txt");
    if (posefile.is_open())
    {
        cout<<"open posefile successful."<<endl;
        PoseData temp_pose;
        while (getline(posefile,Data))
        {
//            cout <<"line data:"<<data<<endl;
//            line_count++;
            line_count = line_count +1;
            float data = atof(Data.c_str());
//            cout<<"  data:  "<<data<<endl;
            if (line_count == 1)
            {
                temp_pose.x = data;
            }
            else if (line_count == 2)
            {
                temp_pose.y = data;
            }
            else if (line_count == 3)
            {
                temp_pose.z = data;
                line_count = 0;
                cout<<"  x:  "<<temp_pose.x <<"  y:  "<<temp_pose.y<<"  z: "<<temp_pose.z<<endl;
                poseLists.push_back(temp_pose);
            }


        }
//        for (int i = 0; i<poseLists.size() ; i++)
//        {
//            cout<<"  x:  "<<poseLists[i].x <<"  y:  "<<poseLists[i].y<<"  z: "<<poseLists[i].z<<endl;
//        }
    }

    // i is the frame of this system.
    // BATCH_NUM determine the number of frames.
    for(int i=0 ; i < BATCH_NUM ; i++ )                //i++
    {
        // Sonar data get
        vector<ReceivedUnitBaseDate> sonarLists;

        stringstream ss_sonar;
        ss_sonar << i;
        string s_sonar = sonar_name + ss_sonar.str() + ".bin";
        ifstream sonarfile(s_sonar, ios::binary);
        // size1 为超声波sonarlist的数量,针对不同的sonarlist存储数量设置
        int size1;
        sonarfile.read((char*)&size1, 4);
        sonarLists.resize(size1);   // 为sonarlists分配内存
        sonarfile.read((char*)&sonarLists[0], size1 * sizeof(ReceivedUnitBaseDate));
        sonarfile.close();

        // ofstream out("Sim_data.txt",ofstream::app);
        // if (out.is_open())
        // j 代表每帧超声波数据个数(对于仿真文件,sonarLists.size值恒定)
        for(int j=0; j<sonarLists.size(); j++)
        {
            for(int k=0;k<25;k++) //30
            {
               // output the data_all.txt.
               cout<<"sonarLists :"<<j<<" distance: "<<k<<"sonar_data: "<<sonarLists[j].distance[k]<<endl;

                // 仅仅简单排除异常值
                if(sonarLists[j].distance[k]>1.5 && sonarLists[j].distance[k]<3.5)
                {
                    //measure_data 一个pose对应一个length
                    //存入sonarLists以后,后续优化时多个length对应相同pose
                    measure_date temp;
                    // 存pose入 receive_sonar
                    temp.pose.x = poseLists[i].x;
                    temp.pose.y = poseLists[i].y;
                    temp.pose.z = poseLists[i].z;

                    //cout<<"temp.pose:"<<i<<"\n"<<pose_buffer[i]<<endl;
                    temp.length = sonarLists[j].distance[k];
                    // 此处同步把sonar和lidar对应帧数据存入
                    receive_sonar[k].push_back(temp);
                }
            }
        }
        sonarLists.clear();
    }
    // 25 sonar station

    float z = 2.1;    //2.1 is good // 0.36 0.39 is best    //0.35 is the basic hight

//    float z0 = 2.40;
//    float z_error = 0;
//    for (float z=0.30; z<0.5;z = z + 0.01)
//    {
//        cout<<"batch "<<z <<"########################################"<<endl;

    for(int ii=0; ii<25; ii++) //30
    {
        //{0,0.1,>0.38}
        double xyz[3] = {0,0.1,z};    //可以考虑优化初值设置方案 //{0,0.1,0.35};
        ceres::Problem problem;
        for(int i=0; i<receive_sonar[ii].size(); i++)
        {
            // AutoDiffCostFunction   // NumericDiffCostFunction  ,ceres::CENTRAL
            problem.AddResidualBlock(
                        new ceres::AutoDiffCostFunction<POSE_FITTING_COST, 1, 3>(
                            //传入第ii个sonar的第i组数据 // sonar.pose存储激光获取的全局位姿
                            new POSE_FITTING_COST(receive_sonar[ii][i].pose, (double)receive_sonar[ii][i].length)
                            ),
                        nullptr,
                        xyz     //问题在这里 函数中是double
                        );
        }

        ceres::Solver::Options options;
        options.linear_solver_type = ceres::DENSE_QR;
        options.minimizer_progress_to_stdout = false;

        ceres::Solver::Summary summary;
        ceres::Solve(options, &problem, &summary);  //开始优化
//        cout << "ID " << ii << endl;
//        cout << summary.BriefReport() <<endl;
        for(auto x : xyz) cout << x << " ";

//        //// error test
//        z_error = z0 - xyz[2];
//        if(abs(z_error)>0.5)
//        {
//            cout <<z_error<<" ";
//        }

        cout <<endl;
//    }
//    cout <<endl;
    }

    return 0;
}
