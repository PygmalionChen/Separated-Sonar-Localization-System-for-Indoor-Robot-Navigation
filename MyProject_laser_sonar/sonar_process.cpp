#include <sstream>
#include <iostream>
#include <fstream>
#include <iterator>
#include <string>
#include <vector>

#include <Eigen/Core>
#include <Eigen/StdVector>
#include <Eigen/Geometry>

#include <ceres/ceres.h>

using namespace std;
//根据pose和超声波数据，构建优化问题 进行解析

//从接收单元发送过来的数据
struct ReceivedUnitBaseDate
{
    char addressH;
    char addressL;
    char sourcePort;
    char targetPort;
    int tof[16];//传播时间us
};

// pose通过读取NDT结果获得
struct measure_date
{
    Eigen::Matrix4f pose;
    float length;
};

vector<measure_date> receive_sonar[16];//16个接受端，每个接受端会有很多的测量数据

vector<Eigen::Matrix4f> pose_buffer;//激光雷达的全局位置


std::string sonar_name("/home/pygmalionchen/SLAM_Prj/SLAM_Data/sonar_laser_date/20170615_02/sonar");


struct POSE_FITTING_COST
{
    //组合类构造函数
    POSE_FITTING_COST(Eigen::Matrix4f pose, float length): _pose(pose), _length(length){}
    //残差计算
    template <typename T>   
    bool operator() (
            const T* const xyz, //模型参数 //xyzvb
            T* residual) const //残差
    {
       // length - sqrt( (pose(0,3)-x)^2 +  (pose(1,3)-y)^2 + (pose(2,3)-z)^2)
        residual[0] = _length - T(ceres::sqrt(ceres::pow(T(_pose(0,3))-xyz[0], 2) +
                                               ceres::pow(T(_pose(1,3))-xyz[1], 2) +
                                               ceres::pow(T(_pose(2,3))-xyz[2], 2) ) );
        //Part1:add the optimization variable
//        residual[0] = _t*xyzvb[3]+xyzvb[4] -  T(ceres::sqrt(ceres::pow(T(_pose(0,3))-xyz[0], 2) +
//                                                ceres::pow(T(_pose(1,3))-xyz[1], 2) +
//                                                ceres::pow(T(_pose(2,3))-xyz[2], 2) ) );
        return true;
    }

    const Eigen::Matrix4f _pose;
    const double _length;
};




int main()
{
    
    ifstream is("pose.bin", ios::binary);
    int size1;
    is.read((char*)&size1, 4);
    // resize()
    pose_buffer.resize(size1);
    is.read((char*)&pose_buffer[0], size1 * sizeof(Eigen::Matrix4f));
    is.close();

    for(int i=0; i<pose_buffer.size(); i++)
    {
        vector<ReceivedUnitBaseDate> sonarLists;

        std::stringstream ss_sonar;
        ss_sonar << i;
        std::string s_sonar = sonar_name + ss_sonar.str() + ".bin";
        ifstream is(s_sonar, ios::binary);
        // size1
        int size1;
        is.read((char*)&size1, 4); 
        // resize()
        sonarLists.resize(size1);   
        is.read((char*)&sonarLists[0], size1 * sizeof(ReceivedUnitBaseDate));
        is.close();
        ofstream out("data_all.txt",ofstream::app);
        if (out.is_open())
        for(int j=0; j<sonarLists.size(); j++)
        {
            //
            for(int k=0;k<16;k++)
            {
                //if (sonarLists[j].tof[k] != 0 )
                    out<<k<<" "<<sonarLists[j].tof[k]<<endl;

//                out<<"the sonarLists:"<<j<<"tof:"<<k<<"data:"<<sonarLists[j].tof[k]<<endl;
                cout<<"the sonarLists:"<<j<<" tof:"<<k<<" data: "<<sonarLists[j].tof[k]<<endl;

                // 仅仅简单排除异常值
                if(sonarLists[j].tof[k]>5000 && sonarLists[j].tof[k]<7000)
                {
                    measure_date temp;
                    temp.pose = pose_buffer[i];
                    //Part2: draw data
                    //Part3: data filter
                    // 0.00034 m/us
                    temp.length = sonarLists[j].tof[k]*0.00034+0.03;//此部分可以考虑用ceres直接优化处理
                    receive_sonar[k].push_back(temp);
                }
            }
        }
        out.close();
    }
    //统计好所有的receive_sonar数据，并且是按时间先后顺序，可以后续做滤波处理。

    //之后进行ceres优化


//////////////////////////////////////////////////////////////////////////////////
/////
//    ceres::Problem problem;
//    for(int i=0; i<receive_sonar[15].size(); i++)
//    {
//        problem.AddResidualBlock(
//                    new ceres::AutoDiffCostFunction<POSE_FITTING_COST, 1, 3>(
//                        new POSE_FITTING_COST(receive_sonar[15][i].pose, (double)receive_sonar[15][i].length)
//                        ),
//                    nullptr,
//                    xyz//问题在这里 函数中是double
//                    );
//    }


//    ceres::Solver::Options options;
//    options.linear_solver_type = ceres::DENSE_QR;
//    options.minimizer_progress_to_stdout = false;

//    ceres::Solver::Summary summary;
//    ceres::Solve(options, &problem, &summary);//开始优化
//    cout << summary.BriefReport() <<endl;
//    for(auto x : xyz) cout << x << " ";
//    cout <<endl;

//////////////////////////////////////////////////////////////////////

    for(int ii=0; ii<16; ii++)
    {
        double xyz[3] = {0,0,2};    //可以考虑优化初值设置方案
        ceres::Problem problem;
        for(int i=0; i<receive_sonar[ii].size(); i++)
        {
            problem.AddResidualBlock(
                        new ceres::AutoDiffCostFunction<POSE_FITTING_COST, 1, 3>(
                            //传入第ii个sonar的第i组数据 // sonar.pose存储激光获取的全局位姿
                            new POSE_FITTING_COST(receive_sonar[ii][i].pose, (double)receive_sonar[ii][i].length)
                            ),
                        nullptr,
                        xyz//问题在这里 函数中是double
                        );
        }
        ceres::Solver::Options options;
        options.linear_solver_type = ceres::DENSE_QR;
        options.minimizer_progress_to_stdout = false;

        ceres::Solver::Summary summary;
        ceres::Solve(options, &problem, &summary);//开始优化
        cout << "id " << ii << endl;
        cout << summary.BriefReport() <<endl;
        for(auto x : xyz) cout << x << " ";

        cout <<endl;
    }
    return 0;
}
