#include <fstream>
#include <iostream>
#include <eigen3/Eigen/Geometry>

using namespace std;
using namespace Eigen;



static Eigen::Vector3d R2ypr(const Eigen::Matrix3d &R)
{
    Eigen::Vector3d n = R.col(0);
    Eigen::Vector3d o = R.col(1);
    Eigen::Vector3d a = R.col(2);

    Eigen::Vector3d ypr(3);
    double y = atan2(n(1), n(0));
    double p = atan2(-n(2), n(0) * cos(y) + n(1) * sin(y));
    double r = atan2(a(0) * sin(y) - a(1) * cos(y), -o(0) * sin(y) + o(1) * cos(y));
    ypr(0) = y;
    ypr(1) = p;
    ypr(2) = r;

    return ypr / M_PI * 180.0;
}

Eigen::Matrix3d ypr2R(double R_x, double R_y, double R_z)
{
    double r = R_x / 180.0 * M_PI;
    Eigen::Matrix<double, 3, 3> Rx;
    Rx << 1., 0., 0.,
        0., cos(r), -sin(r),
        0., sin(r), cos(r);

    double p = R_y / 180.0 * M_PI;
    Eigen::Matrix<double, 3, 3> Ry;
    Ry << cos(p), 0., sin(p),
        0., 1., 0.,
        -sin(p), 0., cos(p);

    double y = R_z / 180.0 * M_PI;
    Eigen::Matrix<double, 3, 3> Rz;
    Rz << cos(y), -sin(y), 0,
        sin(y), cos(y), 0,
        0, 0, 1;

    return Rz * Ry * Rx;
}


int main()
{
    ////////read groundtruth file
    std::ifstream infile("thefile.txt");
    std::string line;

    FILE * pFile;
    std::vector<Vector3d> vector_input_T;
    std::vector<Quaterniond> vector_input_Q;
    std::vector<double> vector_time_stamp;
    const string file_path = "input.txt";
    printf("lode pose graph from: %s \n", file_path.c_str());
    printf("pose graph loading...\n");
    pFile = fopen (file_path.c_str(),"r");
    if (pFile == NULL)
    {
        printf("lode preinputus pose graph error: wrong previous pose graph path or no previous pose graph \n the system will start with new pose graph \n");
        return 0;
    }
    double time_stamp;
    double input_Tx, input_Ty, input_Tz;
    double input_Qw, input_Qx, input_Qy, input_Qz;
    bool input_1st_flag = true;
    while (fscanf(pFile,"%lf %lf %lf %lf %lf %lf %lf %lf", &time_stamp, 
                                    &input_Tx, &input_Ty, &input_Tz, 
                                    &input_Qx, &input_Qy, &input_Qz, &input_Qw
                                    ) != EOF) 
    {
        
        Vector3d input_T(input_Tx, input_Ty, input_Tz);
        Quaterniond input_Q;
        input_Q.x() = input_Qx;
        input_Q.y() = input_Qy;
        input_Q.z() = input_Qz;
        input_Q.w() = input_Qw;
        Matrix3d input_R;
        input_R = input_Q.toRotationMatrix();

        Matrix3d R_mocap_segment = input_R;
        Vector3d T_mocap_segment = input_T;
   


        ////////////////////transform///////////////

        Eigen::Vector3d T_mocap_segment0 = Eigen::Vector3d(0.452711,-1.261699,0.283010);   
        Quaterniond Q_mocap_segment0;
        Q_mocap_segment0.x() =  0.707625;
        Q_mocap_segment0.y() = 0.021402;
        Q_mocap_segment0.z() = 0.049063;
        Q_mocap_segment0.w() = 0.704558;//！！！注意不同文件中，wxyz和xyzw的顺序的不同。
        Matrix3d R_mocap_segment0 = Q_mocap_segment0.toRotationMatrix();

        Eigen::Matrix3d R_vins_mocap;
        Eigen::Vector3d T_vins_mocap;             
        if(input_1st_flag == true)
        {
            cout<<"firstinput:"<<input_Tx<<"  "<<input_Ty<<"  "<<input_Tz<<"  "<<input_Qx<<"  "<<input_Qy<<"  "<<input_Qz<<"  "<<input_Qw<<"  "<<endl;
            cout<<"transform :"<<T_mocap_segment0[0]<<"  "<<T_mocap_segment0[1]<<"  "<<T_mocap_segment0[2]<<"  "<<Q_mocap_segment0.x()<<"  "<<Q_mocap_segment0.y()<<"  "<<Q_mocap_segment0.z()<<"  "<<Q_mocap_segment0.w()<<"  "<<endl;
            cout<<"make sure that the manual input is right?";

        
            Eigen::Matrix3d R_vins_segment0 = ypr2R(90.0, 0.0, 0.0);
            Eigen::Matrix3d R_segment0_vins = R_vins_segment0.inverse();
            Eigen::Matrix3d R_mocap_vins = R_mocap_segment0 * R_segment0_vins;        
            R_mocap_vins =  ypr2R(0, 0, R2ypr(R_mocap_vins).x());//x:yaw
            Eigen::Vector3d T_mocap_vins = T_mocap_segment0;  

            R_vins_mocap =  R_mocap_vins.inverse();
            T_vins_mocap =  (-1) * R_vins_mocap * T_mocap_vins;     
        
            input_1st_flag = false;        
        }

        Matrix3d R_vins_segment = R_vins_mocap * R_mocap_segment;
        Vector3d T_vins_segment = (R_vins_mocap) * T_mocap_segment + T_vins_mocap;

        Quaterniond Q_vins_segment= Quaterniond(R_vins_segment);

        vector_time_stamp.push_back(time_stamp);   
        vector_input_Q.push_back(Q_vins_segment); 
        vector_input_T.push_back(T_vins_segment);  
    }
    fclose (pFile);
    FILE *pFile_save;
    const string file_path_w = "output.txt";
    pFile_save = fopen (file_path_w.c_str(),"w");
    int n_num = int(vector_input_Q.size());
    for (int i = 0; i < n_num; i++)
    {
        fprintf (pFile_save, "%f %f %f %f %f %f %f %f\n",vector_time_stamp[i], 
                                    vector_input_T[i].x(), vector_input_T[i].y(), vector_input_T[i].z(), 
                                    vector_input_Q[i].x(), vector_input_Q[i].y(), vector_input_Q[i].z(),vector_input_Q[i].w());
    }
    fclose(pFile_save);

    FILE *pFile_save_z0;
    const string file_path_w_z0 = "output_z0.txt";
    pFile_save_z0 = fopen (file_path_w_z0.c_str(),"w");
    for (int i = 0; i < n_num; i++)
    {
        fprintf (pFile_save_z0, "%f %f %f %f %f %f %f %f\n",vector_time_stamp[i], 
                                    vector_input_T[i].x(), vector_input_T[i].y(), 0.0, 
                                    vector_input_Q[i].x(), vector_input_Q[i].y(), vector_input_Q[i].z(),vector_input_Q[i].w());
    }
    fclose(pFile_save_z0);
}