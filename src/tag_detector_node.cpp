#include <iostream>
#include <ros/ros.h>
#include <ros/console.h>
#include <sensor_msgs/Image.h>
#include <sensor_msgs/image_encodings.h>
#include <cv_bridge/cv_bridge.h>
#include <nav_msgs/Odometry.h>
#include <aruco/aruco.h>
#include <aruco/cvdrawingutils.h>
#include <opencv2/opencv.hpp>
#include <Eigen/Eigen>
#include <Eigen/SVD>
#include <opencv2/core/eigen.hpp>

#include "ReprojectionError.h"
#include "tic_toc.h"
using namespace cv;
using namespace aruco;
using namespace Eigen;

//global varialbles for aruco detector
aruco::CameraParameters CamParam;
MarkerDetector MDetector;
vector<Marker> Markers;
float MarkerSize = 0.20 / 1.5 * 1.524;
float MarkerWithMargin = MarkerSize * 1.2;
BoardConfiguration TheBoardConfig;
BoardDetector TheBoardDetector;
Board TheBoardDetected;
ros::Publisher pub_odom;
ros::Publisher pub_odom_ref;
Quaterniond Q;


cv::Mat K, D;



void ceresMethod(const vector<cv::Point3f> &pts_3, const vector<cv::Point2f> &un_pts_2, Quaterniond &Q, Vector3d &T)
{
    ROS_INFO("ceres_method!");
    // x y z w
    
    double camera_R[4];
    camera_R[0] = Q.x();
    camera_R[1] = Q.y();
    camera_R[2] = Q.z();
    camera_R[3] = Q.w();

    double* camera_T;
    camera_T = T.data();
    /*
    std::cout << "ceres initial quaterniond" << std::endl;
    for(int i = 0; i < 4; i++)
        std::cout << camera_R[i] << "  ";
    std::cout << std::endl;
    std::cout << "ceres initial translation" << std::endl;
    for(int i = 0; i < 3; i++)
        std::cout << T.transpose() << "  " << std::endl;
    std::cout << std::endl;
    */
    ceres::Problem problem;

    //ceres::LocalParameterization* local_parameterization =
    //    new ceres::AutoDiffLocalParameterization<QuaternionPlus, 4, 3>;
    ceres::LocalParameterization* local_parameterization = new QuaternionParameterization();
    problem.AddParameterBlock(camera_R, 4, local_parameterization);
    problem.AddParameterBlock(camera_T, 3);

    for (int i = 0; i < pts_3.size(); i++)
    {
        
        //ceres::CostFunction* cost_function =
        //    new ceres::AutoDiffCostFunction<ReprojectionError, 2, 4, 3>
        //        (new ReprojectionError(un_pts_2[i].x, un_pts_2[i].y, pts_3[i].x, pts_3[i].y, pts_3[i].z));
        
        
        ceres::CostFunction* cost_function  = 
            new ReprojectionCostFunction(un_pts_2[i].x, un_pts_2[i].y, pts_3[i].x, pts_3[i].y, pts_3[i].z);
        

        problem.AddResidualBlock(cost_function, NULL, camera_R, camera_T);
        /*
        
        double **para = new double *[2];
        double *res = new double[2];
        double **jacobian = new double *[2];
        jacobian[0] = new double [8];
        jacobian[1] = new double [6];
        para[0] = camera_R;
        para[1] = camera_T;
        cost_function->Evaluate(para, res, jacobian);
        std::cout << "auto ceres x " << res[0] << std::endl;
        std::cout << "auto ceres y " << res[1] << std::endl;
        std::cout << "jacobian1 " << jacobian[0][0] <<"  "<< jacobian[0][1] <<"  "<< jacobian[0][2] <<"  "<< jacobian[0][3] <<"  "<< endl;
        std::cout << "jacobian2 " << jacobian[0][4] <<"  "<< jacobian[0][5] <<"  "<< jacobian[0][6] <<"  "<< jacobian[0][7] <<"  "<< endl;
        */
        /*
        {
            Vector3d p_w = Vector3d(pts_3[i].x, pts_3[i].y, pts_3[i].z);
            Vector3d p_cam = R * p_w + T;
            Matrix<double, 2, 3> H;
            H << 1.0 / p_cam(2), 0, -p_cam(0) / (p_cam(2) * p_cam(2)),
                0, 1.0 / p_cam(2), -p_cam(1) / (p_cam(2) * p_cam(2));

            Matrix<double, 3, 6> F;
            F.block<3, 3>(0, 0) = -R * skewSymmetric(p_w);
            F.block<3, 3>(0, 3) = Matrix3d::Identity();

            Matrix<double, 2, 6> tmp_A = H * F;
            std::cout << "tmp_A" << std::endl << tmp_A << endl;
        }
        */
        /*
        ceres::CostFunction* cost_function2 =
            new ceres::AutoDiffCostFunction<ReprojectionError, 2, 4, 3>
                (new ReprojectionError(un_pts_2[i].x, un_pts_2[i].y, pts_3[i].x, pts_3[i].y, pts_3[i].z));
        double *res2 = new double[2];
        cost_function->Evaluate(para, res2, NULL);
        std::cout << "auto ceres x " << res2[0] << std::endl;
        std::cout << "auto ceres y " << res2[1] << std::endl;
        */
    }
    //problem.SetParameterBlockConstant(camera_R);

    ceres::Solver::Options options;
    options.linear_solver_type = ceres::DENSE_QR;
    options.trust_region_strategy_type = ceres::DOGLEG;
    options.minimizer_progress_to_stdout = true;
    ceres::Solver::Summary summary;
    ceres::Solve(options, &problem, &summary);

    //std::cout << summary.BriefReport() << "\n";
/*
    std::cout << "ceres quaterniond" << std::endl;
    for(int i = 0; i < 4; i++)
        std::cout << camera_R[i] << "  " << std::endl;
    std::cout << "ceres translation" << std::endl;
    for(int i = 0; i < 3; i++)
        std::cout << camera_T[i] << "  " << std::endl; 

    std::cout << T.transpose() << "  " << std::endl;
*/
    Q.x() = camera_R[0];
    Q.y() = camera_R[1];
    Q.z() = camera_R[2];
    Q.w() = camera_R[3];

/*
    for (int i = 0; i < pts_3.size(); i++)
    {

        ceres::CostFunction* cost_function =
            new ceres::AutoDiffCostFunction<ReprojectionError, 2, 4, 3>
                (new ReprojectionError(un_pts_2[i].x, un_pts_2[i].y, pts_3[i].x, pts_3[i].y, pts_3[i].z));
        double **para = new double *[2];
        double *res = new double[2];
        para[0] = camera_R;
        para[1] = camera_T;
        cost_function->Evaluate(para, res, NULL);
        //std::cout << "after ceres residual x " << res[0] << std::endl;
        //std::cout << "after ceres residual y " << res[1] << std::endl;
    }
*/
}

void ceresMethodAD(const vector<cv::Point3f> &pts_3, const vector<cv::Point2f> &un_pts_2, Quaterniond &Q, Vector3d &T)
{
    ROS_INFO("ceres_method!");
    double camera_R[4], camera_T[3];
    camera_R[0] = Q.w();
    camera_R[1] = Q.x();
    camera_R[2] = Q.y();
    camera_R[3] = Q.z();

    camera_T[0] = T(0);
    camera_T[1] = T(1);
    camera_T[2] = T(2);
    std::cout << "ceres initial quaterniond" << std::endl;
    for(int i = 0; i < 4; i++)
        std::cout << camera_R[i] << "  ";
    std::cout << std::endl;
    std::cout << "ceres initial translation" << std::endl;
    for(int i = 0; i < 3; i++)
        std::cout << camera_T[i] << "  " << std::endl;
    std::cout << std::endl;
    ceres::Problem problem;

    //ceres::LocalParameterization* local_parameterization =
    //    new ceres::AutoDiffLocalParameterization<QuaternionPlus, 4, 3>;
    ceres::LocalParameterization* local_parameterization = new ceres::QuaternionParameterization();
    problem.AddParameterBlock(camera_R, 4, local_parameterization);
    problem.AddParameterBlock(camera_T, 3);

    for (int i = 0; i < pts_3.size(); i++)
    {
        
        ceres::CostFunction* cost_function =
            new ceres::AutoDiffCostFunction<ReprojectionError, 2, 4, 3>
                (new ReprojectionError(un_pts_2[i].x, un_pts_2[i].y, pts_3[i].x, pts_3[i].y, pts_3[i].z));
        problem.AddResidualBlock(cost_function, NULL, camera_R, camera_T);
    }

    ceres::Solver::Options options;
    options.linear_solver_type = ceres::DENSE_QR;
    options.trust_region_strategy_type = ceres::DOGLEG;
    options.minimizer_progress_to_stdout = true;
    ceres::Solver::Summary summary;
    ceres::Solve(options, &problem, &summary);

    std::cout << summary.BriefReport() << "\n";

    std::cout << "ceres quaterniond" << std::endl;
    for(int i = 0; i < 4; i++)
        std::cout << camera_R[i] << "  " << std::endl;
    std::cout << "ceres translation" << std::endl;
    for(int i = 0; i < 3; i++)
        std::cout << camera_T[i] << "  " << std::endl;
}
void newtonMethod(const vector<cv::Point3f> &pts_3, const vector<cv::Point2f> &un_pts_2, Matrix3d &R, Vector3d &T)
{
    for(int k = 0; k < 5; k ++)
    {
        //ROS_INFO("iteration %d", k);
        Matrix<double, 6, 6> A;
        VectorXd b(6);
        A.setZero();
        b.setZero();
        double error = 0;
        for(unsigned int i = 0; i < pts_3.size(); i++)
        {
            Vector3d p_w = Vector3d(pts_3[i].x, pts_3[i].y, pts_3[i].z);
            Vector3d p_cam = R * p_w + T;

            Vector2d tmp_b = Vector2d(p_cam(0) / p_cam(2) - un_pts_2[i].x,
                                        p_cam(1) / p_cam(2) - un_pts_2[i].y);
            error += tmp_b.norm();
            Matrix<double, 2, 3> H;
            H << 1.0 / p_cam(2), 0, -p_cam(0) / (p_cam(2) * p_cam(2)),
                0, 1.0 / p_cam(2), -p_cam(1) / (p_cam(2) * p_cam(2));

            Matrix<double, 3, 6> F;
            F.block<3, 3>(0, 0) = -R * skewSymmetric(p_w);
            F.block<3, 3>(0, 3) = Matrix3d::Identity();

            Matrix<double, 2, 6> tmp_A = H * F;
            A += tmp_A.transpose() * tmp_A;
            b += tmp_A.transpose() * tmp_b;
        }
        VectorXd dx(6);
        dx = A.llt().solve(-b);
        //ROS_INFO("error  %f", error);
        //cout << "dx  " << dx.transpose() << endl;
        //ROS_INFO("update state!");
        Quaterniond q(R);
        Quaterniond dq(
            1,
            dx(0) / 2,
            dx(1) / 2,
            dx(2) / 2);
        dq.w() = 1 - dq.vec().transpose() * dq.vec();
        R = (q * dq).normalized().toRotationMatrix();
        T += dx.tail(3);
    }
}
// test function, can be used to verify your estimation
void calculateReprojectionError(const vector<cv::Point3f> &pts_3, const vector<cv::Point2f> &pts_2, const cv::Mat R, const cv::Mat t)
{
    //puts("calculateReprojectionError begins");
    vector<cv::Point2f> un_pts_2;
    cv::undistortPoints(pts_2, un_pts_2, K, D);
    double error_sum = 0;
    for (unsigned int i = 0; i < pts_3.size(); i++)
    {
        cv::Mat p_mat(3, 1, CV_64FC1);
        p_mat.at<double>(0, 0) = pts_3[i].x;
        p_mat.at<double>(1, 0) = pts_3[i].y;
        p_mat.at<double>(2, 0) = pts_3[i].z;
        cv::Mat p = (R * p_mat + t);
        
        //printf("(%f, %f, %f) -> (%f, %f) and (%f, %f)\n",
        //       pts_3[i].x, pts_3[i].y, pts_3[i].z,
        //       un_pts_2[i].x, un_pts_2[i].y,
        //       p.at<double>(0) / p.at<double>(2), p.at<double>(1) / p.at<double>(2));
               
        double reprojection_error = 460 * sqrt( (un_pts_2[i].x - p.at<double>(0) / p.at<double>(2)) * (un_pts_2[i].x - p.at<double>(0) / p.at<double>(2)) +
                                                (un_pts_2[i].y - p.at<double>(1) / p.at<double>(2)) * (un_pts_2[i].y - p.at<double>(1) / p.at<double>(2)));
        //cout << "reprojection error   " << reprojection_error << endl;
        error_sum += reprojection_error;
    }
    cout << "reprofection error " << error_sum / (1.0 * pts_3.size()) << endl;
    //puts("calculateReprojectionError ends");
}

// the main function you need to work with
// pts_id: id of each point
// pts_3: 3D position (x, y, z) in world frame
// pts_2: 2D position (u, v) in image frame
void process(const vector<int> &pts_id, const vector<cv::Point3f> &pts_3, const vector<cv::Point2f> &pts_2, const ros::Time& frame_time)
{
    cv::Mat r, rvec, t;

    //version 1, as reference
    TicToc opencv_pnp;
    cv::solvePnP(pts_3, pts_2, K, D, rvec, t);
    ROS_INFO("opencv pnp time %f", opencv_pnp.toc());
    cv::Rodrigues(rvec, r);
    //cout<<"opencv r"<<endl<< r <<endl;
    //cout<<"opencv t"<<endl<< t <<endl;
    ROS_INFO("opencv reprojection error");
    calculateReprojectionError(pts_3, pts_2, r, t);


    // version 2, your work

    vector<cv::Point2f> un_pts_2;
    TicToc opencv_undis;
    cv::undistortPoints(pts_2, un_pts_2, K, D);
    ROS_INFO("opencv undis time %f", opencv_undis.toc());

    int n ;
    n = 2 * pts_id.size();
    MatrixXd A = MatrixXd::Zero(n, 9);
    for(int i = 0; i< pts_id.size();i++)
    {

        A.row(2*i)<<pts_3[i].x, pts_3[i].y, 1,
        0,0,0,
        -1 * un_pts_2[i].x * pts_3[i].x, -1 * un_pts_2[i].x * pts_3[i].y, -1 * un_pts_2[i].x ;

        A.row(2*i+1)<<0,0,0,
        pts_3[i].x, pts_3[i].y, 1,
        -1 * un_pts_2[i].y * pts_3[i].x, -1 * un_pts_2[i].y * pts_3[i].y, -1 * un_pts_2[i].y ;
    }

    //cout<<"matrix"<< A <<endl;
    JacobiSVD<MatrixXd> svd(A, ComputeThinU | ComputeThinV);
    MatrixXd V = svd.matrixV();

    VectorXd M = V.col(V.cols()-1);
    //cout<<"M"<<endl<<M.transpose()<<endl;

    Matrix3d H;
    H.row(0) = M.segment<3>(0);
    H.row(1) = M.segment<3>(3);
    H.row(2) = M.segment<3>(6);
    //cout<<"H"<<endl<<H<<endl;

    H = 1.0 / H.col(0).norm() * H;
    //cout<<"norm H"<<endl<<H<<endl;
    Vector3d T = H.col(2);
    if(T(2)<0)
    {
        H = H * -1;
        T = H.col(2);
    }
    Matrix3d R = H;
    R.col(2) = H.col(0).cross(H.col(1));

    //cout<<" my result R"<<endl<<R<<endl;
    Quaterniond q(R);
    R = q.normalized().toRotationMatrix();
    //cout<<" my result R nonmilize"<<endl<<R<<endl;

    Matrix3d R_ref;
    for(int i=0;i<3;i++)
        for(int j=0;j<3;j++)
        {
            R_ref(i,j) = r.at<double>(i, j);
        }

    //cout<<" my result R"<<endl<<R<<endl;
    //cout<<" my result T"<<endl<<T.transpose()<<endl;


    Q = R_ref;

    nav_msgs::Odometry odom_ref;
      odom_ref.header.stamp = frame_time;
      odom_ref.header.frame_id = "world";
      odom_ref.pose.pose.position.x = t.at<double>(0, 0);
      odom_ref.pose.pose.position.y = t.at<double>(1, 0);
      odom_ref.pose.pose.position.z = t.at<double>(2, 0);
      odom_ref.pose.pose.orientation.w = Q.w();
      odom_ref.pose.pose.orientation.x = Q.x();
      odom_ref.pose.pose.orientation.y = Q.y();
      odom_ref.pose.pose.orientation.z = Q.z();

      pub_odom_ref.publish(odom_ref);
  



    //cout<<"my r"<<endl<<r_my<<endl;
    //cout<<"my t"<<endl<<t_my<<endl;
    //ROS_INFO("linear reprojection error");
    //calculateReprojectionError(pts_3, pts_2, r_my, t_my);

    //cv::Mat rvec_my;
    //cv::Rodrigues(r_my, rvec_my);
    //cv::solvePnP(pts_3, pts_2, K, D, rvec_my, t_my, true);
    //cv::Rodrigues(rvec_my, r_my);
    //calculateReprojectionError(pts_3, pts_2, r_my, t_my);

    // ***************ceres method*******************
    Matrix3d R_linear;
    Vector3d T_linear;
    R_linear = R;
    T_linear = T;
    Quaterniond Q_ceres(R);
    TicToc ceres_time;
    ceresMethod(pts_3, un_pts_2, Q_ceres, T);
    ROS_INFO("ceres time %f", ceres_time.toc());
    Q = Q_ceres;
    cv::Mat r_my, t_my;
    cv::eigen2cv(Q_ceres.toRotationMatrix(), r_my);
    cv::eigen2cv(T, t_my);
    ROS_INFO("ceres reprojection error");
    calculateReprojectionError(pts_3, pts_2, r_my, t_my);

    // **************newton method************************
    TicToc newton_time;
    newtonMethod(pts_3, un_pts_2, R_linear, T_linear);
    ROS_INFO("newton time %f", newton_time.toc());
    cv::eigen2cv(R_linear, r_my);
    cv::eigen2cv(T_linear, t_my);
    ROS_INFO("newton reprojection error");
    calculateReprojectionError(pts_3, pts_2, r_my, t_my);

    // **************pub odom****************************
    nav_msgs::Odometry odom;
    odom.header.stamp = frame_time;
    odom.header.frame_id = "world";
    odom.pose.pose.position.x = T(0);
    odom.pose.pose.position.y = T(1);
    odom.pose.pose.position.z = T(2);
    odom.pose.pose.orientation.w = Q_ceres.w();
    odom.pose.pose.orientation.x = Q_ceres.x();
    odom.pose.pose.orientation.y = Q_ceres.y();
    odom.pose.pose.orientation.z = Q_ceres.z();

    pub_odom.publish(odom);

}

cv::Point3f getPositionFromIndex(int idx, int nth)
{
    int idx_x = idx % 6, idx_y = idx / 6;
    double p_x = idx_x * MarkerWithMargin - (3 + 2.5 * 0.2) * MarkerSize;
    double p_y = idx_y * MarkerWithMargin - (12 + 11.5 * 0.2) * MarkerSize;
    return cv::Point3f(p_x + (nth == 1 || nth == 2) * MarkerSize, p_y + (nth == 2 || nth == 3) * MarkerSize, 0.0);
}

void img_callback(const sensor_msgs::ImageConstPtr &img_msg)
{
    double t = clock();
    cv_bridge::CvImagePtr bridge_ptr = cv_bridge::toCvCopy(img_msg, sensor_msgs::image_encodings::MONO8);
    MDetector.detect(bridge_ptr->image, Markers);
    float probDetect = TheBoardDetector.detect(Markers, TheBoardConfig, TheBoardDetected, CamParam, MarkerSize);
    //ROS_INFO("p: %f, time cost: %f\n", probDetect, (clock() - t) / CLOCKS_PER_SEC);

    vector<int> pts_id;
    vector<cv::Point3f> pts_3;
    vector<cv::Point2f> pts_2;
    for (unsigned int i = 0; i < Markers.size(); i++)
    {
        int idx = TheBoardConfig.getIndexOfMarkerId(Markers[i].id);

        char str[100];
        sprintf(str, "%d", idx);
        cv::putText(bridge_ptr->image, str, Markers[i].getCenter(), CV_FONT_HERSHEY_COMPLEX, 0.4, cv::Scalar(-1));
        for (unsigned int j = 0; j < 4; j++)
        {
            sprintf(str, "%d", j);
            cv::putText(bridge_ptr->image, str, Markers[i][j], CV_FONT_HERSHEY_COMPLEX, 0.4, cv::Scalar(-1));
        }

        for (unsigned int j = 0; j < 4; j++)
        {
            pts_id.push_back(Markers[i].id * 4 + j);
            pts_3.push_back(getPositionFromIndex(idx, j));
            pts_2.push_back(Markers[i][j]);
        }
    }

    //begin your function
    if (pts_id.size() > 5)
        process(pts_id, pts_3, pts_2, img_msg->header.stamp);

    cv::imshow("in", bridge_ptr->image);
    cv::waitKey(10);
}

int main(int argc, char **argv)
{
    //google::InitGoogleLogging(argv[0]);
    //FLAGS_logtostderr = 1;
    ros::init(argc, argv, "tag_detector");
    ros::NodeHandle n("~");

    ros::Subscriber sub_img = n.subscribe("image_raw", 100, img_callback);
    pub_odom = n.advertise<nav_msgs::Odometry>("odom",10);
    pub_odom_ref = n.advertise<nav_msgs::Odometry>("odom_ref",10);

    //init aruco detector
    string cam_cal, board_config;
    n.getParam("cam_cal_file", cam_cal);
    n.getParam("board_config_file", board_config);
    CamParam.readFromXMLFile(cam_cal);
    TheBoardConfig.readFromFile(board_config);

    //init intrinsic parameters
    cv::FileStorage param_reader(cam_cal, cv::FileStorage::READ);
    param_reader["camera_matrix"] >> K;
    param_reader["distortion_coefficients"] >> D;

    //init window for visualization
    cv::namedWindow("in", 1);

    ros::spin();
}
