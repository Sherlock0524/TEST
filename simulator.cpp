#include <string>
#include <iostream>
#include <fstream>
#include <thread>
#include <cstdlib>
#include <ctime>
#include <boost/format.hpp>
#include <LightTrack.hpp>
#include <OSTrack.hpp>
#include <ros/ros.h>
#include <ros/package.h>
#include <tf/transform_datatypes.h>
#include <gazebo_msgs/GetModelState.h>
#include <gazebo_msgs/SetModelState.h>
#include <geometry_msgs/PoseStamped.h>
#include <cv_bridge/cv_bridge.h>
#include <px4_cmd/single_vehicle_external_command.hpp>
#include <Eigen/Dense>
#include <matplotlibcpp.h>

using namespace std;

#define PI 3.14159
#define N 99


float todepth(cv::Mat img, float center_x, float center_y);
std::vector<float> pix2cam(float center_x, float center_y, float z);


// 数据结构定义：识别结果
struct targetPoint
{
    float xAngle = 0;
    float height = 0;
    float dist = 0;
    int flag = 0; // 1: insize view, 0: outsize view
};
targetPoint target_point;
// 全局变量定义
cv::Rect bbox;
bool drawing_box = false;
bool tracker_init = false;
bool box_init = false;
bool shutdown_flag = false;
bool ext_cmd_flag = false;

geometry_msgs::PoseStamped goal_point;
// ros 全局变量 - 订阅仿真图像信息
string color_camera_topic = "/uav_0/color/image_raw";  // RGB相机话题名
string depth_camera_topic = "/uav_0/depth/image_raw";  // 深度相机话题名
string goal_point_topic = "/move_base_simple/goal"; // 期望目标点（ENU系）话题名
cv::Mat color_image_from_topic; // 订阅得到的图像
cv::Mat depth_image_from_topic;
ros::Subscriber color_image_sub;                       // ros订阅
ros::Subscriber depth_image_sub;
ros::Publisher goal_point_pub;
ros::ServiceClient gazebo_client;
ros::ServiceClient gazebo_client_set;

float R[3] = {0};
float yaw_1 = 0.0;
float yaw = 0.0;
float R_real[3] = {0};
float value[3] = {0,0,0};
float value_now[3] = {0,0,0};
float value_i = 0.0;
float dt = 0.1;
float PID(float value_target,float value_now,float value_last)
{
    float kp = 0.1;
    float ki = 0.5;
    float kd = 0.5;

    float value_p = value_target - value_now;
    value_i += (value_target - value_now)*dt;
    float value_d = (value_now - value_last)/dt;

    float control_value = kp*value_p + ki*value_i + kd*value_d;
    return control_value;
}

//控制器
void controller(float *relative_position, float *desire_cmd_value)
{
   
    float a[3] = {0.5,0.5,0.5};      //控制系数
    float value_target[3];  //期望速度
    while(true)
    {
        for (int i=0;i<3;i++)
        {
            value_target[i]=a[i]*relative_position[i];
            float d_value=PID(value_target[i],value[i],value_now[i]);
            value_now[i] = value[i];
            value[i] = value[i] + d_value*dt;
            usleep(100000);
        }
        desire_cmd_value[0] = value[0];
        desire_cmd_value[1] = value[1];
        desire_cmd_value[2] = value[2];
        //ROS_INFO(" use controller");
        //ROS_INFO("value_now: [%.2f, %.2f, %.2f]", value_now[0], value_now[1], value_now[2]);
        //ROS_INFO("value: [%.2f, %.2f, %.2f]", value[0], value[1], value[2]);
        //ROS_INFO("value_target: [%.2f, %.2f, %.2f]", value_target[0], value_target[1], value_target[2]);
        //加载controller函数的最后面）
        ofstream oFile;
        oFile.open("data.csv",ios::out|ios::trunc);
        oFile<<"relative_position[0]"<<","<<"relative_position[1]"<<","<<"relative_position[2]"<<","<<"desire_cmd_value[0]"<<","<<"desire_cmd_value[1]"<<","<<"desire_cmd_value[2]"<<","<<"ball_position[0]"<<","<<"ball_position[1]"","<<"ball_position[2]"<<endl;
        break;
    }
}
// 鼠标回调
void mouse_callback(int event, int x, int y, int flags, void *userdata)
{
    switch (event)
    {
    case cv::EVENT_LBUTTONDOWN:
        drawing_box = true;
        tracker_init = false;
        // 开始一个新的框
        bbox = cv::Rect(x, y, 0, 0);
        break;
    case cv::EVENT_LBUTTONUP:
        // 标准化矩形（确保宽度和高度为正）
        if (bbox.width < 0)
        {
            bbox.x += bbox.width;
            bbox.width *= -1;
        }
        if (bbox.height < 0)
        {
            bbox.y += bbox.height;
            bbox.height *= -1;
        }
        drawing_box = false;
        // 判断矩形框初始化是否成功
        if (bbox.height != 0 && bbox.width != 0)
        {
            box_init = true;
        }
        break;
    case cv::EVENT_MOUSEMOVE:
        if (drawing_box)
        {
            // 更新矩形尺寸
            bbox.width = x - bbox.x;
            bbox.height = y - bbox.y;
        }
        break;
    }
}

// 限幅函数
void MinMax(float &input, float min, float max)
{
    if (input < min)
    {
        input = min;
    }
    else if (input > max)
    {
        input = max;
    }
}

// 球运动轨迹设计
void update_traj(float times, float *init_position, float *now_position)
{
    // now_position[0] = init_position[0] + 2 * sin(times);
    // now_position[1] = init_position[1] + 2 * cos(times);
    // srand(time(NULL));
    // now_position[2] = init_position[2] + (rand() % (N + 1) / (float)(N + 1)) * 0;
    // //now_position[2] = init_position[2];
    now_position[0] = init_position[0] + times;
    now_position[1] = init_position[1] + times;
    now_position[2] = init_position[2] + times;
}

// 发送目标点线程函数
void send_goal_point()
{
    while (!shutdown_flag)
    {
        if (target_point.flag == 1 && ext_cmd_flag)
        {
            goal_point_pub.publish(goal_point);
            ROS_INFO("send goal point");
        }
        sleep(3);
    }
}

// 发送无人机指令线程函数
void send_ext_cmd()
{
    single_vehicle_external_command ext_cmd;
    ext_cmd.total_time = -1;
    ext_cmd.start();
    sleep(1);
    float minDist = 1;
    // int reverse_flag;
    float desire_cmd_value[3];
    bool init = true;
    int fail_count = 0;
    float last_pos = 0.0;
    float yaw_value = 0.0;
    float ball_init_position[3] = {0};
    float ball_position[3] = {0};
    float times = 0;
    goal_point.pose.position.z = 0;
    // 创建发送目标点的线程
    std::thread goal_thread(send_goal_point);
    goal_thread.detach();

    // 更新位置
    tf::Quaternion quad;
    tf::Matrix3x3 matrix;
    tf::Vector3 vec;
    float relative_position[3] = {0};
    gazebo_msgs::GetModelState ball;
    gazebo_msgs::SetModelState ball_set;
    ball.request.model_name = "ball"; // 指定要获取的机器人在gazebo中的名字；
    ball_set.request.model_state.model_name = "ball";
    ball_set.request.model_state.reference_frame = "world";
    ball_set.request.model_state.pose.orientation.w = 1;
    ball_set.request.model_state.pose.orientation.x = 0;
    ball_set.request.model_state.pose.orientation.y = 0;
    ball_set.request.model_state.pose.orientation.z = 0;
    ball_set.request.model_state.twist.linear.x = 0.0;
    ball_set.request.model_state.twist.linear.y = 0.0;
    ball_set.request.model_state.twist.linear.z = 2.0;
    ball_set.request.model_state.twist.angular.x = 0.0;
    ball_set.request.model_state.twist.angular.y = 0.0;
    ball_set.request.model_state.twist.angular.z = 0.0;
    // 获取目标球真实位姿
    gazebo_client.call(ball);
    ball_position[0] = ball.response.pose.position.x;
    ball_position[1] = ball.response.pose.position.y;
    ball_position[2] = ball.response.pose.position.z;
    ball_init_position[0] = ball.response.pose.position.x;
    ball_init_position[1] = ball.response.pose.position.y;
    ball_init_position[2] = ball.response.pose.position.z;

    while (ros::ok())
    {
        
        while (!ext_cmd.ext_cmd_state && !shutdown_flag)
        {
            ext_cmd_flag = false;
            init = true;
            ROS_INFO("External Command: Waiting for user-define mode!");
            ros::Duration(1).sleep();
            //usleep(200000);
            if (shutdown_flag)
            {
                return;
            }
        }
        
        // 更新目标位置
        ext_cmd_flag = true;
        ros::spinOnce();
        // 更新目标球轨迹
        update_traj(times, ball_init_position, ball_position);
        // ROS_INFO("ball Position: [%.2f, %.2f, %.2f]", ball_position[0], ball_position[1], ball_position[2]);
        ball_set.request.model_state.pose.position.x = ball_position[0];
        ball_set.request.model_state.pose.position.y = ball_position[1];
        ball_set.request.model_state.pose.position.z = ball_position[2];
        times += 0.04;
        gazebo_client_set.call(ball_set);
        if (true)
        {
            // 如果获取服务成功了的话，
            // ball.response就是请求服务后返回的数据，可以随便使用了。
            gazebo_client.call(ball);
            ball_position[0] = ball.response.pose.position.x;
            ball_position[1] = ball.response.pose.position.y;
            ball_position[2] = ball.response.pose.position.z;
            relative_position[0] = ball_position[0] - ext_cmd.position[0];
            relative_position[1] = ball_position[1] - ext_cmd.position[1];
            relative_position[2] = ball_position[2] - ext_cmd.position[2];
            vec = tf::Vector3(relative_position[0], relative_position[1], relative_position[2]);
            matrix.setRotation(ext_cmd.quaternion); // 机体系转绝对系
            matrix = matrix.inverse(); // 取逆代表绝对系转机体系
            vec = matrix * vec; // 旋转变换 旋转矩阵 * 当前系向量
            relative_position[0] = vec[0];
            relative_position[1] = vec[1];
            relative_position[2] = vec[2];
            R_real[0] = relative_position[0];
            R_real[1] = relative_position[1];
            R_real[2] = relative_position[2];

           // 获取目标真实位置
            goal_point.pose.position.x = ball_position[0];
            goal_point.pose.position.y = ball_position[1];
            goal_point.pose.position.z = ball_position[2];

            
        }
        else
        {
            ROS_ERROR("Failed to call service /gazebo/get_model_state");
            continue;
        }

        if (target_point.flag == 1) // 检测到目标则发布目标位置同时获取指令
        {
            // 目标位置更新
            goal_point.pose.position.x = ball_position[0];
            goal_point.pose.position.y = ball_position[1];
            goal_point.pose.position.z = ball_position[2];
            //ROS_INFO("相对位置: [%.2f, %.2f, %.2f]", relative_position[0], relative_position[1], relative_position[2]);
            ROS_INFO("即将运行速度控制函数");
            controller(relative_position, desire_cmd_value); 
            /*
            控制器
            输出：desire_cmd_value ： 3轴速度
            yaw_value：偏航角指令
            */
           // 发送速度指令
           //ROS_INFO("即将发送速度指令");
           //ext_cmd.set_velocity(desire_cmd_value[0], desire_cmd_value[1], desire_cmd_value[2], yaw_value = -90+yaw, px4_cmd::Command::BODY);
           ext_cmd.set_velocity(desire_cmd_value[0], desire_cmd_value[1], desire_cmd_value[2], yaw_value = -90 , px4_cmd::Command::BODY);
           ROS_INFO("111已发送速度指令:[%.2f, %.2f, %.2f]", desire_cmd_value[0], desire_cmd_value[1], desire_cmd_value[2]);
           
        }
        else
        {
            ext_cmd.set_hover(ext_cmd.attitude[2]);
        }
        ros::Duration(0.1).sleep();
        ros::spinOnce();
    }
}

template <class T> // 模板声明，定义一个模板类或函数
///////////////////////////////////////////////////////////////////////////
///////////////////////////////////////////////////////////////////////////
void LaunchTrack(shared_ptr<T> tracker)
{
    string display_name = "Track"; //
    cv::namedWindow(display_name, cv::WINDOW_AUTOSIZE); // 创建窗口
    cv::setMouseCallback(display_name, mouse_callback); // 鼠标回调事件

    // 初始化
    ros::spinOnce();
    sleep(1); // 休眠1秒
    cv::Mat img = color_image_from_topic.clone(); // 将相机彩色图像输入
    cv::Mat dep = depth_image_from_topic.clone();
    cv::imshow(display_name, img);

    while (true)
    {
        // 更新图像
        ros::spinOnce();
        img = color_image_from_topic.clone();
        dep = depth_image_from_topic.clone();
        // 初始化 Tracker
        if (!tracker_init) //判断是否为假，即未初始化
        {
            target_point.flag = 0;
            if (box_init) //
            {
                tracker->init(img, bbox); //初始化
                tracker_init = true; //已初始化，设置为真
                box_init = false;  //
            }
            if (drawing_box && bbox.width > 0 && bbox.height > 0)
            {
                cv::rectangle(img, bbox, cv::Scalar(255, 0, 0), 2); //绘制一个矩形框，蓝色，线宽为2
            }
        }
        wz=zeros[1000,3];
        // 开始跟踪目标
        n=0;
        else
        {
            target_point.flag = 1;
            auto start = std::chrono::steady_clock::now();
            bbox = tracker->track(img);
            auto end = std::chrono::steady_clock::now();
            std::chrono::duration<double> elapsed = end - start;
            double time = 1000 * elapsed.count();
            //printf("all infer time: %f ms\n", time);
            cv::rectangle(img, bbox, cv::Scalar(0, 255, 0), 2);
            float center_x = bbox.x + bbox.width / 2;
            float center_y = bbox.y + bbox.height / 2;
            std::vector<float> r =  pix2cam(center_x, center_y, todepth( dep, center_x, center_y));
            R[0] = r[0];
            R[1] = r[1];
            R[2] = r[2];
            ROS_INFO("测量/真实相对位置: [%.2f, %.2f, %.2f] / [%.2f, %.2f, %.2f]", R[0], R[1], R[2], R_real[0], R_real[1], R_real[2]);
           plt::figure_size(1200,780);
           plt::plot(R[0],t,"r--");
            //求yaw
            hold on;
            yaw_1 = atan(R[1]/R[0])*180.0f/PI;
            if ((yaw_1 > 1 || yaw_1 < -1) && yaw_1 != NAN)
            {
                yaw = yaw_1;
            }        
            //ROS_INFO("yaw_1 , yaw : [%.2f , %.2f]", yaw_1, yaw);
        }

        cv::putText(img, "Esc to quit, r to reset", cv::Point2i(20, 30),
                    cv::FONT_HERSHEY_SIMPLEX, 1, cv::Scalar(255, 0, 0), 1);

        cv::imshow(display_name, img);

        int key = cv::waitKey(30);

        // 按 Esc 退出
        if (key == 27)
            break;

        // 按 "r" 或单击鼠标重置边界框
        else if (key == 114 || drawing_box)
        {
            tracker_init = false;
            box_init = false;
        }
        
        if (!ros::master::check())
        {
            return;
        }
    }
    cv::destroyAllWindows();
}

// 图像回调函数
void color_image_raw_sub(const sensor_msgs::Image::ConstPtr &msg)
{
    cv_bridge::CvImagePtr cv_ptr;
    sensor_msgs::Image current_state = *msg;
    cv_ptr = cv_bridge::toCvCopy(msg, msg->encoding);
    cv::cvtColor(cv_ptr->image, color_image_from_topic, cv::COLOR_RGB2BGR);
}

void depth_image_raw_sub(const sensor_msgs::Image::ConstPtr &msg)
{
    cv_bridge::CvImagePtr cv_ptr;
    sensor_msgs::Image current_state = *msg;
    cv_ptr = cv_bridge::toCvCopy(msg, msg->encoding);
    depth_image_from_topic = cv_ptr->image;
}

//求相对的位置
std::vector<float> pix2cam(float center_x, float center_y, float z) {
    //设置相机内参
    Eigen::Matrix3d K, inv_K, RT;
    Eigen::Vector3d I, R;
    K << 462.266357421875,  0.0,                320.0,
         0.0,               462.266357421875,   240.0,
         0.0,               0.0,                1.0;
    I << center_x, center_y, 1;
    RT << 0, 0, 1,
          -1, 0, 0,
          0, -1, 0;
    float ans = 0;
    //求逆矩阵
    inv_K = K.inverse();
    //转换坐标系
    R = RT * inv_K * I * z;
    return {R.x(), R.y(), R.z()};
}
//将深度相机的信息转换成深度信息
float todepth(cv::Mat img, float center_x, float center_y) {
    uint16_t depth = img.at<uint16_t>(round(center_x), round(center_y));
    float z = 0.001 * depth;
    return z;
}



//////////////////////////////////////////////////////////////////////////////////////////////////////////////////
int main(int argc, char *argv[])
{
    // ros 初始化
    ros::init(argc, argv, "faster_uav_tracker_simulator"); // 指定节点名称
    ros::NodeHandle nh("~");
    ROS_INFO("faster-uav-tracker-simulator Start! Color Camera Topic: [%s], Depth Camera Topic: [%s]", color_camera_topic.c_str(), depth_camera_topic.c_str());
    color_image_sub = nh.subscribe<sensor_msgs::Image>(color_camera_topic, 20, color_image_raw_sub);
    depth_image_sub = nh.subscribe<sensor_msgs::Image>(depth_camera_topic, 20, depth_image_raw_sub);
    goal_point_pub = nh.advertise<geometry_msgs::PoseStamped>(goal_point_topic, 1);
    gazebo_client = nh.serviceClient<gazebo_msgs::GetModelState>("/gazebo/get_model_state");
    gazebo_client_set = nh.serviceClient<gazebo_msgs::SetModelState>("/gazebo/set_model_state");
    // 等待图像话题
    while (color_image_sub.getNumPublishers() < 1 || depth_image_sub.getNumPublishers() < 1)
    {
        ROS_WARN_STREAM("Recive_Camera: Can not Connet to Camera Topic [" << color_camera_topic << "] , Retrying ...");
        ros::spinOnce();
        ros::Duration(1).sleep();
    }
    // 模型信息
    string project_path = ros::package::getPath("faster_uav_tracker_simulator");
    string z_path = project_path + "/pretraining/lighttrack-z.trt";
    string x_path = project_path + "/pretraining/lighttrack-x-head.trt";
    string engine_path = project_path + "/pretraining/ostrack-256.trt";

    //记录数据（加在main函数里面，最好在前面）
    ofstream oFile;
    oFile.open("data.csv",ios::out|ios::trunc);
    oFile<<"relative_x"<<","<<"relative_y"<<","<<"relative_z"<<","<<"value_x"","<<"value_y"","<<"value_z"","<<"ball_x"<<","<<"ball_y"","<<"ball_z"<<endl;


    // 初始化跟踪器
    auto tracker = LightTrack::create_tracker(z_path, x_path);
    // auto tracker = OSTrack::create_tracker(engine_path);
    if (tracker == nullptr)
    {
        ROS_ERROR("Tracker is nullptr.\n");
        return -1;
    }

    // 发送目标点
    std::thread ros_thread(send_goal_point);
    ros_thread.detach();
    // 启动发送指令线程
    std::thread ros_thread_2(send_ext_cmd);
    ros_thread_2.detach();

    // 启动跟踪器
    LaunchTrack(tracker);

    //检查ROS的主节点是否在运行
    if (!ros::master::check())
    {
        ROS_ERROR("ROSCORE not running! Exit ...");
        return -1;
    }

    // 结束
    cv::destroyAllWindows();
    shutdown_flag = true;
    sleep(5);
    nh.shutdown();
    //加在return 0那里
    //oFile.close;
    return 0;
}


//void controller(double *relative_position, double *desire_cmd_value)
//{

//}




  /*
        // PID参数
        float xP = 0.5;
        float xD = 0.3;
        float yP = 4.0;
        float yD = 15;
        float zP = 0.5;
        float zD = 0.3;

        // 控制目标
        float targetXA = 0.0;
        float targetHi = -0.15;
        float targetDi = 1.8;

        // 上一时刻差值
        float lastErrXA = 0.0;
        float lastErrHi = 0.0;
        float lastErrDi = 0.0;
        float lastYaw = 0.0;

        float vex = 0.0;
        float vey = 0.0;

        float last_vex = 0.0;
        float last_vey = 0.0;

        float Kp = 0.01;
        float Kd = 0.01;

        float secs = 0.0;
        tf::Quaternion quad;
        tf::Matrix3x3 matrix;
        tf::Vector3 vec;
        while (ros::ok())
        {
            while (!ext_cmd.ext_cmd_state && !shutdown_flag)
            {
                init = true;
                ROS_INFO("External Command: Waiting for user-define mode!");
                ros::Duration(1).sleep();
                if (shutdown_flag)
                {
                    return;
                }
            }
            // 更新位置
            ros::spinOnce();
            update_traj(times, ball_init_position, ball_position);
            ball_set.request.model_state.pose.position.x = ball_position[0];
            ball_set.request.model_state.pose.position.y = ball_position[1];
            ball_set.request.model_state.pose.position.z = ball_position[2];
            times += 0.04;
            if (gazebo_client_set.call(ball_set))
            {
                // 如果获取服务成功了的话，
                // ball.response就是请求服务后返回的数据，可以随便使用了。
                relative_position[0] = ball_position[0] - ext_cmd.position[0];
                relative_position[1] = ball_position[1] - ext_cmd.position[1];
                relative_position[2] = ball_position[2] - ext_cmd.position[2];
                vec = tf::Vector3(relative_position[0], relative_position[1], relative_position[2]);
                matrix.setRotation(ext_cmd.quaternion);
                matrix = matrix.inverse();
                vec = matrix * vec;
                relative_position[0] = vec[0];
                relative_position[1] = vec[1];
                relative_position[2] = vec[2];
                target_point.xAngle = atan2(relative_position[1], relative_position[0]);
                target_point.height = relative_position[2];
                target_point.dist = relative_position[0];
                // ROS_INFO("Relative Position: [%.2f, %.2f, %.2f]", relative_position[0], relative_position[1], relative_position[2]);
            }
            else
            {
                ROS_ERROR("Failed to call service /gazebo/get_model_state");
            }
            if (init)
            {
                yaw_value = 0 - PI / 2;
                init = false;
            }
            lastYaw = yaw_value;
            if (target_point.flag == 1) // 检测到目标
            {
                float errorXA = target_point.xAngle - targetXA;
                float errorHi = target_point.height - targetHi;
                float errorDi = target_point.dist - targetDi;
                ROS_INFO("Error: [%.2f, %.2f, %.2f]", errorXA, errorHi, errorDi);
                if (errorXA > -PI / 36.0 && errorXA < PI / 36.0)
                {
                    errorXA = 0; // if errorXA<5deg, errorXA=0
                }

                if (errorHi > -0.1 && errorHi < 0.1)
                {
                    errorHi = 0;
                }

                if (errorDi > -0.1 && errorDi < 0.1)
                {
                    errorDi = 0;
                }

                // 无人机 X 方向速度控制
                desire_cmd_value[0] = errorDi * xP + (errorDi - lastErrDi) * xD;
                // 无人机 Z 方向速度控制
                desire_cmd_value[2] = errorHi * zP + (errorHi - lastErrHi) * zD;
                // 无人机 yaw value 控制
                yaw_value += errorXA / 10.0;

                // if (errorDi < 0.25){
                //     yaw_value = yaw_value;
                // }
                // else {
                // }

                // 更新上一时刻差值
                lastErrXA = errorXA;
                lastErrHi = errorHi;
                lastErrDi = errorDi;
            }

            else // 当前没有找到目标，执行目标搜索
            {
                desire_cmd_value[0] = 0;
                // desire_cmd_value[1] = 0;
                desire_cmd_value[2] = 0;

                // ROS_INFO("No Target!!");
                // yaw_value += PI/20;
                if (lastErrXA < 0) // 框在左边，飞机在右边
                {
                    yaw_value += PI / 1440;
                }
                else if (lastErrXA > 0) // 框在右边，飞机在左边
                {
                    yaw_value -= PI / 1440;
                }
                else
                {
                    yaw_value += PI / 1440;
                }
            } // end else

            desire_cmd_value[1] = 0;

            // 速度限幅
            MinMax(desire_cmd_value[0], -0.75, 0.75);
            // MinMax(desire_cmd_value[1], -0.2, 0.2);
            MinMax(desire_cmd_value[2], -0.75, 0.75);
            if (yaw_value - lastYaw > 0.1)
            {
                yaw_value = lastYaw + 0.1;
            }
            else if (yaw_value - lastYaw < -0.1)
            {
                yaw_value = lastYaw - 0.1;
            }
            lastYaw = yaw_value;
            ext_cmd.set_velocity(desire_cmd_value[0], desire_cmd_value[1], desire_cmd_value[2], yaw_value, px4_cmd::Command::BODY);
            ros::Duration(0.1).sleep();
            ros::spinOnce();
        }
        */