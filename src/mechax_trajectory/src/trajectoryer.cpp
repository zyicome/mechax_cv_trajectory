//------------------------------------------------------------------------------------
//注：所有计算过程使用弧度制，串口传输过程使用角度制
//------------------------------------------------------------------------------------


#include "trajectoryer.hpp"

//两个内联函数，用于牛顿迭代法，第一个为f(x)，第二个为f'(x)
inline float ft0(float t0,float randa,float tan,float z0,float v0)
{
    return (g / randa) * ((1/randa) * log(1 - randa * t0) + t0) + tan - z0; 
}

inline float f_t0(float t0,float randa,float tan,float z0,float v0)
{
    return (g / randa) * (-randa / randa / (1 - randa * t0) + 1) + (v0 * v0 * t0 / tan);
}

//两个内联函数，用于二次空气阻力模型的牛顿迭代法，第一个为z(x),第二个为z'(x)
inline float z0(float vz0, float z0, float randa, float alpha, float beta)
{
    float z = 0.0;
    if(vz0 <= 0)
    {
        z = -z0 - log(coshf(alpha) - tan(beta) * sinhf(alpha)) / randa;
    }
    else if(vz0 > 0 && alpha <= beta)
    {
        z = -z0 + (1 / randa) * log(cos(beta - alpha) / cos(beta));
    }
    else if(vz0 > 0 && alpha > beta)
    {
        z = -z0 - log(cos(beta) * coshf(alpha - beta)) / randa;
    }
    return z;
}

inline float z_0(float vz0, float z0, float randa,float alpha,float beta, float alpha_angle, float beta_angle)
{
    float z = 0.0;
    if(vz0 <= 0)
    {
        z = -(sinhf(alpha) * alpha_angle - ((1 / cos(beta)) * (1 / acos(beta)) * beta_angle * sinhf(alpha) + tan(beta) * coshf(alpha) * alpha_angle)) / (randa * (coshf(alpha) - tan(beta) * sinhf(alpha)));
    }
    else if(vz0 > 0 && alpha <= beta)
    {
        z = (1 / randa) * (-tan(beta - alpha) * (beta_angle - alpha_angle) + tan(beta) * beta_angle);
    }
    else if(vz0 > 0 && alpha > beta)
    {
        z = (1 / randa) * ((tan(beta) * beta_angle) - tanhf(alpha - beta) * (alpha_angle - beta_angle));
    }
    return z;
}


Trajectoryer::Trajectoryer() : Node("trajectory")
{
    parameters_init();

    tf2_init();

    left_camera_target_sub_ = this->create_subscription<auto_aim_interfaces::msg::Target>(
                  "/left_camera/tracker/target", rclcpp::SensorDataQoS(), std::bind(&Trajectoryer::left_camera_target_callback, this, std::placeholders::_1));

    angle_sub_ = this->create_subscription<auto_aim_interfaces::msg::ReceiveSerial>(
        "/angle/init", 10, std::bind(&Trajectoryer::angle_callback, this, std::placeholders::_1));

    changeyaw_sub_ = this->create_subscription<auto_aim_interfaces::msg::Bias>(
        "/left_camera/trajectory/changeyaw", 10, std::bind(&Trajectoryer::changeyaw_callback, this, std::placeholders::_1));
    
    maker_pub_ = this->create_publisher<visualization_msgs::msg::Marker>(
        "/aiming_point", 10);

    result_pub_ = this->create_publisher<auto_aim_interfaces::msg::SendSerial>(
        "/trajectory/result", 10);
    
    needpose_pub_ = this->create_publisher<geometry_msgs::msg::PointStamped>(
        "/left_camera/trajectory/needpose", 10);

    armorpose_pub_ = this->create_publisher<geometry_msgs::msg::PointStamped>(
        "/left_camera/trajectory/armorpose", 10);

    left_camera_pub_ = this->create_publisher<auto_aim_interfaces::msg::Inter>(
        "/left_camera/result", 10);

    tf_broadcaster_ = std::make_unique<tf2_ros::TransformBroadcaster>(*this);

    front_assist_target_sub_ = this->create_subscription<auto_aim_interfaces::msg::Target>(
        "/front_assist/tracker/target", rclcpp::SensorDataQoS(), std::bind(&Trajectoryer::front_assist_target_callback, this, std::placeholders::_1));

    back_assist_target_sub_ = this->create_subscription<auto_aim_interfaces::msg::Target>(
        "/back_assist/tracker/target", rclcpp::SensorDataQoS(), std::bind(&Trajectoryer::back_assist_target_callback, this, std::placeholders::_1));

    right_camera_sub_ = this->create_subscription<auto_aim_interfaces::msg::Inter>(
        "/right_camera/result", rclcpp::SensorDataQoS(), std::bind(&Trajectoryer::right_result_callback, this, std::placeholders::_1));
   //timer_ = this->create_wall_timer(5s, std::bind(&Trajectoryer::test,this));
}

//用于参数初始化，主要需要修改的参数为
// is_hero, v0，randa，bias_t
void  Trajectoryer::parameters_init()
{
    //----------------------------------------------------
    is_hero = false; // 根据情况自己修改，英雄大弹丸为1,步兵小弹丸为0
    //----------------------------------------------------
    if(is_hero)
    {
        v0 = 16; // m/s
    }
    else
    {
        v0 = 27; // m/s
    }
    //****************************************************
    // 用来计算空气阻力系数
    float c = 0.5; 
    float p = 1.204; // kg/m3
    float d_small = 16.8 / 1000; // mm->m
    float d_big = 42.5 / 1000;  // mm->m   
    float m_small =  3.2 / 1000; // g->kg
    float m_big = 44.5 / 1000; // g->kg
    float s_small = M_PI * pow(d_small / 2, 2);
    float s_big = M_PI * pow(d_big / 2, 2);
    float randa_small =  (c * p * s_small) / 2 / m_small;
    float randa_big = (c * p * s_big) / 2 / m_big;
    if(is_hero)
    {
        randa = randa_big;
    }
    else
    {
        randa = randa_small;
    }
    //****************************************************
    //****************************************************
    bias_t = 0.00;  // s
    fly_t = 0.5; // s
    //摄像头相对于云台的偏置,一般改z_bias即可
    y_bias = 0.0; // m
    z_bias = 0.0; // m
    //****************************************************
    needchangeyaw = 0.0;
    is_left_tracking = 0;
    is_right_tracking = 0;
    is_assist_tracking = 0;
    is_left_can_hit = 0;
    is_right_can_hit = 0;
    angle_bigyaw = 0.0;
    left_angle_pitch = 0.0;
    left_angle_yaw = 0.0;
    right_angle_pitch = 0.0;
    right_angle_yaw = 0.0;
}

//@param: object_x, object_y, object_z, v0
// 无空气阻力模型，最简单的模型，只考虑重力加速度，用来判断是否能够击打到目标
//@result: angle_pitch, fly_t，不能直接使用，用来作为初始化数据, 需要用空气阻力模型进行修正
//@return: 1:可以击打 0:无法击打到目标
int Trajectoryer::no_resistance_model(float &angle_pitch,const float &object_x,const float &object_y,const float &object_z,const float &v0)
{
    float distance = sqrtf(pow(object_x, 2) + pow(object_y, 2));
    float alpha = 0.0;
    float l = sqrtf(pow(object_x,2) + pow(object_y,2) + pow(object_z,2));
    /*if(!is_solvable(object_x, object_y, object_z, v0, alpha))
    {
        return 0;
    }*/
    alpha = asinf((object_z + g*distance*distance / (v0*v0)) / l);
    float process_angle = atanf(object_z / distance);
    float angle_high = (alpha + process_angle) / 2;
    float angle_low = (M_PI + process_angle - alpha) / 2;
    angle_pitch = (fabs(angle_high - now_pitch) > fabs(angle_low - now_pitch)) ? angle_low : angle_high;
    fly_t = distance / (v0 * cos(angle_pitch));
    return 1;
}

//@param: object_x, object_y, object_z, v0, randa
// 根据传入的相对于枪管坐标系下敌方的坐标xyz，在结合子弹速度和空气阻力系数，计算出需要的pitch角度和飞行时间
// 单空气阻力模型，运用牛顿迭代法，迭代10次，精度为0.001
//@result: angle_pitch, fly_t (成员变量，刷新得到)
//@return: 1:计算成功 0:无法迭代到结果精度或者迭代次数过多
int Trajectoryer::single_resistance_model(float &angle_pitch,const float &object_x,const float &object_y,const float &object_z,const float &v0,const float &randa)
{
    float distance = sqrtf(pow(object_x, 2) + pow(object_y, 2));
    if(!no_resistance_model(angle_pitch,object_x, object_y, object_z, v0))
    {
        return 0;
    }
    float diedai_t = 0.0;
    float tan = 0.0;
    for(int i = 1; i <= 10; i++)
    {
        tan = sqrt(v0 * v0 * fly_t * fly_t - distance * distance);
        if(angle_pitch < 0)
        {
            tan = -tan;
        }
        diedai_t = fly_t - (ft0(fly_t, randa, tan, object_z, v0) / f_t0(fly_t, randa, tan, object_z, v0));
        fly_t = diedai_t;
        if(i == 10)
        {
            fabs(ft0(fly_t, randa, tan, object_z, v0)) > 0.01;
            RCLCPP_INFO(get_logger(), "the mistake is too large!");
            return 0;
        }
        if(fabs(ft0(fly_t, randa, tan, object_z, v0)) < 0.001)
        {
            cout << "di"  << i << "end";
            break;
        }
    }
    angle_pitch = acos(distance / v0 / fly_t);
    return 1;
}


//@param: object_x, object_y, object_z, v0, randa
// 根据传入的相对于枪管坐标系下敌方的坐标xyz，在结合子弹速度和空气阻力系数，计算出需要的pitch角度和飞行时间
// 单空气阻力模型
//@result: angle_pitch, fly_t  (成员变量，刷新得到)
//@return: 1:计算成功 0:计算失败
int Trajectoryer::single_resistance_model_two(float &angle_pitch,const float &object_x,const float &object_y,const float &object_z,const float &v0,const float &randa)
{
    float distance = sqrtf(pow(object_x, 2) + pow(object_y, 2));
    if(!no_resistance_model(angle_pitch,object_x, object_y, object_z, v0))
    {
        return 0;
    }
    float k1 = 0.0;
    // float k1 = 0.019;//小弹丸的空气阻力系数
    if(is_hero)
    {
        k1 = randa * 44.5 / 1000;
    }
    else
    {
        k1 = randa * 3.2 / 1000;
    }
    float a = (exp(k1*distance) - 1) / k1;
    float b = (g * pow(exp(k1*distance)-1,2)) / (2*pow(k1,2)*pow(v0,2));
    float delta = pow(a,2) - 4*b*(object_z+b);
    float tan_angle_1 = (a+sqrt(delta)) / (2*b);
    float tan_angle_2 = (a-sqrt(delta)) / (2*b);
    float angle_init = atan2(object_z, distance);	//rad弧度，补偿前的角度
    float angle_actual_1 = -atan(tan_angle_1);
    float angle_actual_2 = -atan(tan_angle_2);//rad
    angle_pitch = (fabs(angle_actual_1 - now_pitch) > fabs(angle_actual_2 - now_pitch)) ? angle_actual_2 : angle_actual_1;//取绝对值小的那个 
    fly_t = (float)((exp(k1 * distance) - 1) / (k1 * v0 * cos(angle_pitch)));//更新飞行时间
    return 1;
}

//@param: object_x, object_y, object_z, v0, randa
//根据传入的相对于枪管坐标系下敌方的坐标xyz，在结合子弹速度和空气阻力系数，计算出需要的pitch角度和飞行时间
//双空气阻力模型
//@result: angle_pitch, fly_t  (成员变量，刷新得到)
//@return: 1:计算成功 0:计算失败
int Trajectoryer::two_resistance_model(float &angle_pitch,const float &object_x,const float &object_y,const float &object_z,const float &v0,const float &randa)
{
    float distance = sqrtf(pow(object_x, 2) + pow(object_y, 2));
    if(!no_resistance_model(angle_pitch,object_x, object_y, object_z, v0))
    {
        return 0;
    }
    float diedai_angle = 0.0;
    float vx0 = 0.0;
    float vz0 = 0.0;
    float alpha = 0.0;
    float beta = 0.0;
    float alpha_angle = 0.0;
    float beta_angle = 0.0;
    float t = 0.0;
    float z1;
    float z2;
    float vt = sqrtf(g / randa);
    for(int i = 0; i < 20; i++)
    {
        vx0 = v0 * cos(angle_pitch);
        vz0 = v0 * sin(angle_pitch);
        t = (exp(randa * distance) - 1) / (randa * v0 * cos(angle_pitch));
        alpha = sqrt(g * randa) * t;
        beta = atan(vz0 * sqrt(randa / g));
        alpha_angle = alpha * tan(angle_pitch);
        beta_angle = (v0 * cos(angle_pitch) * cos(beta) * cos(beta)) / vt;
        z1 = z0(vz0, object_z, randa, alpha, beta);
        if(fabs(z1) < 0.001)
        {
            break;
        }
        z2 = z_0(vz0, object_z, randa, alpha, beta, alpha_angle, beta_angle);
        diedai_angle = angle_pitch - z1 / z2;
        angle_pitch = diedai_angle;
        if(i == 19)
        {
            vx0 = v0 * cos(angle_pitch);
            vz0 = v0 * sin(angle_pitch);
            t = (exp(randa * distance) - 1) / (randa * vx0);
            alpha = sqrt(g * randa) * t;
            beta = atan(vz0 * sqrt(randa / g));
            alpha_angle = alpha * tan(angle_pitch);
            beta_angle = (v0 * cos(angle_pitch) * cos(beta) * cos(beta)) / vt;
            z1 = z0(vz0, object_z, randa, alpha, beta);
            if(fabs(z1) < 0.001)
            {
                break;
            }
            else
            {
                return 0;
            }
        }
    }
    fly_t = (exp(randa * distance) - 1) / (randa * v0 * cos(angle_pitch));
    return 1;
}


//@param: object_x, object_y, object_z, v0, alpha(需更改，所以不能使用const)
// 运用简单的无空气阻力模型判断是否能够击打到目标
//@return: 1:可以击打 0:无法击打到目标
bool Trajectoryer::is_solvable(const float &object_x,const float &object_y,const float &object_z,const float &v0, float &alpha)
{
    float l = sqrtf(pow(object_x, 2) + pow(object_y, 2) + pow(object_z, 2));
    float distance = sqrtf(pow(object_x, 2) + pow(object_y, 2));
    float process = object_z  + g * pow(distance, 2) /  pow(v0, 2);
    if(distance >= process)
    {
        alpha = asinf(process / distance);
        return true;
    }
    RCLCPP_INFO(get_logger(), "Can not solve!");
    return false;
}

//@param: ros_x, vx, ros_y, vy, ros_z, vz, r_1, r_2, dz, v0, randa (需要的参数过多，利用成员变量管理，更方便)
int Trajectoryer::solve_trajectory()
{
//----------------------------------------------
//进行预测，预测出击打目标的位置
    vector<result> results;
    float need_t = fly_t + bias_t;
    float yaw_delay = need_t * v_yaw;
    float tar_yaw = yaw + yaw_delay;
    int use_1 = 1;
    int i = 0;
    int idx = 0;
//进行预测，预测出击打目标的位置
    ros_x = ros_x + vx * need_t;
    ros_y = ros_y + vy * need_t;
    ros_z = ros_z + vz * need_t;
//----------------------------------------------
//----------------------------------------------
//进行选板，选择最适合击打的装甲板
    float xiangdui_yaw = 0.0;
    if(ros_y < 0)
    {
        xiangdui_yaw = atan2(ros_y, ros_x);
    }
    else
    {
        xiangdui_yaw = atan2(ros_y, ros_x) + M_PI;
    }
    if(armor_num == 2)
    {
        for (i = 0; i<2; i++) {
        result position_result;
        float tmp_yaw = tar_yaw + i * M_PI;
        float r = r_1;
        position_result.x = ros_x - r*cos(tmp_yaw); 
        position_result.y = ros_y - r*sin(tmp_yaw);
        position_result.z = ros_z;
        position_result.yaw = tmp_yaw;
        results.push_back(position_result);
        }

        float yaw_diff_min = fabs(results.at(0).yaw - xiangdui_yaw);
        float temp_yaw_diff = fabs(results.at(1).yaw - xiangdui_yaw);
        if(temp_yaw_diff < yaw_diff_min)
        {
            yaw_diff_min = temp_yaw_diff;
            idx = 1;
        }
        else{
            idx = 0;
        }
    }
    else if(armor_num == 3)
    {
        for (i = 0; i<3; i++) {
        result position_result;
        float tmp_yaw = tar_yaw + i * 2.0 * M_PI/3.0;  // 2/3PI
        float r =  (r_1 + r_2)/2;   //理论上r1=r2 这里取个平均值                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                             
        position_result.x = ros_x - r*cos(tmp_yaw);
        position_result.y = ros_y - r*sin(tmp_yaw);
        position_result.z = ros_z;
        position_result.yaw = tmp_yaw;
        results.push_back(position_result);
        }
            // 2       1

            //     0
        float yaw_diff_min = cos(results.at(0).yaw - xiangdui_yaw);
        for(i = 1; i<3;i++)
        {
            float temp_yaw_diff = cos(results.at(i).yaw - xiangdui_yaw);
            if(temp_yaw_diff > yaw_diff_min)
            {
                yaw_diff_min = temp_yaw_diff;
                idx = i;
            }
        }
        //                            i        j

        // new idea  : to choose the       k      position to shoot
    }
    else
    {
        for (i = 0; i<4; i++) {
        result position_result;
        float tmp_yaw = tar_yaw + i * M_PI/2.0;
        float r = use_1 ? r_1 : r_2;
        position_result.x = ros_x - r*cos(tmp_yaw);
        position_result.y = ros_y - r*sin(tmp_yaw);
        position_result.z = use_1 ? ros_z : ros_z + dz;
        position_result.yaw = tmp_yaw;
        results.push_back(position_result);
        use_1 = !use_1;
        }

        float yaw_diff_min = cos(results.at(0).yaw - xiangdui_yaw);
        for(int i = 1; i<4; i++)
        {
            float temp_yaw_diff = cos(results.at(i).yaw - xiangdui_yaw);
            if(temp_yaw_diff > yaw_diff_min)
            {
                yaw_diff_min = temp_yaw_diff;
                idx = i;
            }
        }
    }
//得到results :存放了所有装甲板的位置信息 
//得到idx :选择的装甲板的编号
//----------------------------------------------
    float object_x = results.at(idx).x;
    float object_y = results.at(idx).y + y_bias;
    float object_z = results.at(idx).z + z_bias;
//----------------------------------------------
//---------------------------------------------
    // 用于三维可视化，与计算没有什么影响,后期可以删除
    visualization_msgs::msg::Marker aiming_point_;
    if (abs(object_x) > 0.01) {
        aiming_point_.header.stamp = this->now();
        aiming_point_.pose.position.x = object_x;
        aiming_point_.pose.position.y = object_y;
        aiming_point_.pose.position.z = object_z;
        maker_pub_->publish(aiming_point_);
    }
//---------------------------------------------
//根据需要选择模型
    /*if(no_resistance_model(object_x, object_y, object_z, v0) == 0)
    {
        return 0;
    }*/
    /*if(single_resistance_model(object_x, object_y, object_z, v0, randa)==0）
    {
        return 0;
    }*/
    /*if(single_resistance_model_two(object_x, object_y, object_z, v0, randa) == 0)
    {
        return 0;
    }*/
    if(two_resistance_model(left_angle_pitch,object_x, object_y, object_z, v0, randa) == 0)
    {
        return 0;
    }
    left_angle_yaw = atan2(object_y, object_x);
    this->distance = sqrtf(pow(object_x, 2) + pow(object_y, 2));
    pose.header.stamp = this->now();
    pose.header.frame_id = "leftodom";
    pose.point.x = object_x;
    pose.point.y = object_y;
    pose.point.z = object_z;

    if(is_right_tracking == true)
    {
        return 1;   
    }
    else
    {
        get_bigyaw(pose);
        return 1;
    }
}


//用于测试
void Trajectoryer::test()
{
    float object_x = 10;
    float object_y = 5;
    float object_z = 2;
    v0 = 16;
    now_pitch = 20 / 57.3f;
    now_yaw = 10 / 57.3f;
    no_resistance_model(left_angle_pitch,object_x, object_y, object_z, v0);
    single_resistance_model(left_angle_pitch,object_x, object_y, object_z, v0, randa);
    single_resistance_model_two(left_angle_pitch,object_x, object_y, object_z, v0, randa);
    armor_num = 4;
    yaw = 10 / 53.7f;
    v_yaw = 0.2;
    vx = 0.4;
    ros_x = 10;
    r_1 = 0.4;
    r_2 = 0.4;
    object_x = ros_x - r_1 *cos(now_yaw);
    vy = 0.3;
    ros_y = 1;
    vz = 0.2;
    ros_z = 2;
    object_z = ros_x - r_1*cos(now_pitch);
    dz = 0.1;
    solve_trajectory();
}

//回调函数，用于接收tracker发布的消息
//得到通过追踪到的装甲板相对于云台的信息
//如果追踪到目标并且开启了自瞄模式，则解算弹道，得到需要的pitch和yaw角度
//通过SendSerial信息类型的result发布给serial_driver
void Trajectoryer::left_camera_target_callback(const auto_aim_interfaces::msg::Target msg)
{
    /*pose.header.stamp = msg.header.stamp;
    pose.header.frame_id = "odom";

    armorpose.header.stamp = msg.header.stamp;
    armorpose.header.frame_id = "odom";

    needpose.header.stamp = msg.header.stamp;
    needpose.header.frame_id = "horizom_gimbal_link";*/

    is_left_tracking = msg.tracking;
    id = msg.id;
    armor_num = msg.armors_num;
    yaw = msg.yaw;
    v_yaw = msg.v_yaw;
    vx = msg.velocity.x;
    ros_x = msg.position.x;
    vy = msg.velocity.y;
    ros_y = msg.position.y;
    vz = msg.velocity.z;
    ros_z = msg.position.z;
    r_1 = msg.radius_1;
    r_2 = msg.radius_2;
    dz = msg.dz;
    if(is_left_tracking)
    {
        auto_aim_interfaces::msg::SendSerial result;
        if(solve_trajectory() == 0)
        {
            if(is_right_tracking == true)
            {
                result.header.frame_id = "rightodom";
                result.header.stamp = this->now();
                result.is_left_tracking = false;
                result.is_right_tracking = is_right_tracking;
                result.is_assist_tracking = false;
                result.is_left_can_hit = false;
                result.is_right_can_hit = is_right_can_hit;
                result.left_pitch = 0.0;
                result.left_yaw = 0.0;
                result.right_pitch = right_angle_pitch * 57.3f;
                result.right_yaw = right_angle_yaw * 57.3f;
                result.bigyaw = angle_bigyaw * 57.3f;
            }
            else
            {
                result.header.frame_id = "not";
                result.header.stamp = this->now();
                result.is_left_tracking = false;
                result.is_right_tracking = false;
                result.is_assist_tracking = false;
                result.is_left_can_hit = false;
                result.is_right_can_hit = false;
                result.left_pitch = 0.0;
                result.left_yaw = 0.0;
                result.right_pitch = 0.0;
                result.right_yaw = 0.0;
                result.bigyaw = 0.0;
            }
            result_pub_->publish(result);
            return;
        }
            float send_pitch = left_angle_pitch * 57.3f;
            float send_yaw = 0.0;
            float send_bigyaw = angle_bigyaw * 57.3f;
            if((needchangeyaw) < 5 && (needchangeyaw) > -5 && distance > 1.5)
            {
                send_yaw = (left_angle_yaw) * 57.3f + needchangeyaw;
                //send_yaw = (angle_yaw) * 57.3f;
            }
            else{
                send_yaw = (left_angle_yaw) * 57.3f;
            }
            if(distance > 6)
            {
                is_left_can_hit = false;
            }
            if(is_right_tracking == true)
            {
                result.header.frame_id = "allodom";
                result.header.stamp = this->now();
                result.is_left_tracking = is_left_tracking;
                result.is_right_tracking = is_right_tracking;
                result.is_assist_tracking = false;
                result.is_left_can_hit = is_left_can_hit;
                result.is_right_can_hit = is_right_can_hit;
                result.bigyaw = angle_bigyaw * 57.3f;
                result.left_pitch = -left_angle_pitch * 57.3f;
                result.left_yaw = left_angle_yaw * 57.3f;
                result.right_pitch = -right_angle_pitch * 57.3f;
                result.right_yaw = right_angle_yaw * 57.3f;
                result_pub_->publish(result);
            }
            else
            {
                result.header.frame_id = "leftodom";
                result.header.stamp = this->now();
                result.is_left_tracking = is_left_tracking;
                result.is_right_tracking = false;
                result.is_assist_tracking = false;
                result.is_left_can_hit = is_left_can_hit;
                result.is_right_can_hit = false;
                result.bigyaw = angle_bigyaw * 57.3f;
                result.left_pitch = -left_angle_pitch * 57.3f;
                result.left_yaw = left_angle_yaw * 57.3f;
                result.right_pitch = 0.0;
                result.right_yaw = 0.0;
                result_pub_->publish(result);
            }
            result_pub_->publish(result);
    }
    else
    {
        if(is_assist_tracking == false && is_right_tracking == true)
        {
            auto_aim_interfaces::msg::SendSerial result;
            result.header.frame_id = "rightodom";
            result.header.stamp = this->now();
            result.is_left_tracking = false;
            result.is_right_tracking = is_right_tracking;
            result.is_left_can_hit = false;
            result.is_right_can_hit = is_right_can_hit;
            result.bigyaw = angle_bigyaw * 57.3f;
            result.left_pitch = 0.0;
            result.left_yaw = 0.0;
            result.right_pitch = -right_angle_pitch * 57.3f;
            result.right_yaw = right_angle_yaw * 57.3f;
            result_pub_->publish(result);
        }
        else if(is_assist_tracking == false && is_left_tracking == false)
        {
            auto_aim_interfaces::msg::SendSerial result;
            result.header.frame_id = "notodom";
            result.header.stamp = this->now();
            result.is_left_tracking = false;
            result.is_right_tracking = false;
            result.is_left_can_hit = false;
            result.is_right_can_hit = false;
            result.bigyaw = 0.0;
            result.left_pitch = 0.0;
            result.left_yaw = 0.0;
            result.right_pitch = 0.0;
            result.right_yaw = 0.0;
            result_pub_->publish(result);
        }
    }
}

void Trajectoryer::right_result_callback(const auto_aim_interfaces::msg::Inter msg)
{
    is_right_tracking = msg.is_right_tracking;
    is_right_can_hit = msg.is_right_can_hit;
    if(is_left_tracking == false)
    {
        angle_bigyaw = msg.bigyaw;
    }
    right_angle_pitch = msg.right_pitch;
    right_angle_yaw = msg.right_yaw;
}

void Trajectoryer::front_assist_target_callback(const auto_aim_interfaces::msg::Target msg)
{
    if(is_left_tracking == true || is_right_tracking == true)
    {
        is_assist_tracking = false;
        return;
    }
    if(is_assist_tracking == true)
    {
        return;
    }
    is_assist_tracking = msg.tracking;
    assist_x = msg.position.x;
    assist_y = msg.position.y;
    assist_z = msg.position.z;

    geometry_msgs::msg::PointStamped pose;
    pose.header.stamp = this->now();
    pose.header.frame_id = "odom";
    pose.point.x = assist_x;
    pose.point.y = assist_y;
    pose.point.z = assist_z;
    assist_get_yaw_bigyaw(left_angle_pitch,left_angle_yaw,right_angle_pitch,right_angle_yaw,angle_bigyaw,pose);

    if(is_assist_tracking)
    {
        auto_aim_interfaces::msg::SendSerial result;
        result.header.frame_id = "bigodom";
        result.header.stamp = this->now();

            float left_send_pitch = left_angle_pitch * 57.3f;
            float left_send_yaw = left_angle_yaw * 57.3f;
            float right_send_pitch = right_angle_pitch * 57.3f;
            float right_send_yaw = right_angle_yaw * 57.3f;
            float send_bigyaw = angle_bigyaw * 57.3f;
            result.is_left_tracking = false;
            result.is_right_tracking = false;
            result.is_assist_tracking = true;
            result.is_left_can_hit = false;
            result.is_right_can_hit = false;
            result.bigyaw = send_bigyaw;
            result.left_pitch = -left_send_pitch;
            result.left_yaw = left_send_yaw;
            result.right_pitch = -right_send_pitch;
            result.right_yaw = right_send_yaw;
            result_pub_->publish(result);
    }

    is_assist_tracking = false;
}

void Trajectoryer::back_assist_target_callback(const auto_aim_interfaces::msg::Target msg)
{
    if(is_left_tracking == true || is_right_tracking == true)
    {
        is_assist_tracking = false;
        return;
    }
    if(is_assist_tracking == true)
    {
        return;
    }
    is_assist_tracking = msg.tracking;
    assist_x = msg.position.x;
    assist_y = msg.position.y;
    assist_z = msg.position.z;

    geometry_msgs::msg::PointStamped pose;
    pose.header.stamp = this->now();
    pose.header.frame_id = "odom";
    pose.point.x = assist_x;
    pose.point.y = assist_y;
    pose.point.z = assist_z;
    assist_get_yaw_bigyaw(left_angle_pitch,left_angle_yaw,right_angle_pitch,right_angle_yaw,angle_bigyaw,pose);

    if(is_assist_tracking)
    {
        auto_aim_interfaces::msg::SendSerial result;
        result.header.frame_id = "bigodom";
        result.header.stamp = this->now();

            float left_send_pitch = left_angle_pitch * 57.3f;
            float left_send_yaw = left_angle_yaw * 57.3f;
            float right_send_pitch = right_angle_pitch * 57.3f;
            float right_send_yaw = right_angle_yaw * 57.3f;
            float send_bigyaw = angle_bigyaw * 57.3f;
            result.is_left_tracking = false;
            result.is_right_tracking = false;
            result.is_assist_tracking = true;
            result.is_left_can_hit = false;
            result.is_right_can_hit = false;
            result.bigyaw = send_bigyaw;
            result.left_pitch = -left_send_pitch;
            result.left_yaw = left_send_yaw;
            result.right_pitch = -right_send_pitch;
            result.right_yaw = right_send_yaw;
            result_pub_->publish(result);
    }

    is_assist_tracking = false;
}

//接受串口发来的信息，得到当前云台的pitch和yaw角度,以及是否开启自瞄模式
void Trajectoryer::angle_callback(const auto_aim_interfaces::msg::ReceiveSerial msg)
{
    now_pitch = msg.left_pitch / 57.3f;
    now_yaw = msg.left_yaw / 57.3f;
    // RCLCPP_INFO(get_logger(), "now_pitch: %f", now_pitch);
    // RCLCPP_INFO(get_logger(), "now_yaw: %f", now_yaw);
    // 创建坐标变换消息和发布
    /*geometry_msgs::msg::TransformStamped t;
    t.header.stamp = this->now() + rclcpp::Duration::from_seconds(timestamp_offset_);
    t.header.frame_id = "gimbal_link";
    t.child_frame_id = "horizom_gimbal_link";
    tf2::Quaternion q;
    q.setRPY(0, -now_pitch, 0);
    t.transform.rotation = tf2::toMsg(q);
    tf_broadcaster_->sendTransform(t);

    // 创建坐标变换消息和发布
    geometry_msgs::msg::TransformStamped t_p;
    t_p.header.stamp = this->now() + rclcpp::Duration::from_seconds(timestamp_offset_);
    t_p.header.frame_id = "horizom_gimbal_link";
    t_p.child_frame_id = "prediction_gimbal_link";
    tf2::Quaternion q_p;
    q_p.setRPY(0, -angle_pitch, 0);
    t_p.transform.rotation = tf2::toMsg(q_p);
    tf_broadcaster_->sendTransform(t_p);*/
}

void Trajectoryer::changeyaw_callback(const auto_aim_interfaces::msg::Bias msg)
{
    needchangeyaw = msg.needchangeyaw;
    is_left_can_hit = msg.is_can_hit;
}

void Trajectoryer::get_need_pose(const geometry_msgs::msg::PointStamped pose)
{
    // 创建坐标变换消息和发布
    geometry_msgs::msg::TransformStamped t;
    t.header.stamp = this->now();
    t.header.frame_id = "left_gimbal_link";
    t.child_frame_id = "horizom_gimbal_link";
    tf2::Quaternion q;
    q.setRPY(0, -now_pitch, 0);
    t.transform.rotation = tf2::toMsg(q);
    tf_broadcaster_->sendTransform(t);

    // 创建坐标变换消息和发布
    geometry_msgs::msg::TransformStamped t_p;
    t_p.header.stamp = this->now();
    t_p.header.frame_id = "horizom_gimbal_link";
    t_p.child_frame_id = "prediction_gimbal_link";
    tf2::Quaternion q_p;
    q_p.setRPY(0, -left_angle_pitch, 0);
    t_p.transform.rotation = tf2::toMsg(q_p);
    tf_broadcaster_->sendTransform(t_p);


    get_needpose(pose);
}

int main(int argc, char *argv[])
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<Trajectoryer>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}

//----------------------------------------------------------------------------
//tf2
void Trajectoryer::tf2_init()
{
    // Subscriber with tf2 message_filter
  // tf2 relevant
  tf2_buffer_ = std::make_shared<tf2_ros::Buffer>(this->get_clock());
  // Create the timer interface before call to waitForTransform,
  // to avoid a tf2_ros::CreateTimerInterfaceException exception
  auto timer_interface = std::make_shared<tf2_ros::CreateTimerROS>(
    this->get_node_base_interface(), this->get_node_timers_interface());
  tf2_buffer_->setCreateTimerInterface(timer_interface);
  tf2_listener_ = std::make_shared<tf2_ros::TransformListener>(*tf2_buffer_);
  needpose_target_frame_ = "prediction_camera_optical_frame";
  armorpose_target_frame_ = "prediction_camera_optical_frame";

   timestamp_offset_ = 0.00;
}

void Trajectoryer::get_needpose(const geometry_msgs::msg::PointStamped needpose)
{
  if(tf2_buffer_->canTransform("horizom_gimbal_link", "prediction_camera_optical_frame", tf2::TimePointZero)){
      //std::cout << "Frames can be transformed" << std::endl;
  }else{
      std::cout << "needposeFrames cannot be transformed" << std::endl;
      return;
  }

  geometry_msgs::msg::PointStamped ps;
  ps.header = needpose.header;
  ps.point = needpose.point;
  try {
    ps.point = tf2_buffer_->transform(ps, needpose_target_frame_).point;
  } catch (const tf2::ExtrapolationException & ex) {
    RCLCPP_ERROR(get_logger(), "needposeError while transforming %s", ex.what());
    return;
  }
  needpose_pub_->publish(ps);
}

void Trajectoryer::get_armorpose(const geometry_msgs::msg::PointStamped armorpose)
{
    if(tf2_buffer_->canTransform("leftodom", "left_camera_optical_frame", tf2::TimePointZero)){
      //std::cout << "Frames can be transformed" << std::endl;
    }else{
      std::cout << "armorposeFrames cannot be transformed" << std::endl;
      return;
    }

    geometry_msgs::msg::PointStamped armorps;
    armorps.header = armorpose.header;
    armorps.point = armorpose.point;
    try {
        armorps.point = tf2_buffer_->transform(armorps, "left_camera_optical_frame").point;
    } catch (const tf2::ExtrapolationException & ex) {
        RCLCPP_ERROR(get_logger(), "armorposeError while transforming %s", ex.what());
        return;
    }
    armorpose_pub_->publish(armorps);
}

void Trajectoryer::get_bigyaw(const geometry_msgs::msg::PointStamped smallpose)
{
    if(tf2_buffer_->canTransform("leftodom", "bigodom", tf2::TimePointZero)){
      //std::cout << "Frames can be transformed" << std::endl;
    }else{
      std::cout << "bigyawFrames cannot be transformed" << std::endl;
      return;
    }

    geometry_msgs::msg::PointStamped bigps;
    bigps.header = smallpose.header;
    bigps.point = smallpose.point;
    try {
        bigps.point = tf2_buffer_->transform(bigps, "bigodom").point;
    } catch (const tf2::ExtrapolationException & ex) {
        RCLCPP_ERROR(get_logger(), "Error while transforming %s", ex.what());
        return;
    }
    
    angle_bigyaw = atan2(bigps.point.y, bigps.point.x);
}

void Trajectoryer::assist_get_yaw_bigyaw(float &left_angle_pitch, float &left_angle_yaw,float &right_angle_pitch,float &right_angle_yaw,float &angle_bigyaw,const geometry_msgs::msg::PointStamped &pose)
{
    if(tf2_buffer_->canTransform("bigodom", "leftodom", tf2::TimePointZero)){
      //std::cout << "Frames can be transformed" << std::endl;
    }else{
      std::cout << "leftyawFrames cannot be transformed" << std::endl;
      return;
    }
    if(tf2_buffer_->canTransform("bigodom", "rightodom", tf2::TimePointZero)){
      //std::cout << "Frames can be transformed" << std::endl;
    }else{
      std::cout << "rightyawFrames cannot be transformed" << std::endl;
      return;
    }

    geometry_msgs::msg::PointStamped leftps;
    leftps.header.stamp = pose.header.stamp;
    leftps.header.frame_id = "bigodom";
    leftps.point = pose.point;
    try {
        leftps.point = tf2_buffer_->transform(leftps, "leftodom").point;
    } catch (const tf2::ExtrapolationException & ex) {
        RCLCPP_ERROR(get_logger(), "Error while transforming %s", ex.what());
        return;
    }
    
    if(two_resistance_model(left_angle_pitch,leftps.point.x, leftps.point.y, leftps.point.z, v0, randa) == 0)
    {
        is_assist_tracking = false;
    }
    left_angle_yaw = atan2(leftps.point.y, leftps.point.x);

    geometry_msgs::msg::PointStamped rightps;
    rightps.header.stamp = pose.header.stamp;
    rightps.header.frame_id = "bigodom";
    rightps.point = pose.point;
    try {
        rightps.point = tf2_buffer_->transform(rightps, "rightodom").point;
    } catch (const tf2::ExtrapolationException & ex) {
        RCLCPP_ERROR(get_logger(), "Error while transforming %s", ex.what());
        return;
    }

    if(two_resistance_model(right_angle_pitch,rightps.point.x, rightps.point.y, rightps.point.z, v0, randa) == 0)
    {
        is_assist_tracking = false;
    }
    right_angle_yaw = atan2(rightps.point.y, rightps.point.x);

    angle_bigyaw = atan2(pose.point.y, pose.point.x);
}
//---------------------------------------------------------------------------