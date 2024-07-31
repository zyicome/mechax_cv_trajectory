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

    target_sub_ = this->create_subscription<auto_aim_interfaces::msg::Target>(
                  "/tracker/target", rclcpp::SensorDataQoS(), std::bind(&Trajectoryer::target_callback, this, std::placeholders::_1));

    angle_sub_ = this->create_subscription<auto_aim_interfaces::msg::ReceiveSerial>(
        "/angle/init", 10, std::bind(&Trajectoryer::angle_callback, this, std::placeholders::_1));

    power_rune_sub_ = this->create_subscription<geometry_msgs::msg::PointStamped>(
        "/prediction", 10, std::bind(&Trajectoryer::power_rune_callback, this, std::placeholders::_1));

    maker_pub_ = this->create_publisher<visualization_msgs::msg::Marker>(
        "/aiming_point", 10);

    result_pub_ = this->create_publisher<auto_aim_interfaces::msg::SendSerial>(
        "/trajectory/result", 10);

    bias_time_pub_ = this->create_publisher<auto_aim_interfaces::msg::Bias>(
        "/bias/time", 10);
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
        v0 = 23; // m/s
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
    fly_t = 0.5; // s
    //摄像头相对于云台的偏置,一般改z_bias即可
    y_bias = 0.0; // m
    z_bias = 0.00; // m
    //****************************************************
    is_can_hit = false;
    distance = 0.0;
    //****************************************************
    motor_speed = 10 / 57.3f; //rad/s
    motor_bias_time = 0.0;
    serial_bias_time = 0.002;
    latency_bias_time = 0.003;
    }

//@param: object_x, object_y, object_z, v0
// 无空气阻力模型，最简单的模型，只考虑重力加速度，用来判断是否能够击打到目标
//@result: angle_pitch, fly_t，不能直接使用，用来作为初始化数据, 需要用空气阻力模型进行修正
//@return: 1:可以击打 0:无法击打到目标
int Trajectoryer::no_resistance_model(const float &object_x,const float &object_y,const float &object_z,const float &v0)
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
    /*std::cout << "noangle_pitch: " << angle_pitch << std::endl; 
    std::cout << "nofly_t: " << fly_t << std::endl;
    std::cout << "nol: " << l << std::endl;
    std::cout << "nop: " << process_angle << std::endl;
    std::cout << "noa: " << alpha << std::endl;*/
    return 1;
}

//@param: object_x, object_y, object_z, v0, randa
// 根据传入的相对于枪管坐标系下敌方的坐标xyz，在结合子弹速度和空气阻力系数，计算出需要的pitch角度和飞行时间
// 单空气阻力模型，运用牛顿迭代法，迭代10次，精度为0.001
//@result: angle_pitch, fly_t (成员变量，刷新得到)
//@return: 1:计算成功 0:无法迭代到结果精度或者迭代次数过多
int Trajectoryer::single_resistance_model(const float &object_x,const float &object_y,const float &object_z,const float &v0,const float &randa)
{
    float distance = sqrtf(pow(object_x, 2) + pow(object_y, 2));
    if(!no_resistance_model(object_x, object_y, object_z, v0))
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
//@return: 1:计算成功 0:计算失败d
int Trajectoryer::single_resistance_model_two(const float &object_x,const float &object_y,const float &object_z,const float &v0,const float &randa)
{
    float distance = sqrtf(pow(object_x, 2) + pow(object_y, 2));
    if(!no_resistance_model(object_x, object_y, object_z, v0))
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
int Trajectoryer::two_resistance_model(const float &object_x,const float &object_y,const float &object_z,const float &v0,const float &randa)
{
    float distance = sqrtf(pow(object_x, 2) + pow(object_y, 2));
    if(!no_resistance_model(object_x, object_y, object_z, v0))
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
    float need_t = fly_t + latency_bias_time + motor_bias_time + serial_bias_time;
    if(std::isnan(need_t))
    {
        need_t = 0.27;
    }
    if(std::isnan(fly_t))
    {
        fly_t = 0.14;
    }
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

        float yaw_diff_min = fabs(results.at(0).yaw - now_yaw);
        float temp_yaw_diff = fabs(results.at(1).yaw - now_yaw);
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
        float yaw_diff_min = cos(results.at(0).yaw - now_yaw);
        for(i = 1; i<3;i++)
        {
            float temp_yaw_diff = cos(results.at(i).yaw - now_yaw);
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

        float yaw_diff_min = cos(results.at(0).yaw - now_yaw);
        for(int i = 1; i<4; i++)
        {
            float temp_yaw_diff = cos(results.at(i).yaw - now_yaw);
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
        aiming_point_.type = visualization_msgs::msg::Marker::SPHERE;
        aiming_point_.action = visualization_msgs::msg::Marker::ADD;
        aiming_point_.pose.position.x = object_x;
        aiming_point_.pose.position.y = object_y;
        aiming_point_.pose.position.z = object_z;
        aiming_point_.scale.x = 1;
        aiming_point_.scale.y = 0.1;
        aiming_point_.scale.z = 0.1;
        aiming_point_.color.a = 1.0;
        aiming_point_.color.r = 0.0;
        aiming_point_.color.g = 1.0;
        aiming_point_.color.b = 0.0;
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
    if(two_resistance_model(object_x, object_y, object_z, v0, randa) == 0)
    {
        return 0;
    }
    angle_yaw = atan2(object_y, object_x);
    if(abs(angle_yaw * 57.3f - now_yaw * 57.3f) <= 2 && motor_speed != 0 && abs(angle_yaw - now_yaw) / abs(motor_speed) < 0.3)
    {
        motor_bias_time = abs(angle_yaw - now_yaw) / abs(motor_speed);
    }
    else
    {
        motor_bias_time = 0.01;
    }
    bias_time_msg.header.stamp = this->now();
    bias_time_msg.need_t = need_t;
    bias_time_msg.fly_t = fly_t;
    bias_time_msg.serial_bias_time = serial_bias_time;
    bias_time_msg.latency_bias_time = latency_bias_time;
    bias_time_msg.motor_bias_time =motor_bias_time;
    bias_time_pub_->publish(bias_time_msg);
    this->distance = sqrtf(pow(object_x, 2) + pow(object_y, 2) + pow(object_z, 2));
    return 1;
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
    no_resistance_model(object_x, object_y, object_z, v0);
    single_resistance_model(object_x, object_y, object_z, v0, randa);
    single_resistance_model_two(object_x, object_y, object_z, v0, randa);
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
void Trajectoryer::target_callback(const auto_aim_interfaces::msg::Target msg)
{
    is_tracking = msg.tracking;
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

    if (is_rune) return;

    if(is_tracking)
    {
        auto_aim_interfaces::msg::SendSerial result;
        result.header.frame_id = "not";
        result.header.stamp = msg.header.stamp;
        if(solve_trajectory() == 0)
        {
            result.is_tracking = false;
            result.is_can_hit = false;
            result.pitch = 0.0;
            result.yaw = 0.0;
            result.distance = 0.0;
            result_pub_->publish(result);
            return;
        }
            result.header.frame_id = "odom";
            //--------------------------------------------
            //弧度制转角度制
            float send_pitch = -angle_pitch * 57.3f;
            float send_yaw = 0.0;
            send_yaw = (angle_yaw) * 57.3f;
            //float send_yaw = (angle_yaw) * 57.3f;
            //--------------------------------------------
            float max_yaw_diff = 0.5; //现在的yaw与计算的需求yaw的最大容忍差值，可根据需求更改
            if(abs(send_yaw - now_yaw * 57.3f) > max_yaw_diff)
            {
                is_can_hit = false;
                std::cout << "Can not hit target!!!" << std::endl;
            }
            else
            {
                is_can_hit = true;
            }
            //--------------------------------------------
            result.is_tracking = is_tracking;
            result.is_can_hit = is_can_hit;
            if(std::isnan(send_pitch) || std::isnan(send_yaw) || std::isnan(distance))
            {
                result.is_tracking = false;
                result.is_can_hit = false;
                result.pitch = 0.0;
                result.yaw = 0.0;
                result.distance = 0.0;
            }
            else
            {
                result.pitch = send_pitch;
                result.yaw = send_yaw;
                result.distance = distance;
            }
            result_pub_->publish(result);

            latency_count++;
            all_latency = all_latency + (this->now() - msg.header.stamp).seconds();
            if(latency_count >= 20)
            {
                std::cout << "all_latency: " << all_latency << "s" << " and average latency: " << all_latency / latency_count << "s" << std::endl;
                latency_bias_time = all_latency / latency_count;
                latency_count = 0;
                all_latency = 0;
            }
    }
    else
    {
        auto_aim_interfaces::msg::SendSerial result;
        result.header.frame_id = "not";
        result.header.stamp = msg.header.stamp;
        result.is_tracking = is_tracking;
        result.is_can_hit = false;
        result.pitch = 0.0;
        result.yaw = 0.0;
        result.distance = 0.0;
        result_pub_->publish(result);
    }
}


//接受串口发来的信息，得到当前云台的pitch和yaw角度,以及是否开启自瞄模式
void Trajectoryer::angle_callback(const auto_aim_interfaces::msg::ReceiveSerial msg)
{
    now_pitch = msg.pitch / 57.3f;
    now_yaw = msg.yaw;
    if(now_yaw > 180)
    {
        now_yaw = now_yaw - ((int)now_yaw / 360) * 360;
        if(now_yaw > 180)
        {
            now_yaw = now_yaw - 360;
        }
    }
    else if(now_yaw < -180)
    {
        now_yaw = now_yaw - ((int)now_yaw / 360) * 360;
        if(now_yaw < -180)
        {
            now_yaw = now_yaw + 360;
        }
    }
    //std::cout << "now_yaw: " << now_yaw <<std::endl;
    now_yaw = now_yaw / 57.3f;
    if(msg.motor_speed != 0)
    {
        motor_speed = msg.motor_speed;
    }
    else
    {
        motor_speed = 55 / 57.3f;
    }
    if(msg.serial_time != 0.0)
    {
        serial_bias_time = msg.serial_time;
    }
    //std::cout << "motor_speed: " << motor_speed << std::endl;
    if(msg.v0 > 20)
    {
        v0 = msg.v0;
    }

    is_rune = msg.is_rune;
}

void Trajectoryer::power_rune_callback(const geometry_msgs::msg::PointStamped msg) {
    if (!is_rune) return;
    two_resistance_model(
            msg.point.x,
            msg.point.y,
            msg.point.z,
            v0, randa
    );
    angle_yaw = atan2(msg.point.y,msg.point.x);

    auto_aim_interfaces::msg::SendSerial result;
    result.header.frame_id = "odom";
    //--------------------------------------------
    //弧度制转角度制
    float send_pitch = -angle_pitch * 57.3f;
    float send_yaw = 0.0;
    send_yaw = (angle_yaw) * 57.3f;
    //float send_yaw = (angle_yaw) * 57.3f;
    //--------------------------------------------
    float max_yaw_diff = 0.5; //现在的yaw与计算的需求yaw的最大容忍差值，可根据需求更改
    if(abs(send_yaw - now_yaw * 57.3f) > max_yaw_diff)
    {
        is_can_hit = false;
        std::cout << "Can not hit target!!!" << std::endl;
    }
    else
    {
        is_can_hit = true;
    }
    //--------------------------------------------
    result.is_tracking = true;
    result.is_can_hit = is_can_hit;
    if(std::isnan(send_pitch) || std::isnan(send_yaw) || std::isnan(distance))
    {
        result.is_tracking = false;
        result.is_can_hit = false;
        result.pitch = 0.0;
        result.yaw = 0.0;
        result.distance = 0.0;
    }
    else
    {
        result.pitch = send_pitch;
        result.yaw = send_yaw;
        result.distance = distance;
    }
    result_pub_->publish(result);
}
int main(int argc, char *argv[])
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<Trajectoryer>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}
