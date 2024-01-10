#include "chassis.h"
#include "robot_def.h"
#include "dji_motor.h"
#include "message_center.h"

#include "general_def.h"
#include "bsp_dwt.h"
#include "arm_math.h"

/* 底盘应用包含的模块和信息存储,底盘是单例模式,因此不需要为底盘建立单独的结构体 */

static Subscriber_t *chassis_sub;                                    // 用于订阅底盘的控制命令
static Chassis_Ctrl_Cmd_s chassis_cmd_recv;                          // 底盘接收到的控制命令
static DJIMotor_Instance *motor_lf, *motor_rf, *motor_lb, *motor_rb; // 4个电机实例

static float chassis_vx, chassis_vy;     // 底盘的x,y方向速度
static float vt_lf, vt_rf, vt_lb, vt_rb; // 底盘速度解算后的临时输出,待进行限幅

#ifdef CHASSIS_MCNAMEE_WHEEL
#define CHASSIS_WHEEL_OFFSET 1.0f // 机器人底盘轮子修正偏移量
#elif defined(CHASSIS_OMNI_WHEEL)
#define CHASSIS_WHEEL_OFFSET 0.7071f // 机器人底盘轮子修正偏移量，根号2/2，即45度，用于修正全向轮的安装位置
#endif                               // CHASSIS_OMNI_WHEEL

static void MecanumCalculate(void);
static void LimitChassisOutput(void);

void ChassisInit(void)
{
    // 四个轮子的参数一样,改tx_id和反转标志位即可
    Motor_Init_Config_s chassis_motor_config = {
        .can_init_config.can_handle   = &hcan1,
        .controller_param_init_config = {
            .speed_PID = {
                .Kp            = 10,
                .Ki            = 0,
                .Kd            = 0,
                .IntegralLimit = 3000,
                .Improve       = PID_Trapezoid_Intergral | PID_Integral_Limit | PID_Derivative_On_Measurement,
                .MaxOut        = 12000,
            },
            .current_PID = {
                .Kp            = 0.5f,
                .Ki            = 0,
                .Kd            = 0,
                .IntegralLimit = 3000,
                .Improve       = PID_Trapezoid_Intergral | PID_Integral_Limit | PID_Derivative_On_Measurement,
                .MaxOut        = 15000,
            },
        },
        .controller_setting_init_config = {
            .angle_feedback_source = MOTOR_FEED,
            .speed_feedback_source = MOTOR_FEED,
            .outer_loop_type       = SPEED_LOOP,
            .close_loop_type       = CURRENT_LOOP,
        },
        .motor_type = M3508,
    };

    // 修改4个电机的CAN发送id,初始化4个电机
    chassis_motor_config.can_init_config.tx_id = 1;
    motor_lf                                   = DJIMotorInit(&chassis_motor_config);

    chassis_motor_config.can_init_config.tx_id = 2;
    motor_rf                                   = DJIMotorInit(&chassis_motor_config);

    chassis_motor_config.can_init_config.tx_id = 3;
    motor_lb                                   = DJIMotorInit(&chassis_motor_config);

    chassis_motor_config.can_init_config.tx_id = 4;
    motor_rb                                   = DJIMotorInit(&chassis_motor_config);

    // 单板或接收器在底盘控制整车,则通过pubsub来传递消息
    chassis_sub = SubRegister("chassis_cmd", sizeof(Chassis_Ctrl_Cmd_s));
}

/* 机器人底盘控制核心任务 */
void ChassisTask(void)
{
    // 读取底盘控制命令
    SubGetMessage(chassis_sub, &chassis_cmd_recv);

    if (chassis_cmd_recv.chassis_mode == CHASSIS_ZERO_FORCE) { // 如果出现重要模块离线或遥控器设置为急停,让电机停止
        DJIMotorStop(motor_lf);
        DJIMotorStop(motor_rf);
        DJIMotorStop(motor_lb);
        DJIMotorStop(motor_rb);
    } else { // 正常工作
        DJIMotorEnable(motor_lf);
        DJIMotorEnable(motor_rf);
        DJIMotorEnable(motor_lb);
        DJIMotorEnable(motor_rb);
    }

    // 根据控制模式设定旋转速度
    switch (chassis_cmd_recv.chassis_mode) {
        case CHASSIS_NO_FOLLOW: // 底盘不旋转,但维持全向机动,一般用于调整云台姿态
            chassis_cmd_recv.wz = 0;
            break;
        case CHASSIS_FOLLOW_GIMBAL_YAW: // 跟随云台,不单独设置pid,以误差角度平方为速度输出
            chassis_cmd_recv.wz = -1.5f * chassis_cmd_recv.offset_angle * abs(chassis_cmd_recv.offset_angle);
            break;
        case CHASSIS_ROTATE: // 自旋,同时保持全向机动;当前wz维持定值,后续增加不规则的变速策略
            chassis_cmd_recv.wz = 4000;
            break;
        default:
            break;
    }

    // 根据云台和底盘的角度offset将控制量映射到底盘坐标系上
    static float sin_theta, cos_theta;
    cos_theta  = arm_cos_f32(chassis_cmd_recv.offset_angle * DEGREE_2_RAD);
    sin_theta  = arm_sin_f32(chassis_cmd_recv.offset_angle * DEGREE_2_RAD);
    chassis_vx = chassis_cmd_recv.vx * cos_theta - chassis_cmd_recv.vy * sin_theta;
    chassis_vy = chassis_cmd_recv.vx * sin_theta + chassis_cmd_recv.vy * cos_theta;

    // 计算每个轮毂电机的输出,正运动学解算
    MecanumCalculate();

    // 根据裁判系统和电容剩余容量对输出进行限制并设置电机参考值
    LimitChassisOutput();
}

/**
 * @brief 计算每个轮毂电机的输出,正运动学解算
 *        用宏进行预替换减小开销,运动解算具体过程参考教程
 * @todo 速度解算的过程中,因为轮子实际安装的位置以及方向不同，
 *       仅仅加入偏移量是不够的，还需要加入方向的修正，待添加
 */
static void MecanumCalculate(void)
{
    vt_lf = -chassis_vx * CHASSIS_WHEEL_OFFSET - chassis_vy * CHASSIS_WHEEL_OFFSET - chassis_cmd_recv.wz;
    vt_rf = -chassis_vx * CHASSIS_WHEEL_OFFSET + chassis_vy * CHASSIS_WHEEL_OFFSET - chassis_cmd_recv.wz;
    vt_lb = chassis_vx * CHASSIS_WHEEL_OFFSET - chassis_vy * CHASSIS_WHEEL_OFFSET - chassis_cmd_recv.wz;
    vt_rb = chassis_vx * CHASSIS_WHEEL_OFFSET + chassis_vy * CHASSIS_WHEEL_OFFSET - chassis_cmd_recv.wz;
}

/**
 * @brief 根据裁判系统和电容剩余容量对输出进行限制并设置电机参考值
 *
 */
static void LimitChassisOutput(void)
{
    // 功率限制待添加
    // referee_data->PowerHeatData.chassis_power;
    // referee_data->PowerHeatData.chassis_power_buffer;

    // 完成功率限制后进行电机参考输入设定
    DJIMotorSetRef(motor_lf, vt_lf);
    DJIMotorSetRef(motor_rf, vt_rf);
    DJIMotorSetRef(motor_lb, vt_lb);
    DJIMotorSetRef(motor_rb, vt_rb);
}