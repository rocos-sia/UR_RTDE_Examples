#include <ur_rtde/rtde_control_interface.h>
#include <ur_rtde/rtde_io_interface.h>
#include <ur_rtde/rtde_receive_interface.h>
#include <thread>
#include <chrono>
#include <Eigen/Dense>

void FK(std::vector<double> a)
{
 std::cout<<"this is FK";   
}


void IK(double a)
{
    std::cout<<"this is IK";
}

int main(int argc, char* argv[])
{
//连接UR
ur_rtde::RTDEIOInterface rtde_io("192.168.3.101");
ur_rtde::RTDEControlInterface rtde_control("192.168.3.101");
ur_rtde::RTDEReceiveInterface rtde_receive("192.168.3.101");

//在z轴方向上等效为质量弹簧阻尼系统；
//变量初始化

//机器人读取变量
std::vector<double> TCPpose;//当前末端位姿向量
std::vector<double> TCPForce; //当前六维力向量
std::vector<double> actual_q;//当前关节变量
std::vector<double> qd;
Eigen::MatrixXd J;//雅可比
double x;//当前位置
double x_ref;//参考位置
double x_u;//位置控制输出

double q_u;//关节控制输出

//速度变量
double xd_ref;//参考速度

double xd{0.0};//期望速度
double xedd;//加速度

double xed_t;//当前时刻的速度误差
double xed_tt;//xedd积分一次后的速度误差

double xe_t;//当前位置误差
double xe_tt;//下一时刻的位置误差

//质量弹簧阻尼参数设置
double M{0.0};
double B{0.0};
double K{0.0};
double deltaT{0.001};
double fe;
fe=TCPForce[2];

while(true)
{
//计算位置误差
xe_t=x-x_ref;
//计算速度误差
xed_t=xd-xd_ref;
//计算加速度误差
xedd=fe-B*xed_t-K*xe_t;
//通过两次积分计算下一时刻的位置误差
xed_tt=xe_t+xedd*deltaT;
xe_tt=xe_t+xed_tt*deltaT;
//得到控制输出x_u
x_u=x_ref+xe_tt;

}
return 0;
}


