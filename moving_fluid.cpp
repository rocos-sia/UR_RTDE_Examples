
#include <stdio.h>
#include <ur_rtde/rtde_control_interface.h>
#include <ur_rtde/rtde_receive_interface.h>

#include <boost/asio.hpp>
#include <boost/bind.hpp>
#include <chrono>
#include <iostream>
#include <thread>

using namespace ur_rtde;
using namespace std::chrono;

int main(int argc, char **argv)
{
  //** 串口初始化 **//

  boost::asio::io_service ioService;
  boost::asio::serial_port serialPort(ioService, "/dev/ttyUSB0");

  serialPort.set_option(boost::asio::serial_port::baud_rate(115200));
  serialPort.set_option(boost::asio::serial_port::flow_control(boost::asio::serial_port::flow_control::none));
  serialPort.set_option(boost::asio::serial_port::parity(boost::asio::serial_port::parity::none));
  serialPort.set_option(boost::asio::serial_port::stop_bits(boost::asio::serial_port::stop_bits::one));
  serialPort.set_option(boost::asio::serial_port::character_size(8));

  //**-------------------------------**//

  //** UR控制从这里开始 **//

  RTDEControlInterface rtde_control("192.168.3.101");
  RTDEReceiveInterface rtde_receive("192.168.3.101");
  std::vector<double> init_q = rtde_receive.getActualQ();

  // Target in the robot base
  std::vector<double> new_q = init_q;
  new_q[0] += 0.3;

  unsigned char sendBuffer[] = {0x55, 0XAA, 0x04, 0x02, 0x20, 0x37, 0xE8, 0x00, 0X49};
  //给驱动器发指令
  serialPort->write_some(boost::asio::buffer(sendBuffer, sizeof(sendBuffer)));

  rtde_control.moveJ(new_q, 1.05, 1.4, false);

  rtde_control.moveJ(init_q, 1.05, 1.4, false);

  rtde_control.stopScript();
  //**-------------------------------**//

  return 0;
}