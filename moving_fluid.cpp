
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

void sendThreadRun(boost::asio::serial_port *serialPort)
{
  unsigned char sendBuffer[] = {0x55, 0XAA, 0x04, 0x01, 0x20, 0x37, 0xE8, 0x00, 0X49};
  int sum = 0;

  while (true)
  {
    for (size_t i = 2; i < sizeof(sendBuffer) - 1; i++)
    {
      sum = sum + sendBuffer[i];
    }
    sendBuffer[sizeof(sendBuffer) - 1] = sum;
    sum = 0;

    std::this_thread::sleep_for(std::chrono::milliseconds(1000));

    serialPort->write_some(boost::asio::buffer(sendBuffer, sizeof(sendBuffer)));

    // for (int i = 0; i < sizeof(sendBuffer); i++)
    // {
    //     printf("sendBuffer[%d]=%x\n", i, sendBuffer[i]);
    // }
  }
}

void receiveThreadRun(boost::asio::serial_port *serialPort)
{
  unsigned char buff[1024];
  int readCount = 0;

  while (true)
  {
    memset(buff, 0, sizeof(buff));

    readCount = serialPort->read_some(boost::asio::buffer(buff, sizeof(buff)));
    if (readCount)
    {
      for (int i = 0; i < readCount; i++)
      {
        printf("buff[%d]=%x\n", i, buff[i]);
      }
      std::cout << "---------------------------" << std::endl;
      readCount = 0;
    }
    sleep(2);
  }
}

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

  std::thread sendThread(sendThreadRun, &serialPort);
  std::thread receiveThread(receiveThreadRun, &serialPort);

  //**-------------------------------**//

  //** UR控制从这里开始 **//

  RTDEControlInterface rtde_control("127.0.0.1");
  RTDEReceiveInterface rtde_receive("127.0.0.1");
  std::vector<double> init_q = rtde_receive.getActualQ();

  // Target in the robot base
  std::vector<double> new_q = init_q;
  new_q[0] += 0.3;

  rtde_control.moveJ(new_q, 1.05, 1.4, false);

  rtde_control.moveJ(init_q, 1.05, 1.4, false);

  rtde_control.stopScript();
  //**-------------------------------**//

  sendThread.join();
  receiveThread.join();

  return 0;
}