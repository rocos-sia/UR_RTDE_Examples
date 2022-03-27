
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

int main( int argc, char** argv )
{
    //** 串口初始化 **//

    boost::asio::io_service ioService;
    boost::asio::serial_port serialPort( ioService, "/dev/ttyUSB0" );

    serialPort.set_option( boost::asio::serial_port::baud_rate( 115200 ) );
    serialPort.set_option( boost::asio::serial_port::flow_control( boost::asio::serial_port::flow_control::none ) );
    serialPort.set_option( boost::asio::serial_port::parity( boost::asio::serial_port::parity::none ) );
    serialPort.set_option( boost::asio::serial_port::stop_bits( boost::asio::serial_port::stop_bits::one ) );
    serialPort.set_option( boost::asio::serial_port::character_size( 8 ) );

    //**-------------------------------**//

    //** UR控制从这里开始 **//
    bool simulation = true;//代表是是否仿真，选择是表示不控制真实UR，选择否表示需要控制真实UR
    std::shared_ptr< RTDEControlInterface > rtde_control_ptr;
    std::shared_ptr< RTDEReceiveInterface > rtde_receive_ptr;

    if ( !simulation )
    {
        rtde_control_ptr.reset( new RTDEControlInterface{ "192.168.3.101", RTDEControlInterface::FLAG_USE_EXT_UR_CAP } );
        rtde_receive_ptr.reset( new RTDEReceiveInterface{ "192.168.3.101" } );
        std::vector< double > init_q = rtde_receive_ptr->getActualQ( );
        std::vector< double > new_q  = init_q;
        new_q[ 0 ] += 0.3;
        rtde_control_ptr->moveJ( new_q, 1.05, 1.4, false );
        rtde_control_ptr->moveJ( init_q, 1.05, 1.4, false );
    }


    unsigned char sendBuffer[] = { 0x55, 0XAA, 0x04, 0x01, 0x03, 0x37, 0x14, 0x05, 0X76 };
    //给驱动器发指令
    // serialPort这里不是指针，用.而不用->
    serialPort.write_some( boost::asio::buffer( sendBuffer, sizeof( sendBuffer ) ) );

    //   sendBuffer[] = {0x55, 0XAA, 0x04, 0x01, 0x03, 0x37, 0x14, 0x05, 0X58};
    //   serialPort.write_some(boost::asio::buffer(sendBuffer, sizeof(sendBuffer)));

    //  sendBuffer[] = {0x55, 0XAA, 0x04, 0x02, 0x03, 0x37, 0x14, 0x05, 0X58};
    //   serialPort.write_some(boost::asio::buffer(sendBuffer, sizeof(sendBuffer)));

    if ( !simulation )
    {
        rtde_control_ptr->stopScript( );
    }
    //**-------------------------------**//

    return 0;
}