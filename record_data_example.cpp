#include <ur_rtde/rtde_receive_interface.h>
#include <boost/program_options.hpp>
#include <thread>
#include <chrono>
#include <csignal>
#include <string>
#include <iostream>

using namespace ur_rtde;
using namespace std::chrono;
namespace po = boost::program_options;

// Interrupt flag
bool flag_loop = true;
void raiseFlag(int param)
{
  flag_loop = false;
}

int main(int argc, char* argv[])
{
  try {
    po::options_description desc("Allowed options");
    desc.add_options()
        ("help", "Record robot data to a (.csv) file")
        ("robot_ip", po::value<std::string>()->default_value("localhost"),
             "the IP address of the robot")
        ("frequency", po::value<double>()->default_value(500.0),
                 "the frequency at which the data is recorded (default is 500Hz)")
        ("output", po::value<std::string>()->default_value("robot_data.csv"),
                     "data output (.csv) file to write to (default is \"robot_data.csv\"")
        ;

    po::variables_map vm;
    po::store(po::parse_command_line(argc, argv, desc), vm);
    po::notify(vm);

    if (vm.count("help")) {
      std::cout << desc << "\n";
      return 0;
    }

    signal(SIGINT, raiseFlag);
    double frequency = vm["frequency"].as<double>();
    double dt = 1.0 / frequency;
    RTDEReceiveInterface rtde_receive(vm["robot_ip"].as<std::string>(), frequency);

    rtde_receive.startFileRecording(vm["output"].as<std::string>());
    std::cout << "Data recording started. press [Ctrl-C] to end recording." << std::endl;
    int i=0;
    while (flag_loop)
    {
      auto t_start = steady_clock::now();
      if (i % 10 == 0)
      {
        std::cout << '\r';
        printf("%.3d samples.", i);
        std::cout << std::flush;
      }
      auto t_stop = steady_clock::now();
      auto t_duration = std::chrono::duration<double>(t_stop - t_start);
      if (t_duration.count() < dt)
      {
        std::this_thread::sleep_for(std::chrono::duration<double>(dt - t_duration.count()));
      }
      i++;
    }

    // Stop data recording.
    rtde_receive.stopFileRecording();
    std::cout << "\nData recording stopped." << std::endl;
  }
  catch(std::exception& e) {
    std::cerr << "error: " << e.what() << "\n";
    return 1;
  }
  catch(...) {
    std::cerr << "Exception of unknown type!\n";
  }
  return 0;
}
