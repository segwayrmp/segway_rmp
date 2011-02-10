#include "segway_rmp200.h"
#include "ftdiexceptions.h"
#include "ftdiserver.h"
#include "ftdimodule.h"
#include <iostream>

/** \example test_segwayRMP200.cpp
 *
 * This example shows the basic operation of a Segway RMP200 platform.
 *
 * Before running this example it is important to follow the next steps:
 *
 * 	- check that E-Stop lanyard is plugged. (the red cordon). Motors won't
 * 	  start unless it is plugged
 *	- connect segway to USB slot of Pc host
 *	- start Segway UI processor (green button)
 *	- start Motors (yellow button)
 *
 * This example first creates an FTDI server to detect any possible segway
 * platform connected to the computer. It uses the add_custom_PID() function
 * to add the PID and VID combination used by Segway. It then opens the 
 * first device found, if any, using the serial number. At this point it is 
 * important that no other FTDI device is connected to the computer because 
 * the program does not check which kind of device it is.
 *
 * Once the FTDI device is opened and configured, a new CSegwayRMP200 object
 * is created an associated to the FTDI device. Then the robotic platform is
 * configured and commanded to move forward.
 *
 * While moving, every 1 second, the whole information of the platform is 
 * displayed on screen. Since the odometry of the robot uses inertial sensors,
 * if the platform is not actually moving, the displayed information will 
 * have no real meaning.
 *
 * It is important to note that it is not possible to execute any function 
 * of the public API of the CSegwayRMP200 class until a valid FTDI object 
 * has been associated to the segway object.
 *
 * If the motor are not enabled (yellow button turned off), the segway only
 * sends heartbeat commands, but it is not possible to send motion commands
 * or receive feedback data. In this case, the program will not fail, but
 * nothing will happen.
 */

std::string segway_name="segway";

int main(int argc, char *argv[])
{
  CSegwayRMP200 *segway;
  CFTDIServer *ftdi_server=CFTDIServer::instance();
  std::string serial_number;
  int i=0;

  try{
    ftdi_server->add_custom_PID(0xE729);
    std::cout << (*ftdi_server) << std::endl; 
    if(ftdi_server->get_num_devices()>0)
    {
      serial_number=ftdi_server->get_serial_number(0);
      segway=new CSegwayRMP200(segway_name);
      segway->connect(serial_number);
      segway->unlock_balance();
      segway->set_operation_mode(balance);
      segway->set_gain_schedule(light);
      segway->reset_right_wheel_integrator();
      usleep(10000);
      segway->reset_left_wheel_integrator();
      usleep(10000);
      segway->reset_yaw_integrator();
      usleep(10000);
      segway->reset_forward_integrator();
      usleep(10000);
      segway->move(1.0,0.0);
      for(i=0;i<10;i++)
      {
        sleep(1);
        std::cout << (*segway) << std::endl;
      }
      segway->stop();
      segway->close();
    }
  }catch(CException &e){
     std::cout << e.what() << std::endl;
  }
}
