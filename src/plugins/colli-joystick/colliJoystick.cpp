#include <logging/console.h>
#include <utils/time/time.h>
#include <blackboard/remote.h>
#include <interfaces/NavigatorInterface.h>
#include <interfaces/JoystickInterface.h>
#include <utils/system/argparser.h>

#include <iostream>
#include <string.h>
#include <string>
#include <cstring>
#include <cstdlib>
#include <cstdio>
#include <cstring>
#include <unistd.h>
#include <cstddef>
#include <stdio.h>
#include <math.h>


using namespace fawkes;
using namespace std;

class ColliJoystick
{
  public:
    ColliJoystick();
    ~ColliJoystick();
   void init_bb();
   void run();
  private:
   BlackBoard *bb_if;
   char* host;
   unsigned short int port;   
   JoystickInterface *m_joy; 
   NavigatorInterface *m_navi;
   float last_x,last_y;
};

ColliJoystick::ColliJoystick()
{
  init_bb();
  m_joy = bb_if->open_for_reading<JoystickInterface>("Joystick");
  m_navi = bb_if->open_for_reading<NavigatorInterface>("NavigatorTarget");
  NavigatorInterface::SetDriveModeMessage *drive_msg = new NavigatorInterface::SetDriveModeMessage();
  drive_msg->set_drive_mode(NavigatorInterface::ModerateAllowBackward);
  m_navi->msgq_enqueue(drive_msg);
  last_x = 0; 
  last_y = 0;
}

ColliJoystick::~ColliJoystick()
{
  bb_if->close(m_joy);
  bb_if->close(m_navi);
}

void ColliJoystick::init_bb()
{
  host = (char *)"localhost";
  port = 1910;
  bb_if = new RemoteBlackBoard(host, port);
}

void ColliJoystick::run()
{
  m_joy->read();
  float cur_x = m_joy->axis(0);
  float cur_y = m_joy->axis(1);
  float diff_x = cur_x - last_x;
  float diff_y = cur_y - last_y;
  float thresh_x = 0.1;
  float thresh_y = 0.1;
  if(( fabsf(diff_x) <= thresh_x ) && ( fabsf(diff_y) <= thresh_y ))
    return;

  float ini_phi = atan2(-m_joy->axis(1),m_joy->axis(0));
  float ini_dis = 1.0;
  NavigatorInterface::PolarGotoMessage *nav_msg = new NavigatorInterface::PolarGotoMessage(ini_phi,ini_dis,0);
  m_navi->msgq_enqueue(nav_msg);

  cout << "diffrences are: " << diff_x << " : " << diff_y << endl;
  cout << "last values are: " << last_x << " : " << last_y << endl;
  cout << "joystick values are: " << m_joy->axis(0) << ":" << m_joy->axis(1) << endl;
  last_x = cur_x; 
  last_y = cur_y;

  /*int size = m_joy->num_axes(); 
  float *axis_value = m_joy->axis();
  for( int i = 0; i < size; i++ )
  {
    cout << "axis "<< i << " is: " << axis_value[i] << endl;
  }*/
  /*if( m_joy->pressed_buttons() != 0 )
  {
    cout << "pressed button is: " << m_joy->pressed_buttons() << endl;
  }*/
   
}
 
int main(int argc, char **argv)
{
  ColliJoystick *mcolli = new ColliJoystick();
  while(true)
  {
    mcolli->run();
  }
  return 0;
}

