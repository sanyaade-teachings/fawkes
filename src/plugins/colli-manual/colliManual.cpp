#include <logging/console.h>
#include <utils/time/time.h>
#include <blackboard/remote.h>
#include <interfaces/NavigatorInterface.h>
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

class ColliManualControl
{
 public:
  ColliManualControl(int argc, char **argv);
  ~ColliManualControl();
  void init_bb();
  void test();
  void run();
  void print_usage();
 private:
  NavigatorInterface *m_Target;
  BlackBoard *bb_if;
  ArgumentParser argp;
  char* host;
  unsigned short int port;
  float tar_x,tar_y;
  string drive_mode_str;
  bool setdmode;
  bool setdest;
  int dmodeId;
};
//-----------------------------------------------------------------------------
ColliManualControl::ColliManualControl(int argc, char **argv): argp(argc, argv, "hr:p:li")
{
  init_bb();
  m_Target = bb_if->open_for_reading<NavigatorInterface>("NavigatorTarget");
  tar_x = -1;
  tar_y = -1;
  setdmode = false;
  setdest = false;
  dmodeId = -1;
}
//-----------------------------------------------------------------------------
ColliManualControl::~ColliManualControl()
{
  bb_if->close(m_Target);
  delete bb_if;
}
//----------------------------------------------------------------------------
void ColliManualControl::init_bb()
{
  host = (char *)"localhost";
  port = 1910;
  bb_if = new RemoteBlackBoard(host, port);
}
//------------------------------------------------------------------------------
void ColliManualControl::print_usage()
{
    printf("Usage: [-h] <command>\n"
         " -h              This help message\n"
         " Command:\n"
         "coordx [targetx] coordy [targety] drive_mode [DriveMode]\n");

}
//-----------------------------------------------------------------------------
void ColliManualControl::test()
{
  ArgumentParser *argp_;
  argp_ = new ArgumentParser(argp);
  if ( argp_->has_arg("h") ) 
  {
    //cout << "help message requested" << endl;
    print_usage();
    exit(0);
  }
  NavigatorInterface::SetDriveModeMessage *drive_msg = new NavigatorInterface::SetDriveModeMessage();
  const std::vector< const char * > &items = argp_->items();
  for (unsigned int i = 0; i < items.size(); i++)
  {
    if( strcmp(items[i],"coordx") == 0 )
    {
      if(items.size() > i+1){
      i++;
      float p = argp_->parse_item_float(i);
      cout << "coordx " << p << endl;
      tar_x = p;
      }
      else
       cout << "no x coordinate provided" << endl;
    }
    if( strcmp(items[i],"coordy") == 0 )
    {
      if(items.size() > i+1){
      i++;
      float p = argp_->parse_item_float(i);
      cout << "coordy " << p << endl;
      tar_y = p;
      }
      else
       cout << "no y coordinate provided" << endl;
    }

    if(strcmp(items[i],"drive_mode") == 0 )
    {
      if(items.size() > i+1 )
      {
        setdmode = true;
        const char * dmode = items[++i];
        cout << "drive mode: "<< dmode << endl;

        if(strcmp(dmode,"CarefulForward") == 0)
        {
          drive_msg->set_drive_mode(NavigatorInterface::CarefulForward);
          dmodeId = NavigatorInterface::CarefulForward;
        }
        else if(strcmp(dmode,"MovingNotAllowed") == 0)
        {
          dmodeId = NavigatorInterface::MovingNotAllowed;
          drive_msg->set_drive_mode(NavigatorInterface::MovingNotAllowed);
        }
        else if(strcmp(dmode,"SlowForward") == 0)
        {
          drive_msg->set_drive_mode(NavigatorInterface::SlowForward);
          dmodeId = NavigatorInterface::SlowForward;
        }
        else if(strcmp(dmode,"ModerateForward") == 0)
        {
          drive_msg->set_drive_mode(NavigatorInterface::ModerateForward);
          dmodeId = NavigatorInterface::ModerateForward;
        }
        else if(strcmp(dmode,"FastForward") == 0)
        {
          drive_msg->set_drive_mode(NavigatorInterface::FastForward);
          dmodeId = NavigatorInterface::FastForward;
        }
        else if(strcmp(dmode,"CarefulAllowBackward") == 0)
        {
          drive_msg->set_drive_mode(NavigatorInterface::CarefulAllowBackward);
          dmodeId = NavigatorInterface::CarefulAllowBackward;
        }
        else if(strcmp(dmode,"SlowAllowBackward") == 0)
        {
          drive_msg->set_drive_mode(NavigatorInterface::SlowAllowBackward);
          dmodeId = NavigatorInterface::SlowAllowBackward;
        }
        else if(strcmp(dmode,"ModerateAllowBackward") == 0)
        {
          drive_msg->set_drive_mode(NavigatorInterface::ModerateAllowBackward);
          dmodeId = NavigatorInterface::ModerateAllowBackward;
        }
        else if(strcmp(dmode,"FastAllowBackward") == 0)
        {
          drive_msg->set_drive_mode(NavigatorInterface::FastAllowBackward);
          dmodeId = NavigatorInterface::FastAllowBackward;
        }
        else if(strcmp(dmode,"CarefulBackward") == 0)
        {
          drive_msg->set_drive_mode(NavigatorInterface::CarefulBackward);
          dmodeId = NavigatorInterface::CarefulBackward;
        }
        else if(strcmp(dmode,"SlowBackward") == 0)
        {
          drive_msg->set_drive_mode(NavigatorInterface::SlowBackward);
          dmodeId = NavigatorInterface::SlowBackward;
        }
        else if(strcmp(dmode,"ModerateBackward") == 0)
        {
          drive_msg->set_drive_mode(NavigatorInterface::ModerateBackward);
          dmodeId = NavigatorInterface::ModerateBackward;
        }
        else if(strcmp(dmode,"FastBackward") == 0)
        {
          drive_msg->set_drive_mode(NavigatorInterface::FastBackward);
          dmodeId = NavigatorInterface::FastBackward;
        }
        else if(strcmp(dmode,"ESCAPE") == 0)
        {
          drive_msg->set_drive_mode(NavigatorInterface::ESCAPE);
          dmodeId = NavigatorInterface::ESCAPE;
        }
        else
        {
          cout << "drive mode is not supported" << endl;
          setdmode = false;
        }
      }
    }

  }

  if(setdmode)
  {
    m_Target->msgq_enqueue(drive_msg);
  }
  if(( tar_x != -1 ) && ( tar_y != -1 ))
  {
    setdest = true;
    NavigatorInterface::ObstacleMessage *msg = new NavigatorInterface::ObstacleMessage();
    msg->set_x(tar_x);
    msg->set_y(tar_y);
    m_Target->msgq_enqueue(msg);
  }
  else if(!setdmode)
  {
    print_usage();
    exit(0);
  }
}

void ColliManualControl::run()
{
  test();
  while(true)
  {
    m_Target->read();
    bool b_drive = true;
    if( setdmode )
    {
      b_drive = ( m_Target->drive_mode() == dmodeId );
    }
    bool b_dest = true;
    if( setdest )
    {
      b_dest = (( m_Target->dest_x() == tar_x ) && ( m_Target->dest_y() == tar_y ) );
    }
    if( ( b_drive ) && ( b_dest ) )
    {
      break;
    }
  }
}
//----------------------------------------------------------------------------
int main(int argc, char **argv)
{
  ColliManualControl *mcolli = new ColliManualControl(argc,argv);
  mcolli->run();
  return 0;
}
