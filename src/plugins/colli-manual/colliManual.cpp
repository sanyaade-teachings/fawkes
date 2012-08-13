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
 private:
  NavigatorInterface *m_Target;
  BlackBoard *bb_if;
  ArgumentParser argp;
  char* host;
  unsigned short int port;
 // ArgumentParser *argp;
};
//-----------------------------------------------------------------------------
ColliManualControl::ColliManualControl(int argc, char **argv): argp(argc, argv, "hr:p:li")
{
  init_bb();

  m_Target = bb_if->open_for_writing<NavigatorInterface>("NavigatorTarget");

  m_Target->set_dest_x(0.0);
  m_Target->set_dest_y(0.0);
  m_Target->set_dest_ori(0.0);
  m_Target->set_colliMode(NavigatorInterface::ModerateAllowBackward);
  m_Target->write();
}
//-----------------------------------------------------------------------------
ColliManualControl::~ColliManualControl()
{
  bb_if->close(m_Target);
}
//----------------------------------------------------------------------------
void ColliManualControl::init_bb()
{
  host = (char *)"localhost";
  port = 1910;
  bb_if = new RemoteBlackBoard(host, port);
}
//-----------------------------------------------------------------------------
void ColliManualControl::test()
{
  ArgumentParser *argp_;
  //const char **argv = argp.argv();
  argp_ = new ArgumentParser(argp);
 
  const std::vector< const char * > &items = argp_->items();
  for (unsigned int i = 0; i < items.size(); i++)
  {
    if( strcmp(items[i],"coordx") == 0 )
    {
      if(items.size() > i+1){
      m_Target->set_dest_x(argp_->parse_item_float(++i));
      float p = argp_->parse_item_float(i);
      cout << "coordx " << p << endl;
      }
      else
       cout << "no x coordinate provided" << endl;
    }
    if( strcmp(items[i],"coordy") == 0 )
    {
      if(items.size() > i+1){
      m_Target->set_dest_y(argp_->parse_item_float(++i));
      float p = argp_->parse_item_float(i);
      cout << "coordy " << p << endl;
      }
      else
       cout << "no y coordinate provided" << endl;
    }
    if(strcmp(items[i],"drive_mode") == 0 )
    {
      if(items.size() > i+1 )
      {
        const char * dmode = items[++i];
        cout << "drive mode: "<< dmode << endl;

        if(strcmp(dmode,"CarefulForward") == 0)  
          m_Target->set_colliMode(NavigatorInterface::CarefulForward);
        else if(strcmp(dmode,"MovingNotAllowed") == 0)
          m_Target->set_colliMode(NavigatorInterface::MovingNotAllowed);
        else if(strcmp(dmode,"SlowForward") == 0)
          m_Target->set_colliMode(NavigatorInterface::SlowForward);
        else if(strcmp(dmode,"ModerateForward") == 0)
          m_Target->set_colliMode(NavigatorInterface::ModerateForward);
        else if(strcmp(dmode,"FastForward") == 0)
          m_Target->set_colliMode(NavigatorInterface::FastForward);
        else if(strcmp(dmode,"CarefulAllowBackward") == 0)
          m_Target->set_colliMode(NavigatorInterface::CarefulAllowBackward);
        else if(strcmp(dmode,"SlowAllowBackward") == 0)
          m_Target->set_colliMode(NavigatorInterface::SlowAllowBackward);
        else if(strcmp(dmode,"ModerateAllowBackward") == 0)
          m_Target->set_colliMode(NavigatorInterface::ModerateAllowBackward);
        else if(strcmp(dmode,"FastAllowBackward") == 0)
          m_Target->set_colliMode(NavigatorInterface::FastAllowBackward);
        else if(strcmp(dmode,"CarefulBackward") == 0)
          m_Target->set_colliMode(NavigatorInterface::CarefulBackward);
        else if(strcmp(dmode,"SlowBackward") == 0)
          m_Target->set_colliMode(NavigatorInterface::SlowBackward);
        else if(strcmp(dmode,"ModerateBackward") == 0)
          m_Target->set_colliMode(NavigatorInterface::ModerateBackward);
        else if(strcmp(dmode,"FastBackward") == 0)
          m_Target->set_colliMode(NavigatorInterface::FastBackward);
        else if(strcmp(dmode,"ESCAPE") == 0)
          m_Target->set_colliMode(NavigatorInterface::ESCAPE);
        else
        {
          cout << "drive mode is not supported" << endl;
  //        m_Target->set_colliMode(NavigatorInterface::MovingNotAllowed);
        }
      }
    }    
  }
  m_Target->write();
  cout << "target drive mode: " << m_Target->GetColliMode() << endl;
}

void ColliManualControl::run()
{
  test();
  while(true){
  }
}
//----------------------------------------------------------------------------
int main(int argc, char **argv)
{
  ColliManualControl *mcolli = new ColliManualControl(argc,argv);
  //mcolli->test();
  mcolli->run();
  return 0;
}
