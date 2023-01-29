#include <diffdrive_rasp4.hpp>
#include <chrono>
#include <thread>

//constructor
RaspMotorDriver::RaspMotorDriver()
{
  this->speed_ = 0; 
  this->direction_ = 0;
}

//Destructor
RaspMotorDriver::~RaspMotorDriver()
{
  //if(this->raspmotor_ != NULL) delete this->raspmotor_;
}


void RaspMotorDriver::InitializeDevice()
{
  if(this->raspmotor_ == NULL) this->raspmotor_ = std::make_unique<raspmotor>(BCM, 3, 3);

  this->raspmotor_->Initialize();

  return;
}


bool RaspMotorDriver::DriveDevice()
{
  bool result = false;

  result = this->raspmotor_->Drive();
  
  return result;
}


void RaspMotorDriver::LeftMotorControl_(int dir, int speed)
{
  int dir_ = dir;
  int speed_ = speed;

  raspmotor_->LeftMotorControl(dir_, speed_);

  return;
}

void RaspMotorDriver::RightMotorControl_(int dir, int speed)
{
  int dir_ = dir;
  int speed_ = speed;

  raspmotor_->RightMotorControl(dir_, speed_);

  return;
}





int main(int argc, char ** argv) 
{
    using namespace std::this_thread;     // sleep_for, sleep_until
    using namespace std::chrono_literals; // ns, us, ms, s, h, etc.
    using std::chrono::system_clock;

  (void) argc;
  (void) argv;
  RaspMotorDriver motorctrl_;


  motorctrl_.InitializeDevice();


  motorctrl_.LeftMotorControl_(forward, 50);
  motorctrl_.RightMotorControl_(forward, 50);

  motorctrl_.LeftMotorControl_(backward, 50);
  motorctrl_.RightMotorControl_(backward, 50);

  motorctrl_.LeftMotorControl_(backward, 50);
  motorctrl_.RightMotorControl_(forward, 50);

  motorctrl_.LeftMotorControl_(forward, 50);
  motorctrl_.RightMotorControl_(backward, 50);

  motorctrl_.LeftMotorControl_(backward, 50);
  motorctrl_.RightMotorControl_(backward, 50);

  motorctrl_.LeftMotorControl_(forward, 0);
  motorctrl_.RightMotorControl_(forward, 0);


while(1)
{
  if(!(motorctrl_.DriveDevice())) break;
    sleep_for(100ms); 
}

 // motorctrl_.StopMotor();
  return 0;
}
