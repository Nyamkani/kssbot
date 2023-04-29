#include "kssbot_diffdrive/kssbot_diffdrive.hpp"

int main(int argc, char **argv)
{
  raspmotor raspmotor_(BCM, 20, 20);

  raspmotor_.Initialize();

  raspmotor_.ActivateMotor();

  while(this->raspmotor_->is_run_)
  {
    this->raspmotor_->Drive();

    usleep(10000);
  }

  return ;
}