#include "../include/kssbot_diffdrive/raspmotorctrl.hpp"


//constructor

raspmotor::raspmotor()
{}

raspmotor::raspmotor(int pin_mode_type)
{
    this->pin_mode_type_ = pin_mode_type;

        this->is_init_ = false;
        this->is_run_ = false;
        this->is_err_ = false;

}

raspmotor::raspmotor(int pin_mode_type, int acc, int dec)
{
    this->pin_mode_type_ = pin_mode_type;

        this->is_init_ = false;
        this->is_run_ = false;
        this->is_err_ = false;

        this->pwm_acc_ = acc;
        this->pwm_dec_ = dec;
}

//Destructor
raspmotor::~raspmotor()
{



}

//---------------------------------------------------Initailize
void raspmotor::InitalizePinModeType()
{
    int pin_mode = this->pin_mode_type_;
    int result;

    switch(pin_mode)
    {
        case physical: result = wiringPiSetupPhys(); break;

        case wPi: result = wiringPiSetup(); break;
        
        case BCM: result = wiringPiSetupGpio(); break;

        case sys: result = wiringPiSetupSys(); break;

        default: result = -1; break;
    }

    if(result<= -1) throw init_pwm_mode_fail;

    return;
}

void raspmotor::InitalizePinMode()
{
    pinMode(Motor_A_EN, OUTPUT);

    pinMode(Motor_B_EN, OUTPUT);

    pinMode(Motor_A_Pin1, OUTPUT);

    pinMode(Motor_A_Pin2, OUTPUT);

    pinMode(Motor_B_Pin1, OUTPUT);

    pinMode(Motor_B_Pin2, OUTPUT);

    if(softPwmCreate(Motor_A_EN , 0, 100) <= -1 ) throw init_pwm_mode_fail;

    if(softPwmCreate(Motor_B_EN , 0, 100) <= -1 ) throw init_pwm_mode_fail;

    return;
}

//-----------------------------------------------------------main API

void raspmotor::Initialize()
{
    printf("Init motor config.\n");
    try 
    {
        //pwmSetMode(PWM_MODE_MS);

        InitalizePinModeType();

        InitalizePinMode();

        pwm_left_motor_queue.clear();

        pwm_right_motor_queue.clear();

        StopMotor();

        this->is_init_ = true;

    }

    catch(int error)
    {
        this-> error_code_ = error;

        this->is_err_ = true;

        this->is_run_ = false;

        this->is_init_ = false;

        printf("an error occcur: %d", this->error_code_);
    }

     return;
}

void raspmotor::ActivateMotor()
{
    this->is_run_ = true;

    return; 
}

void raspmotor::DeactivateMotor()
{
    this->is_run_ = false;

    return; 
}

void raspmotor::ResetMotor()
{
    this->is_err_ = false;

    Initialize();

    return; 
}

void raspmotor::LeftMotorControl(int direction, int speed)
{
    int direction_ = direction;
    int speed_ = speed;
    int tickcount = 0;
    int leftover = 0;
    int speedgap = 0;

    pwm_motor_val current_left_motor_status = read_left_motor_val_;
    pwm_motor_val temp_buffer_;
    
    //0. check current direction
    if(current_left_motor_status.pwm_motor_dir_ != direction_)
    {
        /*decreasee vel by deccel*/
        tickcount = (current_left_motor_status.pwm_motor_speed_) / (this->pwm_dec_);
        leftover = (current_left_motor_status.pwm_motor_speed_) % (this->pwm_dec_);

        if(tickcount != 0)
        {
            temp_buffer_.pwm_motor_dir_ = (current_left_motor_status.pwm_motor_dir_);
            temp_buffer_.pwm_motor_speed_ = (current_left_motor_status.pwm_motor_speed_ - ((this->pwm_dec_)));
                                                                
            LeftMotorDriveSaveQueue(temp_buffer_);

            return;
        }

        if(leftover != 0) 
        {
            temp_buffer_.pwm_motor_dir_ = (current_left_motor_status.pwm_motor_dir_);
            temp_buffer_.pwm_motor_speed_  = (current_left_motor_status.pwm_motor_speed_ - leftover);

            LeftMotorDriveSaveQueue(temp_buffer_);

            return;
        }

        //change motor direction
        temp_buffer_.pwm_motor_dir_ = direction_;
        temp_buffer_.pwm_motor_speed_  = (current_left_motor_status.pwm_motor_speed_);

        LeftMotorDriveSaveQueue(temp_buffer_);

        return;
        
    } 
    else 
    {
        //1. check current speed
        speedgap = this->read_left_motor_val_.pwm_motor_speed_ - speed_;

        if(speedgap < 0)
        {
            speedgap = abs(speedgap);

            tickcount = speedgap / (this->pwm_acc_);
            leftover = speedgap % (this->pwm_acc_);

            if(tickcount != 0)
            {
                temp_buffer_.pwm_motor_dir_ = direction_;
                temp_buffer_.pwm_motor_speed_ = current_left_motor_status.pwm_motor_speed_ + (this->pwm_acc_);

                LeftMotorDriveSaveQueue(temp_buffer_);

                return;
            }

            if(leftover != 0) 
            {
                temp_buffer_.pwm_motor_dir_ = direction_;
                temp_buffer_.pwm_motor_speed_  = (current_left_motor_status.pwm_motor_speed_+ leftover);

                LeftMotorDriveSaveQueue(temp_buffer_);

                return;
            }
        }
        else if(speedgap > 0)
        {
            speedgap = abs(speedgap);

            /*decreasee vel by accels*/
            tickcount = speedgap / (this->pwm_dec_);
            leftover = speedgap % (this->pwm_dec_);

            if(tickcount != 0)
            {
                temp_buffer_.pwm_motor_dir_ = direction_;
                temp_buffer_.pwm_motor_speed_ = (current_left_motor_status.pwm_motor_speed_ - (this->pwm_dec_));

                LeftMotorDriveSaveQueue(temp_buffer_);

               return;
            }

            if(leftover != 0) 
            {
                temp_buffer_.pwm_motor_dir_ = direction_;
                temp_buffer_.pwm_motor_speed_  = (current_left_motor_status.pwm_motor_speed_ - leftover);

                LeftMotorDriveSaveQueue(temp_buffer_);

                return;
            }
        }
    }

    return;
}

void raspmotor::RightMotorControl(int direction, int speed)
{
   int direction_ = direction;
    int speed_ = speed;
    int tickcount = 0;
    int leftover = 0;
    int speedgap = 0;

    pwm_motor_val current_right_motor_status = this->read_right_motor_val_;
    pwm_motor_val temp_buffer_;

    //0. check current direction
    if(current_right_motor_status.pwm_motor_dir_ != direction_)
    {
        /*decreasee vel by deccel*/
        tickcount = (current_right_motor_status.pwm_motor_speed_) / (this->pwm_dec_);
        leftover = (current_right_motor_status.pwm_motor_speed_) % (this->pwm_dec_);

        if(tickcount != 0)
        {
            temp_buffer_.pwm_motor_dir_ = (current_right_motor_status.pwm_motor_dir_);
            temp_buffer_.pwm_motor_speed_ = (current_right_motor_status.pwm_motor_speed_ - (this->pwm_dec_));

            RightMotorDriveSaveQueue(temp_buffer_);

            return;
        }

        if(leftover != 0) 
        {
            temp_buffer_.pwm_motor_dir_ = (current_right_motor_status.pwm_motor_dir_);
            temp_buffer_.pwm_motor_speed_  = (current_right_motor_status.pwm_motor_speed_ - leftover);

            RightMotorDriveSaveQueue(temp_buffer_);

            return;
        }

       //change motor direction
        temp_buffer_.pwm_motor_dir_ = direction_;
        temp_buffer_.pwm_motor_speed_  = (current_right_motor_status.pwm_motor_speed_);

        RightMotorDriveSaveQueue(temp_buffer_);

        return;
    } 
    else 
    {
        //1. check current speed
        speedgap = current_right_motor_status.pwm_motor_speed_ - speed_;

        if(speedgap < 0)
        {
            speedgap = abs(speedgap);

            tickcount = speedgap / (this->pwm_acc_);
            leftover = speedgap % (this->pwm_acc_);

            if(tickcount != 0)
            {
                temp_buffer_.pwm_motor_dir_ = direction_;
                temp_buffer_.pwm_motor_speed_ = current_right_motor_status.pwm_motor_speed_ + (this->pwm_acc_);

                RightMotorDriveSaveQueue(temp_buffer_);

                return;
            }

            if(leftover != 0) 
            {
                temp_buffer_.pwm_motor_dir_ = direction_;
                temp_buffer_.pwm_motor_speed_  = (current_right_motor_status.pwm_motor_speed_ + leftover);

                RightMotorDriveSaveQueue(temp_buffer_);

                return;
            }
        }
        else if(speedgap > 0)
        {
            speedgap = abs(speedgap);

            /*decreasee vel by accels*/
            tickcount = speedgap / (this->pwm_dec_);
            leftover = speedgap % (this->pwm_dec_);

            if(tickcount != 0)
            {
                temp_buffer_.pwm_motor_dir_ = direction_;
                temp_buffer_.pwm_motor_speed_ = current_right_motor_status.pwm_motor_speed_ - (this->pwm_dec_);

                RightMotorDriveSaveQueue(temp_buffer_);

                return;
            }

            if(leftover != 0) 
            {
                temp_buffer_.pwm_motor_dir_ = direction_;
                temp_buffer_.pwm_motor_speed_  = (current_right_motor_status.pwm_motor_speed_ - leftover);

                RightMotorDriveSaveQueue(temp_buffer_);

                return;
            }
        }
    }

    return;
}







void raspmotor::LeftMotorDriveSaveQueue(pwm_motor_val data)
{
    this->pwm_left_motor_queue.push_back(data);
    //printf("left / vector size is : %ld, vector element size is %ld\n", pwm_left_motor_queue.size(), sizeof(pwm_left_motor_queue));

    return;
}


void raspmotor::RightMotorDriveSaveQueue(pwm_motor_val data)
{
    this->pwm_right_motor_queue.push_back(data);
    //printf("right / vector size is : %ld, vector element size is %ld\n", pwm_right_motor_queue.size(), sizeof(pwm_right_motor_queue));
    return;
}


void raspmotor::LinkRosToRasp(double l_motor_cmd, double r_motor_cmd)
{
    const int l_motor_val = (int)l_motor_cmd;
    const int r_motor_val = (int)r_motor_cmd;

    //1. check upcoming command. If command is same with present. ignore upcoming command
    if(this->write_left_motor_val_ != l_motor_val) 
        this->write_left_motor_val_  = l_motor_val;

    if(this->write_right_motor_val_ != r_motor_val) 
        this->write_right_motor_val_ = r_motor_val;
    
    //printf("left : %d, right : %d \n", l_motor_cmd_, r_motor_cmd_);
    //printf("left : %f, right : %f \n", l_motor_cmd, r_motor_cmd);
    return;
}

void raspmotor::PreMotorDrive()
{
    int l_motor_cmd_ = this->write_left_motor_val_;
    int l_dir = forward;
    int r_motor_cmd_ = this->write_right_motor_val_;
    int r_dir = forward;

    if(l_motor_cmd_< 0) l_dir = backward;
    if(r_motor_cmd_< 0) r_dir = backward;

    l_motor_cmd_ = abs(l_motor_cmd_);
    r_motor_cmd_ = abs(r_motor_cmd_);

    if(l_motor_cmd_ >= 100) l_motor_cmd_ = 100;
    else if(l_motor_cmd_ <= 2) l_motor_cmd_ *= this->motor_vel;
    else l_motor_cmd_ = 2 * this->motor_vel + l_motor_cmd_;

    if(r_motor_cmd_ >= 100) r_motor_cmd_ = 100;
    else if(r_motor_cmd_ <= 2) r_motor_cmd_ *= this->motor_vel;
    else r_motor_cmd_ = 2 * this->motor_vel + r_motor_cmd_;

    pwm_left_motor_queue.clear();

    pwm_right_motor_queue.clear();

    LeftMotorControl(l_dir, l_motor_cmd_);

    RightMotorControl(r_dir, r_motor_cmd_);

    return;
}

void raspmotor::StopMotor()
{
    digitalWrite(Motor_A_Pin1, LOW);

    digitalWrite(Motor_A_Pin2, LOW);

    digitalWrite(Motor_B_Pin1, LOW);

    digitalWrite(Motor_B_Pin2, LOW);

    digitalWrite(Motor_A_EN, LOW);

    digitalWrite(Motor_B_EN, LOW);

    return;
}

void raspmotor::LeftMotorDrive()
{
    if(this->pwm_left_motor_queue.empty()) return;
       
    const pwm_motor_val temp_left_motor_val = pwm_left_motor_queue.front();

    if(temp_left_motor_val.pwm_motor_dir_ == backward)
    {
        digitalWrite(Motor_B_Pin1, HIGH);

        digitalWrite(Motor_B_Pin2, LOW);
    }

    else
    {
        digitalWrite(Motor_B_Pin1, LOW);

        digitalWrite(Motor_B_Pin2, HIGH);
    }

    softPwmWrite(Motor_B_EN, temp_left_motor_val.pwm_motor_speed_);

    this->read_left_motor_val_ = temp_left_motor_val;

    return;
}

void raspmotor::RightMotorDrive()
{
    if(this->pwm_right_motor_queue.empty()) return;


    const pwm_motor_val temp_right_motor_val = pwm_right_motor_queue.front();

    if(temp_right_motor_val.pwm_motor_dir_ == backward)
    {
        digitalWrite(Motor_A_Pin1, HIGH);

        digitalWrite(Motor_A_Pin2, LOW);
    }

    else
    {
        digitalWrite(Motor_A_Pin1, LOW);

        digitalWrite(Motor_A_Pin2, HIGH);
    }

    softPwmWrite(Motor_A_EN, temp_right_motor_val.pwm_motor_speed_);

    this->read_right_motor_val_ = temp_right_motor_val;

    return;
}

void raspmotor::MotorDrive()
{

    LeftMotorDrive();

    RightMotorDrive();

    return;
}


void raspmotor::LeftPostMotorDrive()
{
     if(!(this->pwm_left_motor_queue.empty()))
     {
        //1. record motor values
        this->read_left_motor_val_= this->pwm_left_motor_queue.front();

        //2. delete queues
        this->pwm_left_motor_queue.erase(this->pwm_left_motor_queue.begin());
     }

     return;
}

void raspmotor::RightPostMotorDrive()
{
    if(!(this->pwm_right_motor_queue.empty()))
    {
        //1. record motor values
        this->read_right_motor_val_ = this->pwm_right_motor_queue.front();

        //2. delete queues
        this->pwm_right_motor_queue.erase(this->pwm_right_motor_queue.begin());
    }
    return;
}

void raspmotor::PostMotorDrive()
{

    LeftPostMotorDrive();

    RightPostMotorDrive();

    //printf("left speed : %d, Right speed : %d\n", read_left_motor_val_.pwm_motor_speed_,read_right_motor_val_.pwm_motor_speed_ );

    return;

}

bool raspmotor::Drive()
{
    if(!(this->is_run_)) return false;

    PreMotorDrive();

    MotorDrive();

    PostMotorDrive();
    
    return true;
}