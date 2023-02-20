#include <wiringPi.h>
#include <softPwm.h>
#include <vector>
#include <thread>
#include <iostream>		// Include all needed libraries here

/*
 +-----+-----+---------+------+---+---Pi 4B--+---+------+---------+-----+-----+
 | BCM | wPi |   Name  | Mode | V | Physical | V | Mode | Name    | wPi | BCM |
 +-----+-----+---------+------+---+----++----+---+------+---------+-----+-----+
 |     |     |    3.3v |      |   |  1 || 2  |   |      | 5v      |     |     |
 |   2 |   8 |   SDA.1 | ALT0 | 1 |  3 || 4  |   |      | 5v      |     |     |
 |   3 |   9 |   SCL.1 | ALT0 | 1 |  5 || 6  |   |      | 0v      |     |     |
 |   4 |   7 | GPIO. 7 |   IN | 1 |  7 || 8  | 1 | IN   | TxD     | 15  | 14  |
 |     |     |      0v |      |   |  9 || 10 | 1 | IN   | RxD     | 16  | 15  |
 |  17 |   0 | GPIO. 0 |   IN | 0 | 11 || 12 | 0 | IN   | GPIO. 1 | 1   | 18  |
 |  27 |   2 | GPIO. 2 |   IN | 0 | 13 || 14 |   |      | 0v      |     |     |
 |  22 |   3 | GPIO. 3 |   IN | 0 | 15 || 16 | 0 | IN   | GPIO. 4 | 4   | 23  |
 |     |     |    3.3v |      |   | 17 || 18 | 0 | IN   | GPIO. 5 | 5   | 24  |
 |  10 |  12 |    MOSI | ALT0 | 0 | 19 || 20 |   |      | 0v      |     |     |
 |   9 |  13 |    MISO | ALT0 | 0 | 21 || 22 | 0 | IN   | GPIO. 6 | 6   | 25  |
 |  11 |  14 |    SCLK | ALT0 | 0 | 23 || 24 | 1 | OUT  | CE0     | 10  | 8   |
 |     |     |      0v |      |   | 25 || 26 | 1 | OUT  | CE1     | 11  | 7   |
 |   0 |  30 |   SDA.0 |   IN | 1 | 27 || 28 | 1 | IN   | SCL.0   | 31  | 1   |
 |   5 |  21 | GPIO.21 |   IN | 1 | 29 || 30 |   |      | 0v      |     |     |
 |   6 |  22 | GPIO.22 |   IN | 1 | 31 || 32 | 0 | IN   | GPIO.26 | 26  | 12  |
 |  13 |  23 | GPIO.23 |   IN | 0 | 33 || 34 |   |      | 0v      |     |     |
 |  19 |  24 | GPIO.24 |   IN | 0 | 35 || 36 | 0 | IN   | GPIO.27 | 27  | 16  |
 |  26 |  25 | GPIO.25 |   IN | 0 | 37 || 38 | 0 | IN   | GPIO.28 | 28  | 20  |
 |     |     |      0v |      |   | 39 || 40 | 0 | IN   | GPIO.29 | 29  | 21  |
 +-----+-----+---------+------+---+----++----+---+------+---------+-----+-----+
 | BCM | wPi |   Name  | Mode | V | Physical | V | Mode | Name    | wPi | BCM |
 +-----+-----+---------+------+---+---Pi 4B--+---+------+---------+-----+-----+
*/

enum pin_mode
{
    physical = 0,
    wPi = 1,
    BCM = 2,
    sys = 3,
};

enum raspberrypi_4_pin_number
{
    Motor_A_EN = 4,
    Motor_B_EN = 17,
    Motor_A_Pin1 = 27,
    Motor_A_Pin2 = 18,
    Motor_B_Pin1 = 26,
    Motor_B_Pin2 = 21,
};

enum motor_direction
{
    forward = 0,
    backward = 1,
};

enum motor_error
{
    init_pin_mode_fail = 0,
    init_pwm_mode_fail = 1,

    drive_motor_failed = 10,

};

typedef struct 
{
    int pwm_motor_dir_;
    int pwm_motor_speed_;
} pwm_motor_val;

class raspmotor
{
    //--------------------------------------------Values
    public:
        bool is_init_;
        bool is_run_;
        bool is_err_;

    private:
        int pin_mode_type_;
        int error_code_;
        int pwm_acc_ = 0;
        int pwm_dec_ = 0;
        int motor_vel = 50;

        pwm_motor_val read_left_motor_val_= {0,0};
        pwm_motor_val read_right_motor_val_ = {0,0};

        std::vector<pwm_motor_val> pwm_left_motor_queue; 
        std::vector<pwm_motor_val> pwm_right_motor_queue; 

    //--------------------------------------------Functions   
    public:
        raspmotor();

        raspmotor(int pin_mode_type);

        raspmotor(int pin_mode_type, int acc, int dec);

        virtual ~raspmotor();

        void Initialize();

        void LinkRosToRasp(int l_motor_cmd, int r_motor_cmd);

        void StopMotor();

        bool Drive();


    private:
        void InitalizePinModeType();

        void InitalizePinMode();

        void LeftMotorControl(int direction, int speed);

        void RightMotorControl(int direction, int speed);

        void LeftMotorDrive();

        void RightMotorDrive();

        void MotorDrive();

        void LeftPostMotorDrive();

        void RightPostMotorDrive();

        void PostMotorDrive();

        void LeftMotorDriveSaveQueue(pwm_motor_val data);

        void RightMotorDriveSaveQueue(pwm_motor_val data);

};