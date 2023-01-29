#include "raspmotorctrl.hpp"
#include <cstdio>

class RaspMotorDriver
{
    public:
        RaspMotorDriver();
        virtual ~RaspMotorDriver();

        void InitializeDevice();

        bool DriveDevice();

        void LeftMotorControl_(int dir, int speed);

        void RightMotorControl_(int dir, int speed);
        
    private:



    public:
        std::unique_ptr<raspmotor> raspmotor_{nullptr};

    private:
        /// @brief 
        int speed_;
        int direction_;

};