#ifndef KSSBOT_HARDWARE_WHEEL_H
#define KSSBOT_HARDWARE_WHEEL_H

#include <string>



// class Wheel
// {
//     public:

//     std::string name = "";
//     int enc = 0;
//     double cmd = 0;
//     double pos = 0;
//     double vel = 0;
//     double eff = 0;
//     double velSetPt = 0;
//     double rads_per_count = 0;

//     Wheel() = default;

//     Wheel(const std::string &wheel_name, int counts_per_rev);
    
//     void setup(const std::string &wheel_name, int counts_per_rev);

//     double calcEncAngle();



// };

class Wheel
{
    public:

    std::string name = "";
    int enc = 0;
    double cmd = 0;
    double pos = 0;
    double vel = 0;
    double eff = 0;
    double velSetPt = 0;

    Wheel() = default;

    Wheel(const std::string &wheel_name);
    
    void setup(const std::string &wheel_name);


};


#endif // DIFFDRIVE_ARDUINO_WHEEL_H