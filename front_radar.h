#ifndef __FRONTRADAR_H__
#define __FRONTRADAR_H__

#include "pcf8591.h"
#include "timer.h"

#include <map>
#include <string>

class FrontRadar
{
private:
    std::map<int, std::string> operation_state_map = {{0, "NO_DETECTION"}, {1, "DETECTION"}, {2, "WARNING"}, {3, "COLLISION"}};
    std::map<int, std::string> object_state_map = {{0, "NO_OBJECT"}, {1, "FAR"}, {2, "NEAR"}, {3, "INVALID"}};
    int pin;    
    double k = 0.05;
    double vk = 0.2;
    double period = 0.1;
    double volt2mm(int volt);
    double volt2cm(int volt);
    PCF8591 pcf8591;
    Timer timer;
    enum CONDITION
    {
        NO_OBJECT = 0,
        FAR = 1,
        NEAR = 2,
        INVALID = 3,
        
    };

    enum OPERATION
    {
        NO_DETECTION = 0,
        DETECTION = 1,
        WARNING = 2,
        COLLISION = 3,        
    };
    
    enum THRESHOLD
    {
        THRESHOLD_HYS = 2,
        THRESHOLD_INVALID = 10,
        THRESHOLD_VALID = 20,
        THRESHOLD_NEAR = 30,
        THRESHOLD_FAR = 60,
    };
    int object_state = NO_OBJECT;
    int operation_state = NO_DETECTION;
    void check_object_state();
    void getTargetDistance();
    void getTargetSpeed();
    void getTTC();
    void check_operation_state();
    double target_distance = THRESHOLD_FAR;
    double target_distance_past = THRESHOLD_FAR;
    double target_speed = 0;    
    double target_speed_past = 0;
    double target_ttc = THRESHOLD_FAR;
public:
    FrontRadar(/* args */){};
    void init();
    
    void run();
    ~FrontRadar(){};
};

#endif