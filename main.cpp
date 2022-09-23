#include <iostream>
#include "diablo_utils/diablo_tools/osdk_vehicle.hpp"


int main(int argc, char **argv) {
    DIABLO::OSDK::Movement_Ctrl* pMovementCtrl;
    
    DIABLO::OSDK::HAL_Pi Hal;
    if(Hal.init("/dev/ttyAMA0")) return -1;

    DIABLO::OSDK::Vehicle vehicle(&Hal);                     
    if(vehicle.init()) return -1;

    vehicle.telemetry->activate();
    
    pMovementCtrl = vehicle.movement_ctrl;
    
    return 0;
}
