#include "sensor_library.h"


int main() {
    
    TemperatureSensor tempSensor(25.0);
    tempSensor.displayReading();
    tempSensor.updateReading(30.5);
    tempSensor.displayReading();

    DistanceSensor distSensor(100.0);
    distSensor.displayReading();
    distSensor.updateReading(150.0);
    distSensor.displayReading();

    return 0;
}