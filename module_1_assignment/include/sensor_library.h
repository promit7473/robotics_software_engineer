#ifndef SENSORLIB
#define SENSORLIB

#include <iostream>
#include <string>

using namespace std;

// Template class for a generic sensor
template <typename T>

class Sensor {
private:
    T reading;

public:
    Sensor(T initialReading) : reading(initialReading) {
        cout<<"Sensor is initialized!"<<endl;
    }

    T getReading() const {
        return reading;
    }

    void updateReading(T newReading) {
        reading = newReading;
    }
};


class TemperatureSensor : public Sensor<double> {
public:
    TemperatureSensor(double initialTemp) : Sensor<double>(initialTemp) {}

    void displayReading() {
        cout << "Temperature: " << getReading() << "°C" << endl;
    }
};

class DistanceSensor : public Sensor<double> {
public:
    DistanceSensor(double initialDistance) : Sensor<double>(initialDistance) {}

    void displayReading() {
        cout << "Distance: " << getReading() << " meters" << endl;
    }
};

#endif //SENSORLIB