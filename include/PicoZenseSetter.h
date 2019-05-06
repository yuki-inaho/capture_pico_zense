#pragma once
#include "header.h"
#include "SensorWrapper.h"
#include "PicoZenseSensorClass.h"

class PicoZenseSensorSetter{
public:

    void initialize(int _image_width, int _image_height, int _image_fps);
    void setSensorObject(std::vector<SensorWrapper> &_sensor_vec);
    int getNumSensor();
    bimap_t bm_idx2serial;

private:

    int32_t n_sensor;
    std::vector<PicoZenseSensor> pico_zense_sensor_list;
};