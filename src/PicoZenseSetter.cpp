#include "PicoZenseSetter.h"

using namespace std;

void
PicoZenseSensorSetter::initialize(int _image_width, int _image_height, int _image_fps){
    PsReturnStatus status;
    int32_t deviceCount = 0;
    status = PsGetDeviceCount(&deviceCount);
    if (status != PsReturnStatus::PsRetOK)
    {
        cout << "PsGetDeviceCount failed!" << endl;
        return;
    }

    n_sensor = deviceCount;
    for(int32_t device_index=0; device_index < n_sensor; device_index++ ){
        PicoZenseSensor _sensor;
        _sensor.initialize(device_index, _image_width, _image_height, _image_fps);
        pico_zense_sensor_list.push_back(_sensor);

        cout << "sensor number: " << device_index << endl;   
        cout << "version : " << _sensor.name << endl;
        cout << "serial  " << _sensor.serial_number << endl;

        bm_idx2serial.insert(bimap_value_t(device_index, _sensor.serial_number));
    }
}

void
PicoZenseSensorSetter::setSensorObject(std::vector<SensorWrapper> &_sensor_vec){
    for(int i=0;i<n_sensor;i++){
        SensorWrapper _sensor_wrapper;
        _sensor_wrapper._get_rgb_image_func = [=](cv::Mat &_rgb){
            _rgb = pico_zense_sensor_list[i].getRGBImage();
        };
        _sensor_wrapper._get_depth_image_func = [=](cv::Mat &_depth){
            _depth = pico_zense_sensor_list[i].getDepthImage();
        };
        _sensor_wrapper._get_colorized_depth_image_func = [=](cv::Mat &_colorized_depth){
            _colorized_depth = pico_zense_sensor_list[i].getColorizedDepthImage();
        };
        _sensor_wrapper._get_ir_image_func = [=](cv::Mat &_ir){
            _ir = pico_zense_sensor_list[i].getIRImage();
        };
        _sensor_wrapper._get_camera_parameter_func = [=](CameraParameter &_camera_parameter){
            _camera_parameter = pico_zense_sensor_list[i].getCameraParameter(i);
        };
        _sensor_wrapper._start_func = [=](){
            pico_zense_sensor_list[i].start();
        };
        _sensor_wrapper._stop_func = [=](){
            pico_zense_sensor_list[i].stop(i);
        };
        _sensor_wrapper._update_func = [=](){
            pico_zense_sensor_list[i].update(i);
        };
        _sensor_vec.emplace_back(_sensor_wrapper);

    }
}

int
PicoZenseSensorSetter::getNumSensor(){
    return n_sensor;
}