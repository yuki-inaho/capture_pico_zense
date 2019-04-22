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

    // 暫定serial固定
    int32_t device_index = 0;
    std::string sn = "zense";
    PicoZenseSensor _sensor;
    _sensor.initialize(device_index, _image_width, _image_height, _image_fps);
    pico_zense_sensor_list.push_back(_sensor);
    bm_idx2serial.insert(bimap_value_t(device_index, sn));

}

void
PicoZenseSensorSetter::setSensorObject(std::vector<SensorWrapper> &_sensor_vec){
    SensorWrapper _sensor_wrapper;
    _sensor_wrapper._get_rgb_image_func = [=](cv::Mat &_rgb){
        _rgb = pico_zense_sensor_list[0].getRGBImage();
    };
    _sensor_wrapper._get_depth_image_func = [=](cv::Mat &_depth){
        _depth = pico_zense_sensor_list[0].getDepthImage();
    };
    _sensor_wrapper._get_colorized_depth_image_func = [=](cv::Mat &_colorized_depth){
        _colorized_depth = pico_zense_sensor_list[0].getColorizedDepthImage();
    };
    _sensor_wrapper._get_ir_image_func = [=](cv::Mat &_ir){
        _ir = pico_zense_sensor_list[0].getIRImage();
    };
    _sensor_wrapper._get_camera_parameter_func = [=](CameraParameter &_camera_parameter){
        _camera_parameter = pico_zense_sensor_list[0].getCameraParameter(0);
    };
    _sensor_wrapper._start_func = [=](){
        pico_zense_sensor_list[0].start();
    };
    _sensor_wrapper._stop_func = [=](){
        pico_zense_sensor_list[0].stop(0);
    };
    _sensor_wrapper._update_func = [=](){
        pico_zense_sensor_list[0].update(0);
    };
    _sensor_vec.emplace_back(_sensor_wrapper);
}

int
PicoZenseSensorSetter::getNumSensor(){
    return n_sensor;
}