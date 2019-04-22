#include "SensorManager.h"
#include "iostream"

using namespace std;

void
SensorManager::setIdxSerialMap(bimap_t _bm_idx2serial)
{
    bm_idx2serial = _bm_idx2serial;
}

void
SensorManager::setSensors(std::vector<SensorWrapper> &_sensor_vec)
{
    sensor_vec = _sensor_vec ;
}

void 
SensorManager::activateSensor(std::string _serial_number){
    cout << "test" << " " << _serial_number << endl;
    int sensor_idx = bm_idx2serial.right.at(_serial_number);
    present_serial = _serial_number;
    
    _get_rgb_image_func = sensor_vec[sensor_idx]._get_rgb_image_func;
    _get_depth_image_func = sensor_vec[sensor_idx]._get_depth_image_func;
    _get_ir_image_func = sensor_vec[sensor_idx]._get_ir_image_func;
    _get_colorized_depth_image_func = sensor_vec[sensor_idx]._get_colorized_depth_image_func;
    _get_camera_parameter_func  = sensor_vec[sensor_idx]._get_camera_parameter_func;
    _update_func = sensor_vec[sensor_idx]._update_func;
    _start_func = sensor_vec[sensor_idx]._start_func;
    _stop_func = sensor_vec[sensor_idx]._stop_func;
}

cv::Mat 
SensorManager::getRGBImage()
{
    cv::Mat _rgb;
    _get_rgb_image_func(_rgb);
    return _rgb;
}

cv::Mat 
SensorManager::getDepthImage()
{
    cv::Mat _depth;
    _get_depth_image_func(_depth);
    return _depth;
}        

cv::Mat 
SensorManager::getIRImage()
{
    cv::Mat _ir;
    _get_ir_image_func(_ir);
    return _ir;
}        

cv::Mat 
SensorManager::getColorizedDepthImage()
{
    cv::Mat _colorized_depth;
    _get_colorized_depth_image_func(_colorized_depth);
    return _colorized_depth;
}        

CameraParameter 
SensorManager::getCameraParameter()
{
    CameraParameter cam_p;
    _get_camera_parameter_func(cam_p);
    return cam_p;
}

void
SensorManager::start()
{
    _start_func();
}

void
SensorManager::stop()
{
    _stop_func();
}

void
SensorManager::update()
{
    _update_func();
}
