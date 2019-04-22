#pragma once

#include "header.h"
#include "SensorWrapper.h"

#include<memory>

typedef std::function<void(cv::Mat&)> GetRGBImageFunc;
typedef std::function<void(cv::Mat&)> GetDepthImageFunc;
typedef std::function<void(cv::Mat&)> GetColorizedDepthImageFunc;
typedef std::function<void(cv::Mat&)> GetIRImageFunc;
typedef std::function<void(CameraParameter&)> GetCameraParameterFunc;
typedef std::function<void(void)> UpdateFunc;
typedef std::function<void(void)> StartFunc;
typedef std::function<void(void)> StopFunc;


class SensorManager{
    public:
        void setIdxSerialMap(bimap_t _bm_idx2serial);
        void setSensors(std::vector<SensorWrapper> &_sensor_vec);
        void activateSensor(std::string _serial_number);
        void start();
        void stop();
        void update();
        cv::Mat getRGBImage();
        cv::Mat getDepthImage();        
        cv::Mat getIRImage();
        cv::Mat getColorizedDepthImage();        
        CameraParameter getCameraParameter();                
        
    private:
        std::string present_serial;

        int n_sensor;
        bimap_t bm_idx2serial;
        /*
        std::unordered_map<int, std::string> idx2serial;
        std::unordered_map<std::string, int> serial2idx;
        */

        GetRGBImageFunc _get_rgb_image_func;
        GetDepthImageFunc _get_depth_image_func;
        GetIRImageFunc _get_ir_image_func;
        GetColorizedDepthImageFunc _get_colorized_depth_image_func;
        GetCameraParameterFunc _get_camera_parameter_func;
        StartFunc _start_func;
        StopFunc _stop_func;
        UpdateFunc _update_func;

        std::vector<SensorWrapper> sensor_vec;
};