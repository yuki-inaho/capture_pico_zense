#include "PicoZenseSensorClass.h"


using namespace std;

void PicoZenseSensor::initialize(int32_t deviceIndex,  int _image_width, int _image_height, int _image_fps){

    PsReturnStatus status;

    // initialize check
    status = PsInitialize();
    if (status != PsReturnStatus::PsRetOK)
    {
        cout << "PsInitialize failed!" << endl;
        return;
    }

    // device count
    int32_t deviceCount = 0;
    status = PsGetDeviceCount(&deviceCount);
    if (status != PsReturnStatus::PsRetOK)
    {
        cout << "PsGetDeviceCount failed!" << endl;
        return;
    }
    cout << "Get device count: " << deviceCount << endl;

    //Set the Depth Range to Near through PsSetDepthRange interface

    //status = PsSetDepthRange(deviceIndex, PsNearRange);
    status = PsSetDepthRange(deviceIndex, PsMidRange);
    //status = PsSetDepthRange(deviceIndex, PsXNearRange);
    if (status != PsReturnStatus::PsRetOK)
        cout << "PsSetDepthRange failed!" << endl;
    else
        cout << "Set Depth Range to Near" << endl;

    // camera open
    status = PsOpenDevice(deviceIndex);
    if (status != PsReturnStatus::PsRetOK)
    {
        cout << "OpenDevice failed!" << endl;
        return;
    }

    int32_t serial_len = 100;
    char serial_buff[serial_len];
    status = PsGetProperty(deviceIndex, PsPropertySN_Str, serial_buff, &serial_len);
    serial_number = serial_buff;

    int32_t firm_len = 100;
    char firm_buff[firm_len];
    status = PsGetProperty(deviceIndex, PsPropertyFWVer_Str, firm_buff, &firm_len);
    name = firm_buff;

    //Set PixelFormat as PsPixelFormatBGR888 for opencv display
    PsSetColorPixelFormat(deviceIndex, PsPixelFormatBGR888);

    /// RGB-D Mode -> depth img size = 640*360
    /// Depth Only Mode -> depth img size = 640*480

    /// Mode
//    int32_t dataMode = PsDepthAndRGB_30;
//    int32_t dataMode = PsDepthAndIRAndRGB_30;
    int32_t dataMode = PsDepthAndIR_15_RGB_30;
    PsSetDataMode(deviceIndex, (PsDataMode)dataMode);

    /// distortion
//  bool f_bDistortionCorrection = false;
    bool f_bDistortionCorrection = true;
    PsSetDepthDistortionCorrectionEnabled(deviceIndex, f_bDistortionCorrection);
    PsSetIrDistortionCorrectionEnabled(deviceIndex, f_bDistortionCorrection);
    PsSetRGBDistortionCorrectionEnabled(deviceIndex, f_bDistortionCorrection);

    /// Smooth Filter
    bool f_bFilter = false;
    //bool f_bFilter = true;
    PsSetFilter(deviceIndex, PsSmoothingFilter, f_bFilter);

    /// Dust Filter
//   bool f_bDustFilter = true;
//   bool f_bDustFilter = false;
//    PsSetDustFilterEnabled(deviceIndex, f_bDustFilter);

    /// RGB resolution
    PsFrameMode frameMode;
    frameMode.resolutionWidth = _image_width;
    frameMode.resolutionHeight = _image_height;
    PsSetFrameMode(deviceIndex, PsRGBFrame, &frameMode);

    /// Mapped FLAG
//    bool f_bMappedDepth = true;
//    status = PsSetMapperEnabledRGBToDepth(deviceIndex, f_bMappedDepth);
    bool f_bMappedRGB = true;
    status = PsSetMapperEnabledDepthToRGB(deviceIndex, f_bMappedRGB);

    if (status != PsRetOK)
    {
        cout << "PsSetMapperEnabledRGBToDepth failed!" << endl;
        return;
    }

    image_width = _image_width;
    image_height = _image_height;
}


void
PicoZenseSensor::update(int32_t deviceIndex) {

    PsReturnStatus status;
    status = PsReadNextFrame(deviceIndex);

    // 起動直後が不安定なためif文によるNULL判定

    /// RGB
    PsFrame rgbFrame = {0};
    PsGetFrame(deviceIndex, PsRGBFrame, &rgbFrame);
    if (rgbFrame.pFrameData != NULL) {
        cv::Mat _color_img = cv::Mat(rgbFrame.height, rgbFrame.width, CV_8UC3, rgbFrame.pFrameData);
        color_img = _color_img.clone();
    }

    PsGetFrame(deviceIndex, PsRGBFrame, &rgbFrame);

    //PsImuWithParams temp_param;
    //PsGetImuWithParams(deviceIndex, &temp_param);
    //cout << "temp:" << temp_param.temp << endl;

    /// RGB->Depth Mapped
    PsFrame mappedRGBFrame = {0};
    PsGetFrame(deviceIndex, PsMappedRGBFrame, &mappedRGBFrame);
    if (mappedRGBFrame.pFrameData != NULL)
    {
        cv::Mat _color_img = cv::Mat(mappedRGBFrame.height, mappedRGBFrame.width, CV_8UC3, mappedRGBFrame.pFrameData);
        color_img = _color_img.clone();
    }

    /// Depth
    PsFrame depthFrame = {0};
    PsGetFrame(deviceIndex, PsDepthFrame, &depthFrame);
    if (depthFrame.pFrameData != NULL) {
        cv::Mat _depth_img = cv::Mat(depthFrame.height, depthFrame.width, CV_16UC1, depthFrame.pFrameData);
        depth_img = _depth_img.clone();
    }

    /// IR
    PsFrame irFrame = {0};
    PsGetFrame(deviceIndex, PsIRFrame, &irFrame);
    if (irFrame.pFrameData != NULL) {
        //cv::Mat _ir_img = cv::Mat(irFrame.height, irFrame.width, CV_16UC1, irFrame.pFrameData);
        ir_img = cv::Mat(irFrame.height, irFrame.width, CV_16UC1, irFrame.pFrameData);
        //_ir_img.convertTo(ir_img, CV_8U, 255.0 / 3840);
        //cv::Mat ir_img = cv::Mat(irFrame.height, irFrame.width, CV_16UC1, irFrame.pFrameData);
    }

    PsFrame confFrame = {0};
    PsGetFrame(deviceIndex, PsConfidenceFrame, &confFrame);
    if (confFrame.pFrameData != NULL) {
        //cv::Mat _ir_img = cv::Mat(irFrame.height, irFrame.width, CV_16UC1, irFrame.pFrameData);
        cv::Mat conf = cv::Mat(confFrame.height, confFrame.width, CV_16UC1, confFrame.pFrameData);
    }

}

void
PicoZenseSensor::start() {
    cout << "Capture Start " << endl;
}

void
PicoZenseSensor::stop(int32_t deviceIndex){

    PsReturnStatus status;
    status = PsCloseDevice(deviceIndex);
    cout << "CloseDevice status: " << status << endl;

    status = PsShutdown();
    cout << "Shutdown status: " << status << endl;
}


cv::Mat
PicoZenseSensor::getRGBImage(){
    return color_img;
}

cv::Mat
PicoZenseSensor::getDepthImage(){
    return depth_img;
}

cv::Mat
PicoZenseSensor::getIRImage(){
    return ir_img;
}

cv::Mat
PicoZenseSensor::getColorizedDepthImage(){
    cv::Mat tmp, color_tmp;
    uint32_t slope = 1450;
    depth_img.convertTo(tmp, CV_8U, 255.0 / slope);
    cv::applyColorMap(tmp, color_tmp, cv::COLORMAP_JET);
    colorized_depth_img = color_tmp.clone();
    return colorized_depth_img;
}

CameraParameter
PicoZenseSensor::getCameraParameter(int32_t deviceIndex){

    PsReturnStatus status;
    PsCameraParameters cameraParameters;
    status = PsGetCameraParameters(deviceIndex, PsDepthSensor, &cameraParameters);

    camera_param.fx = cameraParameters.fx;
    camera_param.fy = cameraParameters.fy;
    camera_param.cx = cameraParameters.cx;
    camera_param.cy = cameraParameters.cy;
    camera_param.image_width = image_width;
    camera_param.image_height = image_height;
    camera_param.base_line = 0.0;
    camera_param.type = SensorType::TOF;

//    cout << "Depth Camera Intinsic: " << endl;
//    cout << "Fx: " << camera_param.fx << endl;
//    cout << "Cx: " << camera_param.cx << endl;
//    cout << "Fy: " << camera_param.fy << endl;
//    cout << "Cy: " << camera_param.cy << endl;

    return camera_param;
}
