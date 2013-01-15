#include "inputsource.hpp"

void loadSettings(cv::Mat &cameraMatrix, cv::Mat &distortionCoeffs)
{
    const std::string config("config");
    cv::FileStorage fs (config, cv::FileStorage::READ);

    fs["cameraMatrix"] >> cameraMatrix;
    fs["distortionCoeffs"] >> distortionCoeffs;

    fs.release();
}

void saveSettings(cv::Mat &cameraMatrix, cv::Mat &distortionCoeffs)
{
    const std::string config("config");
    cv::FileStorage fs (config, cv::FileStorage::WRITE);

    fs << "cameraMatrix" << cameraMatrix;
    fs << "distortionCoeffs" << distortionCoeffs;

    fs.release();
}

/**
  *
  */
void undistortImage(cv::Mat &image, cv::Mat &cameraMatrix, cv::Mat &distortionCoeffs)
{
    if(!cameraMatrix.empty() && !distortionCoeffs.empty())
    {
        cv::Mat temp = image.clone();
        cv::undistort(temp, image, cameraMatrix, distortionCoeffs);
    }
}

//InputSource::InputSource();

FileInput::FileInput(const std::string foldername)
{
    this->foldername = foldername;
    index = 0;
    std::stringstream ss;
    ss << foldername << "/odometry.txt";
    odometryFile.open(ss.str().c_str());
}

FileInput::~FileInput()
{
    odometryFile.close();
}

Frame FileInput::getNextFrame()
{
    Frame frame;
    float x, y, z, wx, wy, wz;
    if( odometryFile >> x >> y >> z >> wx >> wy >> wz )
    {
        float parr[6] = {x,y,z,wx,wy,wz};
        std::vector<float> positionVector ( parr, parr + 6 );
        frame.camPosition = positionVector;

        std::stringstream ss;
        ss << foldername << "/image" <<  index++ << ".png";
        cv::Mat image = cv::imread(ss.str());
        frame.img = image;
    }
    return frame;
}

NaoInput::NaoInput(const std::string &robotIp)
{
    cv::Mat temp1;
    cv::Mat temp2;
    init(robotIp, "NaoInput", AL::kTopCamera, temp1, temp2);
}

NaoInput::NaoInput(const std::string &robotIp,
                   std::string name,
                   int cameraId,
                   cv::Mat &cameraMatrix,
                   cv::Mat &distortionCoeffs)
{
    init(robotIp, name, cameraId, cameraMatrix, distortionCoeffs);
}

void NaoInput::init(const std::string &robotIp,
                    std::string name,
                    int cameraId,
                    cv::Mat &cameraMatrix,
                    cv::Mat &distortionCoeffs)
{
    this->camProxy = new AL::ALVideoDeviceProxy(robotIp);
    this->motProxy = new AL::ALMotionProxy(robotIp);

    this->cameraMatrix = cameraMatrix;
    this->distortionCoeffs = distortionCoeffs;

    //// Use the initial camera position to calculate relative positions
    // space = 1;
    // this->initialCameraPosition = motProxy->getPosition(topCamName, space, true);
    this->subscribe(name, cameraId);
}

NaoInput::~NaoInput()
{
    this->unsubscribe(clientName);
}

void NaoInput::subscribe(std::string name, int cameraId=AL::kTopCamera )
{
    unsubscribe(name);
    clientName = camProxy->subscribe(name, AL::kVGA, AL::kYuvColorSpace, 30);
    std::cout << "Subscribed to cameraproxy " << name << "." << std::endl;
    camProxy->setActiveCamera(clientName, cameraId);
}

void NaoInput::unsubscribe(std::string &name)
{
    try
    {
        camProxy->unsubscribe(name);
    }
    catch (const AL::ALError& e) { }
}

Frame NaoInput::getNextFrame()
{
    Frame frame;

    std::string cameraTop = "CameraTop";
    int space = 1;

    std::vector<float> newCameraPosition = this->motProxy->getPosition(cameraTop, space, true);
    //std::vector<float> relativeCameraPosition;
    //for (int i=0; i<6 ; i++)
    //{
    //    relativeCameraPosition.push_back( newCameraPosition[i] - initialCameraPosition[i] );
    //}
    frame.camPosition = newCameraPosition;

    // get the image from camera
    cv::Mat imgHeader = cv::Mat(cv::Size(640, 480), CV_8UC1);

    AL::ALValue img = camProxy->getImageRemote(clientName);
    imgHeader.data = (uchar*) img[6].GetBinary();
    camProxy->releaseImage(clientName);

    undistortImage(imgHeader, cameraMatrix, distortionCoeffs);
    frame.img = imgHeader.clone();

    return frame;
}
