/**
 *
 * This example demonstrates how to get images from the robot remotely and how
 * to display them on your screen using opencv.
 *
 * Copyright Aldebaran Robotics
 */

#include "getimages.hpp"

static string matrixToString(cv::Mat matrix)
{
    ostringstream out;

    for(int i=0; i<matrix.rows; i++)
    {
        for(int j=0; j<matrix.cols; j++)
        {
            out << matrix.at<double>(i,j) << "\t";
        }
        out << "\n";
    }
    return out.str();
}

static double computeReprojectionErrors( const vector<vector<cv::Point3f> >& objectPoints,
                                         const vector<vector<cv::Point2f> >& imagePoints,
                                         const vector<cv::Mat>& rvecs,
                                         const vector<cv::Mat>& tvecs,
                                         const cv::Mat& cameraMatrix ,
                                         const cv::Mat& distCoeffs,
                                         vector<float>& perViewErrors)
{
    vector<cv::Point2f> imagePoints2;
    int i, totalPoints = 0;
    double totalErr = 0, err;
    perViewErrors.resize(objectPoints.size());

    for( i = 0; i < (int)objectPoints.size(); ++i )
    {
        projectPoints( cv::Mat(objectPoints[i]), rvecs[i], tvecs[i], cameraMatrix,
                       distCoeffs, imagePoints2);
        err = norm(cv::Mat(imagePoints[i]), cv::Mat(imagePoints2), CV_L2);

        int n = (int)objectPoints[i].size();
        perViewErrors[i] = (float) std::sqrt(err*err/n);
        totalErr        += err*err;
        totalPoints     += n;
    }

    return std::sqrt(totalErr/totalPoints);
}

naoCamera::naoCamera(const string& robotIP)
{
    this->robotIP = robotIP;

    setProxies();
}

void naoCamera::setProxies()
{
    camProxy = new ALVideoDeviceProxy(this->robotIP);
    motProxy = new ALMotionProxy(this->robotIP);
}

void naoCamera::subscribe(string name, int cameraId=kTopCamera)
{
    unsubscribe(name);
    clientName = camProxy->subscribe(name, kVGA, kYuvColorSpace, 30);
    camProxy->setActiveCamera(clientName, cameraId);
}

void naoCamera::unsubscribe(const string name)
{
    try
    {
        camProxy->unsubscribe(name);
    }
    catch (const AL::ALError& e) { }
}

/**
void naoCamera::detectBriskFeatures()
{
    cv::Ptr<cv::FeatureDetector> detector;
    detector =  cv::FeatureDetector::create("BRISK");

    // the filename is given some path

    //cv::Mat img = imread(filename, 0);
    //CV_Assert( !img.empty() );

    //vector<cv::KeyPoint> kp;

    //detector->detect(img, kp);
}**/

void naoCamera::cameraCalibration()
{
    subscribe("test");

    cv::Size imageSize = cv::Size(640, 480);
    cv::Mat imgHeader = cv::Mat(imageSize, CV_8UC1);
    cv::namedWindow("images");

    // calibration params and variables
    bool calibrated = false;
    bool found;

    // calibration points storage
    vector<vector<cv::Point2f> > finalImagePoints;

    // board dimension and size
    cv::Size boardSize = cv::Size(8,5);
    float squareSize = 0.027;

    // fill a vector of vectors with 3d-points
    vector<vector<cv::Point3f> > objectPoints(1);
    for( int i = 0; i < boardSize.height; ++i )
        for( int j = 0; j < boardSize.width; ++j )
            objectPoints[0].push_back(cv::Point3f(float( j*squareSize ), float( i*squareSize ), 0));

    // camera matrix for intrinsic and extrinsic parameters
    cameraMatrix = cv::Mat::eye(3, 3, CV_64F);
    cameraMatrix.at<double>(0,0) = 1.0;
    distCoeffs = cv::Mat::zeros(8, 1, CV_64F);

    /** Main loop. Exit when pressing ESC.*/
    while ((char) cv::waitKey(30) != 27)
    {
        ALValue img = camProxy->getImageRemote(clientName);
        imgHeader.data = (uchar*) img[6].GetBinary();
        camProxy->releaseImage(clientName);

        vector<cv::Mat> rvecs, tvecs;
        vector<cv::Point2f> pointBuf;

        found = cv::findChessboardCorners( imgHeader, 
                                           boardSize, 
                                           pointBuf,
                                           CV_CALIB_CB_FAST_CHECK | CV_CALIB_CB_NORMALIZE_IMAGE);

        if (found)
        {
            // improve the found corners' coordinate accuracy for chessboard
            cv::cornerSubPix( imgHeader,
                              pointBuf,
                              cv::Size(11,11),
                              cv::Size(-1,-1),
                              cv::TermCriteria( CV_TERMCRIT_EPS+CV_TERMCRIT_ITER, 30, 0.1 ));

            // size of chessboard squares, compute real points in the object
            vector<vector<cv::Point2f> > imagePoints;
            imagePoints.push_back(pointBuf);

            // Draw the corners.
            cv::drawChessboardCorners( imgHeader, boardSize, cv::Mat(pointBuf), found );

            // If c is pressed, add the current chessboard to the image points
            if (cv::waitKey(30) == 'c')
            {
                finalImagePoints.push_back(pointBuf);
                objectPoints.resize(finalImagePoints.size(), objectPoints[0]);

                // Perform calibration based on old values, if possible.
                double rms;
                try
                {
                    if(calibrated)
                    {
                        // keep updating
                        rms = cv::calibrateCamera(objectPoints,
                                                  finalImagePoints,
                                                  imageSize,
                                                  cameraMatrix,
                                                  distCoeffs,
                                                  rvecs,
                                                  tvecs,
                                                  CV_CALIB_USE_INTRINSIC_GUESS|CV_CALIB_FIX_K4|CV_CALIB_FIX_K5);
                    } else {
                        // estimate parameters for the first time
                        rms = cv::calibrateCamera(objectPoints,
                                                  finalImagePoints,
                                                  imageSize,
                                                  cameraMatrix,
                                                  distCoeffs,
                                                  rvecs,
                                                  tvecs,
                                                  CV_CALIB_FIX_K4|CV_CALIB_FIX_K5);
                    }
                    cout << "Re-projection error reported by calibrateCamera: "<< rms << endl;
                } catch(cv::Exception e) {
                    // Probably due to wrong distortion coefficients (cam movement?)
                }
                calibrated = (cv::checkRange(cameraMatrix) && cv::checkRange(distCoeffs));

                vector<float> reprojErrs;
                double totalAvgErr;
                totalAvgErr = computeReprojectionErrors(objectPoints,
                                                        finalImagePoints,
                                                        rvecs,
                                                        tvecs,
                                                        cameraMatrix,
                                                        distCoeffs,
                                                        reprojErrs);
                cout << "Avg re projection error = "  << totalAvgErr << endl;

            }
        }
        cv::imshow("images", imgHeader);
    }


    while ((char) cv::waitKey(20) != 27)
    {
        ALValue img = camProxy->getImageRemote(clientName);
        imgHeader.data = (uchar*) img[6].GetBinary();
        camProxy->releaseImage(clientName);

        cv::Mat temp = imgHeader.clone();
        cv::undistort(temp, imgHeader, cameraMatrix, distCoeffs);
        cv::imshow("images", imgHeader);
    }

}

/**
  * Get images and save these, annotated with timestamp and 3d pose estimation.
  */
void naoCamera::recordDataSet()
{
    subscribe("dataset");

    ALValue jointName = "Body";
    ALValue stiffness = 1.0f;
    motProxy->stiffnessInterpolation(jointName, stiffness, 1.0f);

    sleep(2);

    motProxy->walkTo(0.1, 0.0, 0.0);

    ALValue headPitchName = "HeadPitch";
    ALValue headPitchAngle = 0.0;
    ALValue headPitchSpeed = 0.5;
    motProxy->setAngles(headPitchName, headPitchAngle, headPitchSpeed);

    bool doBreak = false;
    thread keyboardThread (bind(&naoCamera::keyboard, this, &doBreak));
    thread sweepThread (bind(&naoCamera::sweep, this, &doBreak));

    keyboardThread.join();
    sweepThread.join();

    cout << "asdasd" << endl;
}

void naoCamera::keyboard(bool *doBreak)
{
    int lastCase = 0;
    while (!*doBreak)
    {
        // basic keyboardinterface
        int c = cv::waitKey(500);
        if( c != lastCase || c == -1) {
            switch(c)
            {
            case ESC: // esc key
                motProxy->setWalkTargetVelocity(0.0, 0.0, 0.0, 1.0);
                *doBreak = true;
                break;
            case RIGHT: // right arrow
                motProxy->setWalkTargetVelocity(0.0, 0.0, -0.8, 1.0);
                break;
            case LEFT: // left arrow
                motProxy->setWalkTargetVelocity(0.0, 0.0, 0.8, 1.0);
                break;
            case UP: // up arrow
                motProxy->setWalkTargetVelocity(0.8, 0.0, 0.0, 1.0);
                break;
            case DOWN: // down arrow
                motProxy->setWalkTargetVelocity(-0.8, 0.0, 0.0, 1.0);
                break;
            default:
                motProxy->setWalkTargetVelocity(0, 0, 0, 0);
                break;
            }
        }
        this_thread::sleep(posix_time::milliseconds(10));
    }
}

void naoCamera::sweep(bool *doBreak)
{
    string clientName = this->clientName;
    cv::Size imageSize = cv::Size(640, 480);
    cv::Mat imgHeader = cv::Mat(imageSize, CV_8UC1);
    cv::namedWindow("images");

    ALValue topCamName = "CameraTop";
    int space = 1; // world coordinates
    vector<float> camPosition;
    vector<float> initialCamPosition = motProxy->getPosition(topCamName, space, true);

    time_t start = clock();
    double seconds_since_start;

    ALValue headYawName = "HeadYaw";
    ALValue headYawAngles = ALValue::array(-1.5f, 1.5f);
    ALValue headYawTimes = ALValue::array(5, 10);
    motProxy->post.angleInterpolation(headYawName, headYawAngles, headYawTimes, true);

    while(!*doBreak)
    {
        cout << "sweep" << *doBreak;

        // get imagedata, show feed
        ALValue img = camProxy->getImageRemote(clientName);
        imgHeader.data = (uchar*) img[6].GetBinary();
        camProxy->releaseImage(clientName);

        camPosition = motProxy->getPosition(topCamName, space, true);

        // determine filename
        stringstream ss;
        // find relative positionvector
        for(size_t i = 0; i < camPosition.size(); ++i)
        {
          if(i != 0)
            ss << ",";
          ss << (camPosition[i] - initialCamPosition[i]);
        }
        ss << ".png";
        string positionvec = ss.str();

        // get timestamp
        seconds_since_start =  ((clock() - start) / CLOCKS_PER_SEC);
        char filename[50];
        int length = sprintf(filename,
                             "./images/image%.4lf_%s",
                             seconds_since_start,
                             (char*)positionvec.c_str());

        cv::imshow("images", imgHeader);
        cv::imwrite(filename, imgHeader);

        this_thread::sleep(posix_time::milliseconds(30));

        // Perform sweep, get images
        bool sweep = true;
        ALValue taskList = motProxy->getTaskList();
        string angleInterpolation("angleInterpolation");
        for(int i=0; i<taskList.getSize(); i++)
        {
            string motionName = taskList[i][0];
            if(!motionName.compare(angleInterpolation))
            {
                sweep = false;
                break;
            }
        }
        if (sweep)
        {
            motProxy->post.angleInterpolation(headYawName, headYawAngles, headYawTimes, true);
        }
    }
}

/**
  * Given image, cameramatrix and distortion coefficients, undistort the image.
  */
static cv::Mat undistortImage(cv::Mat image, cv::Mat cameraMatrix, cv::Mat distCoeffs)
{
    cv::Mat temp = image.clone();
    cv::undistort(temp, image, cameraMatrix, distCoeffs);
    return image;
}

ostream& operator<<(ostream &strm, const naoCamera &naoCam)
{

    strm << matrixToString(naoCam.cameraMatrix);
    strm << "\n";
    strm << matrixToString(naoCam.distCoeffs);

    return strm;
}

int main(int argc, char* argv[])
{
    if (argc < 2)
    {
        std::cerr << "Usage 'getimages robotIp'" << std::endl;
        return 1;
    }

    const std::string robotIp(argv[1]);

    try
    {

        naoCamera naoCam (robotIp);
        //naoCam.cameraCalibration();
        //cout << naoCam;

        naoCam.recordDataSet();
    }
    catch (const AL::ALError& e)
    {
        std::cerr << "Caught exception " << e.what() << std::endl;
    }

    return 0;
}

