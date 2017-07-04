#include "messenger.hpp"
#include <websocketpp/common/thread.hpp>
#include <websocketpp/common/memory.hpp>
#include <cstdlib>
#include <sstream>
#include "opencv2/imgproc/imgproc.hpp"
#include "opencv2/imgcodecs.hpp"
#include <opencv2/highgui.hpp>
#include <opencv2/core.hpp>
#include <opencv2/aruco.hpp>
#include <iostream>
#include <math.h>
#include <unistd.h>
#include <string>

// one can set the basic pwms of the running motors and also set the control loop constants
#define PWM 100
#define KP 0.3
using namespace std;
using namespace cv;

// Declaring the keys, etc required for aruco detection. Just keep these, if you have run aruco create programs, you can understand these keys
namespace {
const char* about = "Basic marker detection";
const char* keys = "{d        |       | dictionary: DICT_4X4_50=0, DICT_4X4_100=1, DICT_4X4_250=2,"
                   "DICT_4X4_1000=3, DICT_5X5_50=4, DICT_5X5_100=5, DICT_5X5_250=6, DICT_5X5_1000=7, "
                   "DICT_6X6_50=8, DICT_6X6_100=9, DICT_6X6_250=10, DICT_6X6_1000=11, DICT_7X7_50=12,"
                   "DICT_7X7_100=13, DICT_7X7_250=14, DICT_7X7_1000=15, DICT_ARUCO_ORIGINAL = 16, DICT_6X6_10=17}"
                   "{v        |       | Input from video file, if ommited, input comes from camera }"
                   "{ci       | 0     | Camera id if input doesnt come from video (-v) }"
                   "{c        |       | Camera intrinsic parameters. Needed for camera pose }"
                   "{l        | 0.1   | Marker side lenght (in meters). Needed for correct scale in camera pose }"
                   "{dp       |       | File of marker detector parameters }"
                   "{r        |       | show rejected candidates too }";
}

static bool readDetectorParameters(string filename, Ptr<aruco::DetectorParameters>& params)
{
    FileStorage fs(filename, FileStorage::READ);
    if (!fs.isOpened())
        return false;
    fs["adaptiveThreshWinSizeMin"] >> params->adaptiveThreshWinSizeMin;
    fs["adaptiveThreshWinSizeMax"] >> params->adaptiveThreshWinSizeMax;
    fs["adaptiveThreshWinSizeStep"] >> params->adaptiveThreshWinSizeStep;
    fs["adaptiveThreshConstant"] >> params->adaptiveThreshConstant;
    fs["minMarkerPerimeterRate"] >> params->minMarkerPerimeterRate;
    fs["maxMarkerPerimeterRate"] >> params->maxMarkerPerimeterRate;
    fs["polygonalApproxAccuracyRate"] >> params->polygonalApproxAccuracyRate;
    fs["minCornerDistanceRate"] >> params->minCornerDistanceRate;
    fs["minDistanceToBorder"] >> params->minDistanceToBorder;
    fs["minMarkerDistanceRate"] >> params->minMarkerDistanceRate;
    fs["doCornerRefinement"] >> params->doCornerRefinement;
    fs["cornerRefinementWinSize"] >> params->cornerRefinementWinSize;
    fs["cornerRefinementMaxIterations"] >> params->cornerRefinementMaxIterations;
    fs["cornerRefinementMinAccuracy"] >> params->cornerRefinementMinAccuracy;
    fs["markerBorderBits"] >> params->markerBorderBits;
    fs["perspectiveRemovePixelPerCell"] >> params->perspectiveRemovePixelPerCell;
    fs["perspectiveRemoveIgnoredMarginPerCell"] >> params->perspectiveRemoveIgnoredMarginPerCell;
    fs["maxErroneousBitsInBorderRate"] >> params->maxErroneousBitsInBorderRate;
    fs["minOtsuStdDev"] >> params->minOtsuStdDev;
    fs["errorCorrectionRate"] >> params->errorCorrectionRate;
    return true;
}

// function for having some time delay in the code , can be used when we want a time delay
int msleep(unsigned long milisec)
{
    struct timespec req = { 0 };
    time_t sec = (int)(milisec / 1000);
    milisec = milisec - (sec * 1000);
    req.tv_sec = sec;
    req.tv_nsec = milisec * 1000000L;
    while (nanosleep(&req, &req) == -1)
        continue;
    return 1;
}

//////////////////////////////////////////////////////

int main(int argc, char* argv[])
{

    //declaring variables which will be used later during aruco marker detection
    CommandLineParser parser(argc, argv, keys);
    parser.about(about);

    Ptr<aruco::Dictionary> dictionary = aruco::generateCustomDictionary(10, 6);
    Ptr<aruco::DetectorParameters> detectorParams = aruco::DetectorParameters::create();
    detectorParams->doCornerRefinement = true; // do corner refinement in markers

    ///////////////////////////////file read stuff////////

    // reading stored co-ordinates from a yml file
    FileStorage read_yml0("path_0.yml", FileStorage::READ);

    // i have stored the coordinates under the node : 'features', so accessing them and declaring FileNodeIterator
    FileNode features = read_yml0["features"];
    FileNodeIterator it = features.begin(), it_end = features.end();
    int path00_read = 0, path00_readnew = 0, loop3 = 0;
    int path_size = 0;
    int width = 40;
    float mag, temp, length = 0;

    // making vectors to store these read coordinates
    std::vector<int> x_read , y_read ;
    std::vector<int> ximpose, yimpose, x00, y00, xloop00, yloop00;

    // iterate through a sequence using FileNodeIterator
    for (; it != it_end; ++it, path00_read++) {

        //cout << "feature #" << path00_read << ": ";
        //cout << "x=" << (int)(*it)["x"];
        //cout << "y=" << (int)(*it)["y"];

        // storing the accessed coordinates in vectors x_read , y_read  and declaring the vector variable x00, y00 to be of the samesize
        x_read .push_back(int((*it)["x"]));
        y_read .push_back(int((*it)["y"]));
        x00.push_back(0);
        y00.push_back(0);

        //cout << endl<< path00_read << endl;
    }

    ximpose = x_read ;
    yimpose = y_read ;

    path00_readnew = path00_read;
    //cout << endl<< endl;

    int waitTime;

    // If error, change cap(0) to cap(1)
    VideoCapture cap(0);

    if (!cap.isOpened()) // if not success, exit program
    {
        //cout << "Cannot open the video cam" << endl;
        return -1;
    }

    // setting the resolution of the camera frames
    cap.set(CV_CAP_PROP_FRAME_WIDTH, 1280);
    cap.set(CV_CAP_PROP_FRAME_HEIGHT, 720);
    Mat image = Mat::zeros(720, 1280, CV_8UC3);

    ///////////////////////////////send message

    char text[255];
    int loop1 = 0;

    // declaring some variables which will be used later
    float mx0, my0, mx1, my1, mxlast, mylast;
    float yoo, xoo, yol, xol;

    //websocket code
    bool done = false;
    std::string input;

    // we are using websockets for sending information or commands
    // declaring a websocket endpoint (refer messenger.hpp for more info), which will be used in sending commands
    websocket_endpoint endpoint;
    int id0;

    // declaring the url and corresponding id of our r-pi to send information to
    std::string uri0 = "ws://chairbot00.local:8000";
    id0 = endpoint.connect(uri0);

    ///cv
    char Diff[50];
    char varpwm[50];
    char cords[255];
    char Angle[5];

    char y, n;
    double totalTime = 0;
    int totalIterations = 0, mkv0 = 0;
    float angle;
    int path_window = 500;

    ///cv
    // declaring some strings, these will be used while sending commands to arduino, as arduino recieves 3 integers for now in a format like this {a,b,c}
    // a means fwd, bwd, right, left, and b,c represents the rightpwm and leftpwm
    string cmd1("1");
    string cmd2("2");
    string cmd3("3");
    string cmd4("4");
    string cmd5("5");
    string comma(",");
    string endline("\n");
    char pwm_right[15];
    char pwm_left[15];
    double tick;
    int xdifflast = 0;
    string start;

    // declaring videowriter object for saving the output video in a specific format
    VideoWriter video("out.avi", CV_FOURCC('M', 'J', 'P', 'G'), 10, Size(1280, 720), true);

    //	cout<<"start"<<endl;
    //	std::getline(std::cin, start);

    // declaring images to store captured frames
    Mat myimage; //= Mat::zeros( image.size(), CV_8UC3 );
    Mat cvimage; //= Mat::zeros( image.size(), CV_8UC3 );

    while (cap.isOpened()) {
        double tick = (double)getTickCount();

        //////////CV//////////

        // copying read frames to our image matrices
        bool b1Success = cap.read(myimage);
        bool b2Success = cap.read(cvimage);

        // declaring local matrix of same size for visualizing current loop stuff
        Mat dummyimage = Mat::zeros(myimage.size(), CV_8UC3);

        // variables and booleans declared for our understanding and accessing different detected markers.
        int number0;
        bool yes0 = false;

        // parameters declared used in aruco marker detection
        vector<int> ids;
        vector<vector<Point2f> > corners, rejected;
        vector<Vec3d> rvecs, tvecs;

        // detect markers and estimate pose
        aruco::detectMarkers(cvimage, dictionary, corners, ids, detectorParams, rejected);

        // draw results
        if (ids.size() > 0) {
            aruco::drawDetectedMarkers(cvimage, corners, ids, Scalar(255, 0, 0));
            aruco::drawDetectedMarkers(dummyimage, corners, ids, Scalar(255, 0, 0));
        }

        // setting values of declared variables and booleans corresponding to different markers
        for (int l = 0; l < ids.size(); l++) {
            if (ids[l] == 0) {
                yes0 = true;
                number0 = l;
            }
        }

        //////////////-------------------CHAIRBOT00-------------------////////////////

        if ((ids.size() > 0) && (yes0)) {

            // declaring variables storing coordinates of the corners of the detected marker in this loop
            int xeint, yeint;
            float xe, x0, x1, x2, x3, x30m, x12m, ye, y0, y1, y2, y3, y30m, y12m, abovex, abovey;
            Point2f c0, c1, c2, c3, midpt, above;

            // will print yes if this marker corresponding to chairbot00 is detected.
            // accessing and saving all the detected corners, finding a mid point as well
            //cout << "------------------yes" << endl;
            c0 = corners[number0][0];
            c1 = corners[number0][1];
            c2 = corners[number0][2];
            c3 = corners[number0][3];
            midpt = (c0 + c1 + c2 + c3) / 4;

            x0 = c0.x;
            x1 = c1.x;
            x2 = c2.x;
            x3 = c3.x;
            xe = (midpt.x);
            xeint = int(xe);

            y0 = c0.y;
            y1 = c1.y;
            y2 = c2.y;
            y3 = c3.y;
            ye = (midpt.y);
            yeint = int(ye);

            //x30m means x-coord of mid pt. of corners 3 and 0, similarly x12m means of between 1 and 2.
            x30m = (x3 + x0) / 2;
            x12m = (x1 + x2) / 2;
            y30m = (y3 + y0) / 2;
            y12m = (y1 + y2) / 2;

            // drawing two polygons visualizing the chair
            ///////////////////////////////////////////drawing polygon

            vector<Point> pointpoly;
            pointpoly.push_back(Point(x0, y0));
            pointpoly.push_back(Point(x1, y1));
            pointpoly.push_back(Point(x12m, y12m));
            pointpoly.push_back(Point(x30m, y30m));

            const Point* pts = (const Point*)Mat(pointpoly).data;
            int npts = 4;

            //front poly
            polylines(dummyimage, &pts, &npts, 1,
                true, // draw closed contour (i.e. joint end to start)
                Scalar(0, 255, 255), // colour RGB ordering (here = green)
                3, // line thickness
                CV_AA, 0);

            ///////////////////////////////////////////drawing polygon
            vector<Point> pointpoly2;
            pointpoly2.push_back(Point(x12m, y12m));
            pointpoly2.push_back(Point(x2, y2));
            pointpoly2.push_back(Point(x3, y3));
            pointpoly2.push_back(Point(x30m, y30m));

            const Point* pts2 = (const Point*)Mat(pointpoly2).data;
            int npts2 = 4;

            //backpoly
            polylines(dummyimage, &pts2, &npts2, 1,
                true, // draw closed contour (i.e. joint end to start)
                Scalar(255, 255, 0), // colour RGB ordering (here = green)
                3, // line thickness
                CV_AA, 0);

            ///////////////////////////////////polygon...... :) //////

            // to find the orientation as at which angle is the chair oriented at, declaring the point 'above'
            above.x = (x3 + 50);
            above.y = (y3);

            //y1=corners[0][1].y;
            Point2f Point0(x0, y0), Point3(x3, y3);
            Point2i Pointe(xeint, yeint);

            // finding out angles based on the detected markers position relative to x-y axes
            // corrected angle
            if (x0 >= x3 && y3 >= y0) { //Quad1
                angle = (90 - (atan((x0 - x3) / (y3 - y0))) * (180 / 3.414));
                if (angle < 45) {
                    angle = ((atan((y3 - y0) / (x0 - x3))) * (180 / 3.414));
                }
            }
            else if (x0 <= x3 && y3 >= y0) { //Quad2
                angle = (90 + (atan((x3 - x0) / (y3 - y0))) * (180 / 3.414));
                if (angle > 135) {
                    angle = (180 - ((atan((y3 - y0) / (x3 - x0))) * 180 / 3.414));
                }
            }
            else if (x0 < x3 && y3 <= y0) { //Quad3
                angle = (-180 + ((atan((y0 - y3) / (x3 - x0))) * 180 / 3.414));
                if (angle > -135) {
                    angle = (-90 - ((atan((x3 - x0) / (y0 - y3))) * 180 / 3.414));
                }
            }
            else if (x0 >= x3 && y3 < y0) { //Quad4
                angle = (((atan((y3 - y0) / (x0 - x3))) * 180 / 3.414));
                if (angle < -45) {
                    angle = (-90 + ((atan((x0 - x3) / (y0 - y3))) * 180 / 3.414));
                }
            }

// for our visualization drawing arrowed lines on our local image matrice "dummyimage", which is declared fresh each time
// this loop is run

            arrowedLine(dummyimage, Point3, Point0, Scalar(0, 0, 255), 2, 8, 0, 0.1);
            arrowedLine(dummyimage, Point3, above, Scalar(0, 0, 255), 2, 8, 0, 0.1);

// calculating distance between two corners, very important so as to be independent of camera height, or marker's size

              float distance = norm(Point0 - Point3);

// projecting location and angles of the detected marker on the image

            sprintf(cords, "Chairbot %d is at (%d,%d)", id0, xeint, yeint);
            sprintf(Angle, "  A   %0.1f,", angle);
            putText(dummyimage, cords, Point(50, 75), 1, 2, Scalar(255, 255, 255), 0.5);
            putText(dummyimage, Angle, Point(25, 50), 1, 2, Scalar(255, 255, 255), 0.5);
            //circles
// since I am using circles at the corners for controlling the chair, declaring the radius here

            float radius = ((distance / 4));
            float radiussq = (radius * radius);
            float radiusee = (distance / 4) * (1 + 2 * (sqrt(2)));
            float radiussqee = radiusee * radiusee;
            float Widthline = ((distance * 2) / 4);

// for our better visualization of all the corners and circles drawing circles and text onto the image

            circle(dummyimage, Point(x0, y0), (radius), cv::Scalar(0, 100, 255), 1);
            circle(dummyimage, Point(x1, y1), (radius), cv::Scalar(0, 100, 255), 1);
            circle(dummyimage, Point(x2, y2), (radius), cv::Scalar(255, 100, 0), 1);
            circle(dummyimage, Point(x3, y3), (radius), cv::Scalar(255, 100, 0), 1);
            circle(dummyimage, Point(xe, ye), (radiusee), cv::Scalar(255, 255, 255), 1);

            putText(dummyimage, "0", Point(x0, y0), 1, 2, Scalar(255, 0, 255), 1.0);
            putText(dummyimage, "1", Point(x1, y1), 1, 2, Scalar(255, 0, 255), 1.0);
            putText(dummyimage, "2", Point(x2, y2), 1, 2, Scalar(255, 0, 255), 1.0);
            putText(dummyimage, "3", Point(x3, y3), 1, 2, Scalar(255, 0, 255), 1.0);

            /////////////// play here /////////////////////////
// for sending different commands to the r-pi , decaring more parameters to be used.

            int ydiff;
            int xdiff, pwmr, pwml;
            int circle0 = 0, circle1 = 0, circle2 = 0, circle3 = 0, circlee = 0;

// circle0 means no. of pixxels of the path lies inside the corner0 circle, similarly circle1-3also,
// circlee means no. of pixxels of the path which lies inside the marker.
            ////////////////////////super-imposing///////////////////

// mkv is the loop no. representing how many time a particular loop for a marker has run, for chairbot00 it's mkv0, for chairbot01 it'll be mkv1, etc

            if (mkv0 == 0) {// this loop will run only once at the beginning when mkv0 is == 0;
                int loop2 = 0;
                for (; loop2 < (path00_readnew - 1); loop2++) {
                    x00.at(loop2) = (ximpose.at(loop2) - (ximpose.at(0) - xeint));
                    y00.at(loop2) = (yimpose.at(loop2) - (yimpose.at(0) - yeint));
                    x00.at(loop2 + 1) = (ximpose.at(loop2 + 1) - (ximpose.at(0) - xeint));
                    y00.at(loop2 + 1) = (yimpose.at(loop2 + 1) - (yimpose.at(0) - yeint));
                    line(image, Point2i(x00.at(loop2), y00.at(loop2)), Point2i(x00.at(loop2 + 1), y00.at(loop2 + 1)), Scalar(255, 0, 0));
                }
                mxlast = x00.at(loop2);
                mylast = y00.at(loop2);
                sprintf(text, "THE END");
                putText(image, "THE END", Point(mxlast, mylast), 1, 2, Scalar(255, 0, 255), 1.0);

                yoo = y00.at(0);
                xoo = x00.at(0);
                yol = y00.at(path00_readnew - 1);
                xol = x00.at(path00_readnew - 1);

                ////////////////////thickening/////////////////////
// Since till now for the path we were using only coordinates, it is a very thin line, now by this command I am thickening the path
// by adding some more coordinates around our path, so as to get better hold on the control. So i'll be making and saving a new vector
// storing all the new coordinates, as you can

                for (; path_size < (path00_readnew - 10); path_size++) {
                    Point2f ci, cf, v, CON;
                    ci.x = (x00.at(path_size));
                    ci.y = (y00.at(path_size));
                    cf.x = (x00.at(path_size + 10));
                    cf.y = (y00.at(path_size + 10));

                    //line(perline, ci, cf, Scalar(255, 0, 0));

                    v.x = ci.x - cf.x;
                    v.y = ci.y - cf.y;
                    mag = sqrt(v.x * v.x + v.y * v.y);
                    v.x = v.x / mag;
                    v.y = v.y / mag;
                    temp = v.x;
                    v.x = -v.y;
                    v.y = temp;
                    width = Widthline;
                    for (length = width; length >= 0;) {
                        CON.x = ci.x + v.x * (length - (width / 4));
                        CON.y = ci.y + v.y * (length - (width / 4));

                        //line(perline, ci, CON, Scalar(0, 0, 255));
                        length = length - (width / 8);
                        xloop00.push_back(CON.x);
                        yloop00.push_back(CON.y);
                        loop3++;
                        //cout << loop3 << endl
                             //<< CON.x << "," << CON.y << endl;
                    }
                }
                for (path_size = 0; path_size < (loop3 - 1); path_size++) {
                    line(image, Point(xloop00.at(path_size), yloop00.at(path_size)), Point(xloop00.at(path_size + 1), yloop00.at(path_size + 1)), Scalar(0, 0, 255));
                }
                //cout << "loop3 is :" << loop3 << endl;

                ///////////thickened//////////////
                // so now we have many more coordinates and better control
            }

// when the code is running, we would be super imposing the detected path onto the first detected marker position, also
// we will be accessing a window of the path for our control. SO that will eliminate the possibility of getting confused at path intersections
// we will also be saving information about which path element was accessed by which circle(around a particular corner), so that
// we know the orientation of the bot and not following path in reverse direction

            int c0first = 0, c0last = 0, c1first = 0, c1last = 0, c2first = 0, c2last = 0, c3first = 0, c3last = 0, cefirst = 0, celast = 0;
            int loop2n = (cefirst - 200);
            if (loop2n < 0) {
                loop2n = 0;
            }
            for (; (loop2n < (path_window - 1)); loop2n++) {

                float inside0 = ((float(xloop00.at(loop2n)) - float(x0)) * (float(xloop00.at(loop2n)) - float(x0)) + (float(yloop00.at(loop2n)) - float(y0)) * (float(yloop00.at(loop2n)) - float(y0)));
                float inside1 = ((float(xloop00.at(loop2n)) - float(x1)) * (float(xloop00.at(loop2n)) - float(x1)) + (float(yloop00.at(loop2n)) - float(y1)) * (float(yloop00.at(loop2n)) - float(y1)));
                float inside2 = ((float(xloop00.at(loop2n)) - float(x2)) * (float(xloop00.at(loop2n)) - float(x2)) + (float(yloop00.at(loop2n)) - float(y2)) * (float(yloop00.at(loop2n)) - float(y2)));
                float inside3 = ((float(xloop00.at(loop2n)) - float(x3)) * (float(xloop00.at(loop2n)) - float(x3)) + (float(yloop00.at(loop2n)) - float(y3)) * (float(yloop00.at(loop2n)) - float(y3)));
                float insidee = (((float(xloop00.at(loop2n)) - float(xe)) * (float(xloop00.at(loop2n)) - float(xe))) + ((float(yloop00.at(loop2n)) - float(ye)) * (float(yloop00.at(loop2n)) - float(ye))));

                if (inside0 < radiussq) {
                    circle0++;
                    if (circle0 == 1) {
                        c0first = loop2n;
                    }
                    else if(circle0 != 1){
                        c0last = loop2n;
                    }
                }
                if (inside1 < radiussq) {
                    circle1++;
                    if (circle1 == 1) {
                        c1first = loop2n;
                    }
                    else if(circle1 != 1) {
                        c1last = loop2n;
                    }
                }
                if (inside2 < radiussq) {
                    circle2++;
                    if (circle2 == 1) {
                        c2first = loop2n;
                    }
                    else if(circle2 != 1){
                        c2last = loop2n;
                    }
                }
                if (inside3 < radiussq) {
                    circle3++;
                    if (circle3 == 1) {
                        c3first = loop2n;
                    }
                    else if(circle3 != 1){
                        c3last = loop2n;
                    }
                }
                if (insidee < (radiussqee)) {
                    circlee++;
                    if (circlee == 1) {
                        cefirst = loop2n;
                    }
                    else if(circlee != 1){
						celast = loop2n;
					}
				}
		}
            path_window = celast + 800;

// path_window is only used for making our window of the path to avoid intersections

            if (path_window > loop3) {
                path_window = (loop3 - 1);
            }
            //cout << "circle0 is" << circle0 << "---circle2 is" << circle1 << "---circlee is" << circlee << endl;

            circle(dummyimage, Point((xloop00.at(path_window)), (yloop00.at(path_window))), (radiusee), cv::Scalar(0, 255, 0), 1);
            circle(dummyimage, Point((xloop00.at(cefirst)), (yloop00.at(cefirst))), (radiusee), cv::Scalar(0, 255, 0), 1);

            bool c023 = ((c0first > c2last) && (c0first > c3last));
            bool c123 = ((c1first > c2last) && (c1first > c3last));
            bool c210 = ((c2first > c1last) && (c2first > c0last));
            bool c310 = ((c3first > c1last) && (c3first  > c0last));

            //cout << "cefirst is:" << cefirst << " and celast is:" << celast << endl;

// based on where our path is respective to robot, we will send diff. commands to arduino

		if (circlee)
			{
				if((!circle0)&&(!circle1))
				{
				//send forward
				pwmr=PWM;
				pwml=PWM;
				sprintf(Diff, "(%d,%d)", pwmr,pwml);
				sprintf(pwm_right, "%d", pwmr);
				sprintf(pwm_left, "%d", pwml);

				std::string messageef =cmd1+comma+pwm_right+comma+pwm_left+endline;
				endpoint.send(id0, messageef);
				}
				else if(circle0&&c023)
				{//send fwdleft command
				pwmr=PWM+(KP*circle0);
				pwml=PWM-(KP*circle0);
					if(pwml<0){pwml=0;}
					if(pwmr>255){pwmr=255;}
				sprintf(Diff, "(%d,%d)", pwmr,pwml);
				sprintf(pwm_right, "%d", pwmr);
				sprintf(pwm_left, "%d", pwml);
				 	 cout<<"fwdleft---------"<<circle0<<endl;
					std::string messageel =cmd1+comma+pwm_right+comma+pwm_left+endline;
				if (pwmr==255){
					std::string messageel =cmd5+comma+pwm_right+comma+pwm_left+endline;
					endpoint.send(id0, messageel);
					break;
					}
					endpoint.send(id0, messageel);
				}
				else if(circle1&&c123)
				{//send fwdright command
				pwmr=PWM-(KP*circle1);
				pwml=PWM+(KP*circle1);
					if(pwmr<0){pwmr=0;}
					if(pwml>255){pwml=255;}
				sprintf(Diff, "(%d,%d)", pwmr,pwml);
				sprintf(pwm_right, "%d", pwmr);
				sprintf(pwm_left, "%d", pwml);
				 	 cout<<"fwdright-------"<<circle1<<endl;

				std::string messageel =cmd1+comma+pwm_right+comma+pwm_left+endline;
				if (pwml==255){
					std::string messageel =cmd5+comma+pwm_right+comma+pwm_left+endline;
					endpoint.send(id0, messageel);
					break;
					}
					endpoint.send(id0, messageel);
				}
				else if((c210))
				{//send right command
				pwmr=PWM-(KP*circle2);
				pwml=PWM+(KP*circle2);
					if(pwmr<0){pwmr=0;}
					if(pwml>255){pwml=255;}
				sprintf(Diff, "(%d,%d)", pwmr,pwml);
				sprintf(pwm_right, "%d", pwmr);
				sprintf(pwm_left, "%d", pwml);
				 	 cout<<"right"<<endl;
				std::string messageel =cmd5+comma+pwm_right+comma+pwm_left+endline;
				endpoint.send(id0, messageel);
				}
				else if((c310))
				{//send left command
				pwmr=PWM+(KP*circle3);
				pwml=PWM-(KP*circle3);
					if(pwml<0){pwml=0;}
					if(pwmr>255){pwmr=255;}
				sprintf(Diff, "(%d,%d)", pwmr,pwml);
				sprintf(pwm_right, "%d", pwmr);
				sprintf(pwm_left, "%d", pwml);
				 	 cout<<"left"<<endl;
				std::string messageel =cmd5+comma+pwm_right+comma+pwm_left+endline;
				endpoint.send(id0, messageel);
				}
			}
		else
				{//send stop command
				pwmr=101;
				pwml=001;
				sprintf(Diff, "(%d,%d)", pwmr,pwml);
				sprintf(pwm_right, "%d", pwmr);
				sprintf(pwm_left, "%d", pwml);

				std::string messageel =cmd5+comma+pwm_right+comma+pwm_left+endline;
				endpoint.send(id0, messageel);
				}
		  //cout<<mkv0<<endl;

            mkv0++;
        }
        /////////////////////////////TTTTTTTHEENDDDDDD///////////////////////////////////////
        else{
			// If there was no marker detected, always sending stop command

			int xdiff, pwmr, pwml;
				sprintf(Diff, "(%d,%d)", pwmr,pwml);
				sprintf(pwm_right, "%d", pwmr);
				sprintf(pwm_left, "%d", pwml);

				std::string messagees =cmd5+comma+pwm_right+comma+pwm_left+endline;

				//endpoint.send(id0, messagees);

			//cout<<"------------------no"<<endl;
			}
        /////////////CV//////////////////
// for our visualization of the whole code while it's running we'll have the camera feed on display with our drawings(of text, circle, polygon etc)
        myimage = image + myimage + dummyimage;
        namedWindow("ShowImage", WINDOW_NORMAL);
        imshow("ShowImage", myimage); //show the frame in "ShowImage" window
        video.write(myimage);

        if (waitKey(30) == 27) //wait for 'esc' key press for 30ms. If 'esc' key is pressed, break loop
        {
            cout << "esc key is pressed by user" << endl;
            std::string messageestop = cmd5 + comma + "300" + comma + "300" + endline;

            endpoint.send(id0, messageestop);

            break;
        }
        //loop1++;

        tick = 1000 * ((double)getTickCount() - tick) / getTickFrequency();
        double freq = (1000 / tick);
        //cout << "Frequency =" << freq << " hertz"<< endl;
        //cout << "=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=" << endl;
    }

    read_yml0.release();
    return 0;

}
