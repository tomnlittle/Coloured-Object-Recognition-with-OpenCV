#include <iostream>
#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>

#include "libs/camera.h"

#define LOWER_SATURATION_VALUE          100
#define UPPER_SATURATION_VALUE          255
#define UPPER_GREEN                     75
#define LOWER_GREEN                     40
#define UPPER_BLUE                      127
#define LOWER_BLUE                      76
#define BORDER_THICKNESS                4
#define BLUR_AMOUNT                     0.1
#define FILTER_X                        25
#define FILTER_Y                        25

static bool run;
void command();

using namespace std;

cv::Mat getRect(cv::Mat HSV, int lowHue, int highHue, cv::Scalar Color);

int main (int argc, char** argv)
{
    cv::Mat disp, HSV;

    cv::Size size(640, 320);

    cv::Scalar colors[3];
    colors[0] = cv::Scalar(255, 0, 0);
    colors[1] = cv::Scalar(0, 255, 0);
    colors[2] = cv::Scalar(0, 0, 255);

    Camera cam0(0, 3, size);

    run = true; 
    std::thread commandsThread(command);

    while(run){
        disp = cam0.getFrame();
        
        cv::cvtColor(disp, HSV, cv::COLOR_BGR2HSV); //convert image to HSV

        cv::Mat lowRed = getRect(HSV, 0, 10, colors[2]); 
        cv::Mat highRed = getRect(HSV, 160, 179, colors[2]);
        cv::Mat blue = getRect(HSV, LOWER_BLUE, UPPER_BLUE, colors[0]);
        cv::Mat green = getRect(HSV, LOWER_GREEN, UPPER_GREEN, colors[1]);
       
        cv::addWeighted(lowRed, 1, disp, 1, 0.0, disp);
        cv::addWeighted(highRed, 1, disp, 1, 0.0, disp);
        cv::addWeighted(blue, 1, disp, 1, 0.0, disp);
        cv::addWeighted(green, 1, disp, 1, 0.0, disp);
        
        cv::imshow("display", disp);

        cv::waitKey(UPDATE_FREQUENCY);
    }
    commandsThread.join();
    return 0;
}

void command(){
    string hold_value = " ";
    printf("Press any key to close\n");
    cin >> hold_value;	
    run = false;
    return;
}

cv::Mat getRect(cv::Mat HSV, int lowHue, int highHue, cv::Scalar Color){
    cv::Mat range;
    cv::Mat disp(HSV.rows, HSV.cols, CV_8UC3, cv::Scalar(0,0,0));
    std::vector<std::vector<cv::Point>> contours;
    std::vector<cv::Vec4i> hierarchy;

    cv::Size filterSize(FILTER_X,FILTER_Y); // size of the filter block
    
    cv::inRange(HSV, cv::Scalar(lowHue, LOWER_SATURATION_VALUE, LOWER_SATURATION_VALUE), 
                                cv::Scalar(highHue, UPPER_SATURATION_VALUE, UPPER_SATURATION_VALUE), range);
    
    cv::erode(range, range, cv::getStructuringElement(cv::MORPH_ELLIPSE, filterSize));
    cv::dilate(range, range, cv::getStructuringElement(cv::MORPH_ELLIPSE, filterSize)); 
    cv::dilate(range, range, cv::getStructuringElement(cv::MORPH_ELLIPSE, filterSize)); 
    cv::erode(range, range, cv::getStructuringElement(cv::MORPH_ELLIPSE, filterSize));

    cv::findContours(range, contours, cv::RETR_TREE, CV_CHAIN_APPROX_SIMPLE);

    std::vector<vector<cv::Point>> contours_poly(contours.size());
    std::vector<cv::Rect> boundRect(contours.size());

    for(int i = 0; i < contours.size(); i++)
    { 
        cv::approxPolyDP(cv::Mat(contours[i]), contours_poly[i], 3, true );
        boundRect[i] = cv::boundingRect(cv::Mat(contours_poly[i]));
        cv::rectangle(disp, boundRect[i].tl(), boundRect[i].br(), Color, BORDER_THICKNESS, 8, 0 );
    }
    
    return disp;
}


