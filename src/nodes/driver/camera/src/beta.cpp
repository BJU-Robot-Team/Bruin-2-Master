#include "opencv2/opencv.hpp"
#include "opencv2/imgcodecs.hpp"
#include "opencv2/highgui/highgui.hpp"
#include "opencv2/imgproc/imgproc.hpp"

#include <iostream>
#include <vector>

using namespace cv;
using namespace std;

int main(int, char**)

{

    VideoCapture cap(0);                                                        //Open the default camera
    Mat bnwf, linimg;                                                           //Global image variables to return

    if(!cap.isOpened())                                                         //Check if we succeeded
      {
        return -1;
      }

    for(;;)
      {
        Mat image, dst, cdst, cimg, bnw;                                        //Image transformation variables

        cap>>image;                                                             //Get a new frame from camera
        medianBlur(image,image,5);                                              //Smooth out the image
        flip(image,image,-1);                                                   //Flip image upside down because camera is mounted upside down--uncomment for final code
        bnw = image.clone();                                                    //Create a copy of original image
        Canny(image, dst, 90, 200, 3);                                          //Lowthresh originally set at 50

        //Debugging between ObstacleDetect and HoughLines
        //Set if==0 to debug ObstacleDetect
        //Set if==1 to debug HoughLines
        //Comment out if, else, and endif for running program in Bruin 2

        //#if 1--uncomment for debugging purposes

        //ObstacleDetect
        //Set up loop to scan image for GREEN > BLUE && GREEN > RED
        //Then create BLACK and WHITE image from scan results
        //WHITE is TRUE, BLACK is FALSE

        for(int y = 0; y < image.rows; y++)                                     //Set up number of rows to loop through
        {
            for(int x = 0; x < image.cols; x++)                                 //Set up number of columbs to loop through
            {
                if((image.at<cv::Vec3b>(y,x)[1]-10)>image.at<cv::Vec3b>(y,x)[0] && (image.at<cv::Vec3b>(y,x)[1]-10)>image.at<cv::Vec3b>(y,x)[2])
                //Compare Green greater than Red and Green greater than Blue
                {
                  bnw.at<cv::Vec3b>(y,x)[0]=255;                                //If GREEN greater than RED and BLUE, make it WHITE (TRUE)
                  bnw.at<cv::Vec3b>(y,x)[1]=255;
                  bnw.at<cv::Vec3b>(y,x)[2]=255;
                }
                else
                {
                  bnw.at<cv::Vec3b>(y,x)[0]=0;                                  //If RED or BLUE greater than GREEN, make it BLACK (FALSE)
                  bnw.at<cv::Vec3b>(y,x)[1]=0;
                  bnw.at<cv::Vec3b>(y,x)[2]=0;
                }
            }
        }

        //flip(bnw,bnw,1);                                                      //Flip image horizontally since scan flips image horizontally--uncomment to debug ObstacleDetect
                                                                                //Note: Scan was flipping horizontally at some point, maybe due to debug code--leave in case of debug
        medianBlur(bnw,bnw,5);                                                  //Smooth out the noise
        cvtColor(bnw, bnwf, COLOR_BGR2GRAY);                                    //Convert from BGR color to grayscale--Pass processed image to global variable
        //imshow("B&W Image",bnwf);                                             //Display B&W image--uncomment to debug ObstacleDetect


        //#else--uncomment for debugging purposes

        //HoughLines
        cvtColor(dst, cdst, COLOR_GRAY2BGR);

        vector<Vec4i> lines;
        HoughLinesP(dst, lines, 1, CV_PI/180, 50, 50, 10 );

        //Line drawing function--draws red lines where lines detected in image
        for( size_t i = 0; i < lines.size(); i++ )
        {
            Vec4i l = lines[i];
            line( bnw, Point(l[0], l[1]), Point(l[2], l[3]), Scalar(0,0,255), 3, LINE_AA); //Draws red lines on image
        }

        flip(bnw,bnw,0);                                                        //Flip image because it's backwards and upside down
        linimg = bnw;                                                           //Pass processed image to global variable
        //imshow("Line Image", linimg);                                         //Display final processed image--uncomment to debug LineDetect

        //#endif--uncomment for debugging purposes

        if(waitKey(30) >= 0) break;                                             //30 ms delay

      }

    // the camera will be deinitialized automatically in VideoCapture destructor
    return 0;                                                                   //Kill program
}
