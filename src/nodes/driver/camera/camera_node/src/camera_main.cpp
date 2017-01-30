// camera_main
// V1.0 by Bill Lovegrove modified from Michael Boye's beta code
// added integration with ROS
// V1.1 added green/blue target tracking
using namespace std;

// Factors when searching for green/blue tracking target
// (green square above blue square)
#define TARGET_WIDTH 0.14           // size of blue/green squares in meters
#define TARGET_SPACING 0.26         // center to center spacing of squares
#define WIDTH_MIN 15                // squares must be at least this many pixels
#define SQUARE_TEST 30              // Difference between height and width must be less than this many pixels
#define WIDTH_TOLERANCE 0.3         // Percent error of green and blue width
#define HORIZONTAL_TOLERANCE 0.3    // Percent error of horizontal offset between blue and green compared to size
#define BASE_PIXEL_EQUIVALENT 600.0 // See spreadsheet for details
#define BLUE_THRESHOLD 10 //was 30
#define GREEN_THRESHOLD 15


#include <iostream>
#include <vector>
#include <chrono>
#include <thread>

#include "opencv2/opencv.hpp"
#include "opencv2/imgcodecs.hpp"
#include "opencv2/highgui/highgui.hpp"
#include "opencv2/imgproc/imgproc.hpp"


using namespace cv;

#include "camera_node/ros_interface.h"


int main(int argc, char **argv)

{

    bool target_found;
    float target_distance;
    float target_direction;

    cout << "Bruin-2 Camera Driver V1.1 Starting" << endl;

    //function defined in ros_interface header
    startROS(argc, argv);
    ROSInterface ros_interface;


    VideoCapture cap(0);                                                        //Open the default camera
    Mat bnwf, linimg;                                                           //Global image variables to return

    if(!cap.isOpened())                                                         //Check if we succeeded
      {
        cout << "Camera open failed." << endl;
	ROS_ERROR_STREAM("Camera open failed.");
        return -1;
      }

    while (ros_interface.isNodeRunning()) {
        Mat image, dst, cdst, cimg, igreen, iblue, bnw;                                        //Image transformation variables

        cap>>image;                                                           //Get a new frame from camera
 	//cout << "Got a new image." << endl;
        //imshow("Camera Image", image);                                         
        medianBlur(image,image,5);                                              //Smooth out the image
        flip(image,image,-1);                                                   //Flip image upside down because camera is mounted upside down--uncomment for final code
        cvtColor(image, igreen, COLOR_RGB2GRAY);                             //Create a gray scale copy of original image
        cvtColor(image, iblue, COLOR_RGB2GRAY);                             //Create a gray scale copy of original image
        Canny(image, dst, 90, 200, 3);                                          //Lowthresh originally set at 50

        //Debugging between ObstacleDetect and HoughLines
        //Set if==0 to debug ObstacleDetect
        //Set if==1 to debug HoughLines
        //Comment out if, else, and endif for running program in Bruin 2

        #if 1 //--uncomment for debugging purposes

        //ObstacleDetect
        //Set up loop to scan image for GREEN > BLUE && GREEN > RED
        //Then create BLACK and WHITE image from scan results
        //WHITE is TRUE, BLACK is FALSE

        for(int y = 0; y < image.rows; y++)                                     //Set up number of rows to loop through
        {
            for(int x = 0; x < image.cols; x++)                                 //Set up number of columbs to loop through
            {
                if((image.at<cv::Vec3b>(y,x)[1]-GREEN_THRESHOLD)>image.at<cv::Vec3b>(y,x)[0] && (image.at<cv::Vec3b>(y,x)[1]-GREEN_THRESHOLD)>image.at<cv::Vec3b>(y,x)[2])
                //Compare Green greater than Red and Green greater than Blue
                {
		  igreen.at<uchar>(y,x)=255;
                }
                else
                {
                  igreen.at<uchar>(y,x)=0;
                }
                if((image.at<cv::Vec3b>(y,x)[0]-BLUE_THRESHOLD)>image.at<cv::Vec3b>(y,x)[1] && (image.at<cv::Vec3b>(y,x)[0]-BLUE_THRESHOLD)>image.at<cv::Vec3b>(y,x)[2])
                //Compare Blue greater than Red and Blue greater than green
                {
		  iblue.at<uchar>(y,x)=255;
                }
                else
                {
                  iblue.at<uchar>(y,x)=0;
                }
            }
        }

	// Put a bounding rectangle around the contours in the image
 	//blur( bnw, bnw, Size(3,3));	
        // Create a structuring element
        int erosion_size = 6;  
        Mat element = getStructuringElement(cv::MORPH_CROSS,
              cv::Size(2 * erosion_size + 1, 2 * erosion_size + 1),
              cv::Point(erosion_size, erosion_size) );
 
        // Apply erosion or dilation on the image
        //erode(igreen, igreen,element);
        //erode(iblue, iblue,element);

 	vector<vector<Point> > g_contours, b_contours;
  	vector<Vec4i> hierarchy;

        // rectangles around green
	findContours( igreen, g_contours, hierarchy, RETR_TREE, CHAIN_APPROX_SIMPLE, Point(0, 0) );
	vector<vector<Point> > g_contours_poly( g_contours.size() );
  	vector<Rect> g_boundRect( g_contours.size() );
  	vector<Point2f>g_center( g_contours.size() );
  	vector<float>g_radius( g_contours.size() );

        for( size_t i = 0; i < g_contours.size(); i++ )
        {  approxPolyDP( Mat(g_contours[i]), g_contours_poly[i], 3, true );
           g_boundRect[i] = boundingRect( Mat(g_contours_poly[i]) );
       	   //minEnclosingCircle( (Mat)g_contours_poly[i], g_center[i], g_radius[i]);
         }
        for( size_t i = 0; i< g_contours.size(); i++ )
        {
             Scalar color = Scalar( 0, 255, 0 );
             // drawContours( image, g_contours_poly, (int)i, color, 1, 8, vector<Vec4i>(), 0, Point() );
             if ( g_boundRect[i].width > WIDTH_MIN && (abs(g_boundRect[i].width-g_boundRect[i].height) < SQUARE_TEST) ) {
		// Draw the areas that match the size and shape threshold
		rectangle( image, g_boundRect[i].tl(), g_boundRect[i].br(), color, 2, 8, 0 );
	     }
         }

	// rectangles around blue
	findContours( iblue, b_contours, hierarchy, RETR_TREE, CHAIN_APPROX_SIMPLE, Point(0, 0) );
	vector<vector<Point> > b_contours_poly( b_contours.size() );
  	vector<Rect> b_boundRect( b_contours.size() );

        for( size_t i = 0; i < b_contours.size(); i++ )
        {  approxPolyDP( Mat(b_contours[i]), b_contours_poly[i], 3, true );
           b_boundRect[i] = boundingRect( Mat(b_contours_poly[i]) );
        }
        Mat drawing = Mat::zeros( iblue.size(), CV_8UC3 );
        for( size_t i = 0; i< b_contours.size(); i++ )
        {
             Scalar color = Scalar( 255, 0, 0 );
             // drawContours( image, contours_poly, (int)i, color, 1, 8, vector<Vec4i>(), 0, Point() );
              if ( b_boundRect[i].width > WIDTH_MIN && (abs(b_boundRect[i].width-b_boundRect[i].height) < SQUARE_TEST) ) {
		// Draw the areas that match the size and shape threshold
		rectangle( image, b_boundRect[i].tl(), b_boundRect[i].br(), color, 2, 8, 0 );
	     }
         }

        target_found = false;
	target_distance = 0.0;
	target_direction = 0.0;
        for( size_t ig = 0; ig< g_contours.size(); ig++ ) {
             if ( g_boundRect[ig].width > WIDTH_MIN && (abs(g_boundRect[ig].width-g_boundRect[ig].height) < SQUARE_TEST) ) {
                // look for matching blue but only on right-sized green
    		for( size_t ib = 0; ib < b_contours.size(); ib++ ) {
			if (
			  ( abs( ((1.0*g_boundRect[ig].tl().x - b_boundRect[ib].tl().x)/g_boundRect[ig].width) ) < HORIZONTAL_TOLERANCE ) // on top of each other
			  && ( g_boundRect[ig].br().y > b_boundRect[ib].tl().y) // green on top of blue
			  && ( abs ( 1.0 - (1.0*g_boundRect[ig].width/b_boundRect[ib].width) ) < WIDTH_TOLERANCE ) // similar size
			  && ( g_boundRect[ig].width>WIDTH_MIN) && ( b_boundRect[ib].width>WIDTH_MIN) // minimum size
			) {
                          int pixels_offset;
	                  Scalar color = Scalar( 0,0,255);
			  rectangle( image, b_boundRect[ib].tl(), b_boundRect[ib].br(), color, 2, 8, 0 );
			  rectangle( image, g_boundRect[ig].tl(), g_boundRect[ig].br(), color, 2, 8, 0 );
                          target_found = true;
                          //cout << "tl xy" << b_boundRect[ib].tl().x << " " << b_boundRect[ib].tl().y << " " << image.cols << endl;
                          //cout << "br xy" << b_boundRect[ib].br().x << " " << b_boundRect[ib].br().y << " " << image.cols << endl;
			  pixels_offset = ( b_boundRect[ib].br().x - (image.cols/2) + b_boundRect[ib].width/2 );
			  target_direction = atan(pixels_offset/BASE_PIXEL_EQUIVALENT);
			  //cout << "OK " << target_direction << " = " << target_direction*180/3.14159 << " deg" << endl;
			  target_distance = TARGET_SPACING * BASE_PIXEL_EQUIVALENT / (g_boundRect[ig].tl().y-b_boundRect[ib].tl().y) - 1;
			}
		}
            }
	}

        namedWindow( "Rectangles", WINDOW_NORMAL);
	imshow( "Rectangles", image);
        resizeWindow("Rectangles", 600,337);

        //flip(bnw,bnw,1);                                                      //Flip image horizontally since scan flips image horizontally--uncomment to debug ObstacleDetect
                                                                                //Note: Scan was flipping horizontally at some point, maybe due to debug code--leave in case of debug
        //medianBlur(bnw,bnw,5);                                                  //Smooth out the noise
        //cvtColor(bnw, bnwf, COLOR_BGR2GRAY);                                    //Convert from BGR color to grayscale--Pass processed image to global variable
        //imshow("B&W Image",bnwf);                                             //Display B&W image--uncomment to debug ObstacleDetect
        //imshow( "bnw", bnw);

        #else //--uncomment for debugging purposes

/* Hugh lines out for new
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
        imshow("Line Image", linimg);                                         //Display final processed image--uncomment to debug LineDetect
*/

        #endif //--uncomment for debugging purposes

        ros_interface.publishMessages(target_found, target_direction, target_distance);
        // Sleep between images
	waitKey(250);
        //this_thread::sleep_for(chrono::milliseconds(500));
      }

    // the camera will be deinitialized automatically in VideoCapture destructor
    return 0;                                                                   //Kill program
}
