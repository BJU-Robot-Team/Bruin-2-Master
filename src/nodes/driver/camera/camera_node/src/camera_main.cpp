// camera_main
// V1.0 by Bill Lovegrove modified from Michael Boye's beta code
// added integration with ROS
// V1.1 added green/blue target tracking
// V1.2 changed to red/yellow stacked squares, attempt to flush the buffer
// V1.3 change yellow to green (now looking for green/red)
using namespace std;

// Factors when searching for colored tracking target
// (yellow above red square) - change to green above red? 
#define TARGET_WIDTH 0.14           // size of blue/green squares in meters
#define TARGET_SPACING 0.26         // center to center spacing of squares
#define WIDTH_MIN 7                // squares must be at least this many pixels
#define SQUARE_TEST 0.3              // Difference between height and width must be less than this percentage
#define WIDTH_TOLERANCE 0.4         // Percent error in width
#define HORIZONTAL_TOLERANCE 0.4    // Percent error of horizontal offset between squares compared to size
#define BASE_PIXEL_EQUIVALENT 640.0 // See spreadsheet for details 800 for laptop, 600 for Bruin-2
#define YELLOW_THRESHOLD 25
#define RED_THRESHOLD 40
#define GREEN_THRESHOLD 20

#define BLUE 0
#define GREEN 1
#define RED 2

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
    int no_target_count = 0; // count how many frames since we saw the target
    #define NO_TARGET_LIMIT 3  // after this many frames, start reporting no target

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

    namedWindow( "Rectangles", WINDOW_NORMAL);
    resizeWindow("Rectangles", 500,337);

    while (ros_interface.isNodeRunning()) {
        Mat image, dst, cdst, cimg, ired, iyellow, bnw;                                        //Image transformation variables

        // buffer fills with several images, so let's try to flush them
        for (int i=0; i<3; i++) cap>>image;     //Get a new frame from camera
        cap>>image;                                                           //Get a new frame from camera
 	//cout << "Got a new image." << endl;
        //imshow("Camera Image", image);                                         
        medianBlur(image,image,5);                                              //Smooth out the image
        flip(image,image,-1);                                                   //Flip image upside down because camera is mounted upside down--uncomment for final code
        cvtColor(image, ired, COLOR_RGB2GRAY);                             //Create a gray scale copy of original image
        cvtColor(image, iyellow, COLOR_RGB2GRAY);                             //Create a gray scale copy of original image
        Canny(image, dst, 90, 200, 3);                                          //Lowthresh originally set at 50

        //Debugging between ObstacleDetect and HoughLines
        //Set if==0 to debug ObstacleDetect
        //Set if==1 to debug HoughLines
        //Comment out if, else, and endif for running program in Bruin 2

        #if 1 //--uncomment for debugging purposes

        //ObstacleDetect
        //Set up loop to scan image for RED/YELLOW SQUARES
        //Then create BLACK and WHITE image from scan results
        //WHITE is TRUE, BLACK is FALSE

        for(int y = 0; y < image.rows; y++)                                     //Set up number of rows to loop through
        {
            for(int x = 0; x < image.cols; x++)                                 //Set up number of columbs to loop through
            {
                if((image.at<cv::Vec3b>(y,x)[RED]-RED_THRESHOLD)>image.at<cv::Vec3b>(y,x)[GREEN] && (image.at<cv::Vec3b>(y,x)[RED]-RED_THRESHOLD)>image.at<cv::Vec3b>(y,x)[BLUE])
                //This is dominant RED
                {
		  ired.at<uchar>(y,x)=255;
                }
                else
                {
                  ired.at<uchar>(y,x)=0;
                }
                if((image.at<cv::Vec3b>(y,x)[GREEN]-GREEN_THRESHOLD)>image.at<cv::Vec3b>(y,x)[BLUE] && (image.at<cv::Vec3b>(y,x)[GREEN]-GREEN_THRESHOLD)>image.at<cv::Vec3b>(y,x)[RED])
                //This is dominant GREEN (formerly RED/GREEN=Yellow)
                {
		  iyellow.at<uchar>(y,x)=255;
                }
                else
                {
                  iyellow.at<uchar>(y,x)=0;
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
 
 	vector<vector<Point> > r_contours, y_contours;
  	vector<Vec4i> hierarchy;

        // rectangles around red
	findContours( ired, r_contours, hierarchy, RETR_TREE, CHAIN_APPROX_SIMPLE, Point(0, 0) );
	vector<vector<Point> > r_contours_poly( r_contours.size() );
  	vector<Rect> r_boundRect( r_contours.size() );
  	//vector<Point2f>r_center( r_contours.size() );
  	//vector<float>r_radius( r_contours.size() );

        for( size_t i = 0; i < r_contours.size(); i++ )
        {  approxPolyDP( Mat(r_contours[i]), r_contours_poly[i], 3, true );
           r_boundRect[i] = boundingRect( Mat(r_contours_poly[i]) );
       	   //minEnclosingCircle( (Mat)g_contours_poly[i], g_center[i], g_radius[i]);
         }
        for( size_t i = 0; i< r_contours.size(); i++ )
        {
             Scalar color = Scalar( 0, 0, 255 );
             // drawContours( image, g_contours_poly, (int)i, color, 1, 8, vector<Vec4i>(), 0, Point() );
             if ( r_boundRect[i].width > WIDTH_MIN 
                  && (abs(1.0-(1.0*r_boundRect[i].width/r_boundRect[i].height) ) < SQUARE_TEST) ) {
		// Draw the areas that match the size and shape threshold
		rectangle( image, r_boundRect[i].tl(), r_boundRect[i].br(), color, 3, 8, 0 );
	     }
         }

	// rectangles around yellow
	findContours( iyellow, y_contours, hierarchy, RETR_TREE, CHAIN_APPROX_SIMPLE, Point(0, 0) );
	vector<vector<Point> > y_contours_poly( y_contours.size() );
  	vector<Rect> y_boundRect( y_contours.size() );

        for( size_t i = 0; i < y_contours.size(); i++ )
        {  approxPolyDP( Mat(y_contours[i]), y_contours_poly[i], 3, true );
           y_boundRect[i] = boundingRect( Mat(y_contours_poly[i]) );
        }
        Mat drawing = Mat::zeros( iyellow.size(), CV_8UC3 );
        for( size_t i = 0; i< y_contours.size(); i++ )
        {
             Scalar color = Scalar( 0, 255, 0 );
             // drawContours( image, contours_poly, (int)i, color, 1, 8, vector<Vec4i>(), 0, Point() );
              if ( y_boundRect[i].width > WIDTH_MIN
                   && (abs(1.0-(1.0*y_boundRect[i].width/y_boundRect[i].height) ) < SQUARE_TEST) ) {
		// Draw the areas that match the size and shape threshold
		rectangle( image, y_boundRect[i].tl(), y_boundRect[i].br(), color, 3, 8, 0 );
	     }
         }

        target_found = false;
	target_distance = 0.0;
	target_direction = 0.0;
        for( size_t ir = 0; ir< r_contours.size(); ir++ ) {
             if ( r_boundRect[ir].width > WIDTH_MIN
                 && (abs(1.0-(1.0*r_boundRect[ir].width/r_boundRect[ir].height) ) < SQUARE_TEST) ) {
                // look for matching yellow but only on right-sized red
    		for( size_t iy = 0; iy < y_contours.size(); iy++ ) {
			if (
			  ( abs( ((1.0*r_boundRect[ir].tl().x - y_boundRect[iy].tl().x)/r_boundRect[ir].width) ) < HORIZONTAL_TOLERANCE ) // yellow on top of red
			  && ( r_boundRect[ir].br().y > y_boundRect[iy].tl().y) // red on top of yellow
			  && ( abs ( 1.0 - (1.0*r_boundRect[ir].width/y_boundRect[iy].width) ) < WIDTH_TOLERANCE ) // similar size
			  && ( r_boundRect[ir].width>WIDTH_MIN) && ( y_boundRect[iy].width>WIDTH_MIN) // minimum size
                          && (abs(1.0-(1.0*y_boundRect[iy].width/y_boundRect[iy].height) ) < SQUARE_TEST) // also square
                          && ( (r_boundRect[ir].tl().x - y_boundRect[iy].tl().x) << 2*r_boundRect[ir].width ) // not too far apart
			) {
                          int pixels_offset;
	                  Scalar color = Scalar( 0,200,200);
			  rectangle( image, y_boundRect[iy].tl(), y_boundRect[iy].br(), color, 5, 8, 0 );
			  line( image, y_boundRect[iy].tl(), y_boundRect[iy].br(), color, 5, 8, 0 );
			  rectangle( image, r_boundRect[ir].tl(), r_boundRect[ir].br(), color, 5, 8, 0 );
			  line( image, r_boundRect[ir].tl(), r_boundRect[ir].br(), color, 5, 8, 0 );
                          target_found = true;
			  pixels_offset = ( r_boundRect[ir].br().x - (image.cols/2) + y_boundRect[iy].width/2 );
			  target_direction = atan(pixels_offset/BASE_PIXEL_EQUIVALENT);
			  target_distance = TARGET_SPACING * BASE_PIXEL_EQUIVALENT / (r_boundRect[ir].tl().y-y_boundRect[iy].tl().y) - 1;
			}
		}
            }
	}

	imshow( "Rectangles", image);

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

        if ( target_found) {
          ros_interface.publishMessages(target_found, target_direction, target_distance);
          no_target_count = 0;
        } else {
          if ( no_target_count < NO_TARGET_LIMIT ) {
            // Dont' send updates until several target misses
            no_target_count += 1;
          } else {
            ros_interface.publishMessages(target_found, target_direction, target_distance);
          } 
        }
        // Sleep between images
	waitKey(50);
        //this_thread::sleep_for(chrono::milliseconds(500));
      }

    // the camera will be deinitialized automatically in VideoCapture destructor
    return 0;                                                                   //Kill program
}
