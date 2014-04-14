// include the necessary libraries
#include <iostream>
#include <opencv2/opencv.hpp>

void switch_callback_h( int position );
void switch_callback_l( int position );


int high_switch_value = 0;
int highInt = 0;
int low_switch_value = 0;
int lowInt = 0;

void switch_callback_h( int position ){
    highInt = position;
}
void switch_callback_l( int position ){
    lowInt = position;
}


// initialize the main function
int main(int argc, char *argv[])
{
    CvCapture* capture =capture = cvCaptureFromCAM(-1); // capture from video device (Macbook iSight)
    
    cvNamedWindow("Window1", CV_WINDOW_AUTOSIZE); // setup window
    cvNamedWindow("Window2", CV_WINDOW_AUTOSIZE); // setup window
    cvNamedWindow("Window3", CV_WINDOW_AUTOSIZE); // setup window
    
    IplImage* newImg; // original image
    IplImage* grayImg; // gray image for the conversion of the original image
    IplImage* cannyImg; // gray image for the canny edge detection
    IplImage* flipImg;
    IplImage* layerImg;
    IplImage* drawnImg;
    
    while(1) {
        newImg = cvQueryFrame( capture );
        if( !newImg ) break;
        
        //create a single channel 1 byte image (i.e. gray-level image)
        grayImg = cvCreateImage( cvSize(newImg->width, newImg->height), IPL_DEPTH_8U, 1 );
        cvCvtColor( newImg, grayImg, CV_BGR2GRAY );
        
        // flip image
        flipImg = cvCreateImage(cvGetSize(newImg), IPL_DEPTH_8U, 1);
        cvFlip(grayImg, flipImg, 1);
        
        // canny
        cannyImg = cvCreateImage(cvGetSize(newImg), IPL_DEPTH_8U, 1);
        
        
        // Edge Detection Variables
        int N = 7; //kernel size?
        
        int aperature_size = N;
        double lowThresh = 20;
        double highThresh = 40;
        
        // Create trackbars
        cvCreateTrackbar( "High", "Window1", &high_switch_value, 4, switch_callback_h );
        cvCreateTrackbar( "Low", "Window1", &low_switch_value, 4, switch_callback_l );
        
        
        switch( highInt ){
            case 0:
                highThresh = 200;
                break;
            case 1:
                highThresh = 400;
                break;
            case 2:
                highThresh = 600;
                break;
            case 3:
                highThresh = 800;
                break;
            case 4:
                highThresh = 1000;
                break;
        }
        switch( lowInt ){
            case 0:
                lowThresh = 0;
                break;
            case 1:
                lowThresh = 100;
                break;
            case 2:
                lowThresh = 200;
                break;
            case 3:
                lowThresh = 400;
                break;
            case 4:
                lowThresh = 600;
                break;
        }
        
        // Edge Detection
        cvCanny(flipImg, cannyImg, lowThresh*N*N, highThresh*N*N, aperature_size);
        cvShowImage("Window1", cannyImg);
        
        char c = cvWaitKey(33); // press escape to quit
        if( c == 27 ) break;
        
    }
    
    cvReleaseCapture( &capture );
    cvDestroyWindow( "Window1" );
    cvDestroyWindow( "Window2" );
    cvDestroyWindow( "Window3" );
    
    cvReleaseImage( &newImg );
    cvReleaseImage( &grayImg );
    cvReleaseImage( &cannyImg );
    cvReleaseImage( &flipImg );
    cvReleaseImage( &layerImg );
    cvReleaseImage( &drawnImg );
    
    
    return 0;
}