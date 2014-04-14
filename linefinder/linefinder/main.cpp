// include the necessary libraries
#include <iostream>
#include <opencv2/opencv.hpp>
#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc/imgproc.hpp>


using namespace std;
using namespace cv;

int n_rois;
int n_rois_max = 50;
int n_rois_min = 10;
int roi_slider;
int roi_height ;
float x_roi,y_roi,width_roi;


void on_n_roisTrackbar(int, void*){
    n_rois = roi_slider;
}

// Finds center of line
int find_center(vector<Vec4i> lines){
    if (lines.data()) {
        int center =(lines[0][1]+ lines[0][3])/2;
        return center;

    }
    else return -1;
};

// Calculates distance from line
int calc_dist(Point center, int height){
    //dist = (y1+y2)/2 - center_of_pic
    int dist =  center.y;
    return dist;
};

Point calc_2d(int d,int n_rois, int j){

    float res = 90/n_rois;
    Point p_new;
    float alpha = (45-j*res) * CV_PI/180; // (60 - j* degree)*Pi/180
    
    p_new.x = d*sin(alpha);
    cout << "p_new.x = " << p_new.x << ", j =  " << j << ", alpha: " << alpha << endl;
    p_new.y = -d;
    
    return p_new;
};

// initialize the main function
int main(int argc, char *argv[])
{
    Mat newImg;     // original image
    Mat grayImg;    // gray image for the conversion of the original image
    Mat blurImg;    // gray flipped image
    Mat invImg;     // Inverted image
    Mat bwImg;      // binary image
    Mat cdst;       // final image
    Mat cannyImg;
    


    VideoCapture capture(1); // capture from video device (Webcam)
    newImg = imread("/Volumes/Data/Dropbox/DTU/master/Blender/Laser/0001.png");
    
    // Set frame size down from 1080p to 640*480
    Size s = newImg.size();
    int height = s.height;
    int width = s.width;
    
    int d_width = 640;
    int d_height = 480;
    capture.set(CV_CAP_PROP_FRAME_WIDTH, width);
    capture.set(CV_CAP_PROP_FRAME_HEIGHT, height);
    
    
    Mat background = Mat(height, width, IPL_DEPTH_8U,3);
    Mat openrov = Mat(86,60,IPL_DEPTH_8U,3);
    
   
    
    
    namedWindow("Hough Lines", CV_WINDOW_AUTOSIZE); // setup window
    namedWindow("Laser Points", CV_WINDOW_AUTOSIZE);
    namedWindow("True Image",CV_WINDOW_AUTOSIZE);
    namedWindow("Blurred Image",CV_WINDOW_AUTOSIZE);
    
    
    
    
    /// Create Trackbars
    char n_roisTrackbar[50];
    sprintf( n_roisTrackbar, "No. of Points %d", n_rois );
    createTrackbar(n_roisTrackbar, "Hough Lines", &roi_slider, n_rois_max, on_n_roisTrackbar);
    

    
    while(1) {
        // Check if Camera frame is grabbed
//                if (!(capture.read(newImg))) {
//         std::cout << "Cannot find image" << std::endl;
//         return -1;
//         }
        
        background = imread("/Volumes/Data/Shared_Folder/openrov_background.png");
        openrov = imread("/Volumes/Data/Shared_Folder/openrov.png");
        
        cv::Rect roi( cv::Point( 640/2-60/2, 480/2-86/2 ), cv::Size( 60, 86 ));
        cv::Mat destinationROI = background( roi );
        openrov.copyTo(destinationROI);
        
        //create a single channel 1 byte image (i.e. gray-level image)
        grayImg = Mat(height, width, IPL_DEPTH_8U, 1 );
        cvtColor( newImg, grayImg, CV_BGR2GRAY );
        
        // Invert image
        invImg = Mat(height, width, IPL_DEPTH_8U, 1 );
        bitwise_not(grayImg,invImg);
        
        // flip image
        blurImg = Mat(height, width, IPL_DEPTH_8U, 1 );
        GaussianBlur(invImg, blurImg, Size(3,3),2,2);
        
        
        // Edge detect
        Canny(blurImg, cannyImg, 50, 300);
        
        // Create Binary image with a threshold value of 128
        bwImg = Mat(height, width, IPL_DEPTH_8U, 1 );
        threshold(cannyImg, bwImg, 128, 255.0, THRESH_BINARY);
        cvtColor(bwImg, cdst, CV_GRAY2BGR);
        
        for (int j = 0; j < n_rois; j++) {
            // Set parameters for ROI
            roi_height = height/2;
            width_roi = width/n_rois;
            x_roi =j*width_roi;

            // Set and draw region of interest (TOP)
            Rect region_of_interest = Rect(x_roi, 0, width_roi, roi_height );
            Mat top_roi = bwImg(region_of_interest);
            rectangle(cdst, region_of_interest, Scalar(0,0,255), 1, 8, 0);
            // (BOTTOM)
            region_of_interest = Rect(x_roi, height/2, width_roi, roi_height );
            Mat bottom_roi = bwImg(region_of_interest);
            rectangle(cdst, region_of_interest, Scalar(0,255,255), 1, 8, 0);
            
            // Find lines
            vector<Vec4i> top_lines,bottom_lines;
            HoughLinesP(top_roi, top_lines, 1, CV_PI/180, 5, (int)width/n_rois/3, 5 );
            HoughLinesP(bottom_roi, bottom_lines, 1, CV_PI/180, 5, (int)width/n_rois/3, 5 );
            
            // Find the center of lines
            Point top_center,bottom_center;
            try {
                top_center = Point(x_roi+width_roi/2 , find_center(top_lines) );
            } catch (Point top_center) {
                cout << "Exception Thrown: Top";
            }
            try {
                bottom_center = Point(x_roi+width_roi/2 , find_center(bottom_lines) + roi_height);
            } catch (Point bottom_center) {
                cout << "Exception Thrown: Bottom";
            }
        
            // Calculate the distance
            int top_dist = calc_dist(top_center,height);
            int bottom_dist = calc_dist(Point(bottom_center.x,height-bottom_center.y), height);
            string top_string = to_string(top_dist);
            string bottom_string = to_string(bottom_dist);

            // Write distance on image
            putText(cdst, top_string, Point(top_center.x,top_center.y-10), FONT_HERSHEY_PLAIN, 0.7, Scalar(0,255,255) );
            putText(cdst, bottom_string, Point(bottom_center.x,bottom_center.y+20), FONT_HERSHEY_PLAIN, 0.7, Scalar(0,0,255) );
            
            // Draw Houghlines
            if (top_lines.data()) {
                line( cdst, Point(top_lines[0][0]+x_roi, top_lines[0][1]),
                     Point(top_lines[0][2]+x_roi, top_lines[0][3]), Scalar(0,0,255), 3, 8 );
            }
            if (bottom_lines.data()) {
                line( cdst, Point(bottom_lines[0][0]+x_roi, bottom_lines[0][1]+roi_height),
                     Point(bottom_lines[0][2]+x_roi, bottom_lines[0][3]+roi_height), Scalar(0,0,255), 3, 8 );
            }
          
            // Draw Center of lines
            circle(cdst, top_center, 3, Scalar(0,255,0),2);
            circle(cdst, bottom_center , 3, Scalar(0,255,0),2);
            line(cdst, top_center, Point(top_center.x, roi_height), Scalar(0,255,255),1,8);
            line(cdst, bottom_center, Point(bottom_center.x, roi_height), Scalar(0,0,255),1);
            
            
            Point p_top,p_bottom;
            p_top = calc_2d(top_dist, n_rois, j);
            //p_bottom = calc_2d(bottom_dist, n_rois, j);
            //cout << "(top.x,top.y): (" << p_top.x << "," << p_top.y << ")" << endl;
            //cout << "(bottom.x,bottom.y): (" << p_bottom.x << "," << p_bottom.y << ")" << endl;
            circle(background, Point(d_width/2-p_top.x , d_height/2 + p_top.y), 2, Scalar(0,0,255));
            //circle(background, Point(d_width/2-p_bottom.x , d_height/2 + p_bottom.y), 2, Scalar(0,255,0));
            
        }
        
        // Draw image
        imshow("True Image", newImg);
        imshow("Blurred Image", blurImg);
        imshow("Hough Lines", cdst);
        imshow("Laser Points", background);
        
        char c = cvWaitKey(33); // press escape to quit
        if( c == 27 ) break;
        
    }
    
    return 0;
}
