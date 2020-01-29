#include <ros/ros.h>
#include <image_transport/image_transport.h>
#include <cv_bridge/cv_bridge.h>
#include <sensor_msgs/image_encodings.h>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <iostream>

using namespace cv;
using namespace std;

class ImageConverter
{
  ros::NodeHandle nh_;
  image_transport::ImageTransport it_;
  image_transport::Subscriber image_sub_;
  image_transport::Publisher image_pub_;
  Mat img;
  Mat img_threshold;
  Mat img_gray;
  Mat img_roi;
  char text[40];

public:
  ImageConverter()
    : it_(nh_)
  {
    // Subscribe to input video feed and publish output video feed
    image_sub_ = it_.subscribe("image", 1,
      &ImageConverter::imageCallback, this);
    image_pub_ = it_.advertise("/image_converter/output_video", 1);

    namedWindow("Original_image",CV_WINDOW_AUTOSIZE);
    namedWindow("Gray_image",CV_WINDOW_AUTOSIZE);
    namedWindow("Thresholded_image",CV_WINDOW_AUTOSIZE);
    namedWindow("ROI",CV_WINDOW_AUTOSIZE);
  }

  ~ImageConverter()
  {
    destroyWindow("Original_image");
    destroyWindow("Gray_image");
    destroyWindow("Thresholded_image");
    destroyWindow("ROI");
  }
  
  void imageCallback(const sensor_msgs::ImageConstPtr& msg)
  {
    cv_bridge::CvImagePtr cv_ptr;
    try
    {
      cv_ptr = cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::BGR8);
    }
    catch (cv_bridge::Exception& e)
    {
      ROS_ERROR("cv_bridge exception: %s", e.what());
      return;
    }

    // Draw an example circle on the video stream
    if (cv_ptr->image.rows > 60 && cv_ptr->image.cols > 60)
      circle(cv_ptr->image, Point(50, 50), 10, CV_RGB(255,0,0));

    // Gesture recognition
    int count = 0;
    img = cv_ptr->image;
    Rect roi(340,100,270,270);
    img_roi=img(roi);
    cvtColor(img_roi,img_gray,CV_RGB2GRAY);

    GaussianBlur(img_gray,img_gray,Size(19,19),0.0,0);
    threshold(img_gray,img_threshold,0,255,THRESH_BINARY_INV+THRESH_OTSU);

    vector<vector<Point> >contours;
    vector<Vec4i>hierarchy;
    findContours(img_threshold,contours,hierarchy,CV_RETR_EXTERNAL,CV_CHAIN_APPROX_SIMPLE,Point());
    if(contours.size()>0){
      size_t indexOfBiggestContour = -1;
      size_t sizeOfBiggestContour = 0;

      for (size_t i = 0; i < contours.size(); i++){
        if(contours[i].size() > sizeOfBiggestContour){
          sizeOfBiggestContour = contours[i].size();
          indexOfBiggestContour = i;
        }
      }

      vector<vector<int> >hull(contours.size());
      vector<vector<Point> >hullPoint(contours.size());
      vector<vector<Vec4i> >defects(contours.size());
      vector<vector<Point> >defectPoint(contours.size());
      vector<vector<Point> >contours_poly(contours.size());
      Point2f rect_point[4];
      vector<RotatedRect>minRect(contours.size());
      vector<Rect> boundRect(contours.size());

      for(size_t i=0;i<contours.size();i++){
        if(contourArea(contours[i])>5000){
          convexHull(contours[i],hull[i],true);
          convexityDefects(contours[i],hull[i],defects[i]);
          if(indexOfBiggestContour==i){
            minRect[i]=minAreaRect(contours[i]);
            for(size_t k=0;k<hull[i].size();k++){
              int ind=hull[i][k];
              hullPoint[i].push_back(contours[i][ind]);
            }
            
            count =0;
            for(size_t k=0;k<defects[i].size();k++){
              if(defects[i][k][3]>13*256){
                /*   int p_start=defects[i][k][0];   */
                int p_end=defects[i][k][1];
                int p_far=defects[i][k][2];
                defectPoint[i].push_back(contours[i][p_far]);
                circle(img_roi,contours[i][p_end],3,Scalar(0,255,0),2);
                count++;
              }
            }

            if(count==1)
                strcpy(text,"Hello :D ");
            else if(count==2)
                strcpy(text,"Peace :) ");
            else if(count==3)
                strcpy(text,"3 it is !!");
            else if(count==4)
                strcpy(text,"0100");
            else if(count==5)
                strcpy(text,"FIVE");
            else
                strcpy(text,"Welcome !!");

            putText(img,text,Point(70,70),CV_FONT_HERSHEY_SIMPLEX,3,Scalar(255,0,0),2,8,false);
            drawContours(img_threshold, contours, i,Scalar(255,255,0),2, 8, vector<Vec4i>(), 0, Point() );
            drawContours(img_threshold, hullPoint, i, Scalar(255,255,0),1, 8, vector<Vec4i>(),0, Point());
            drawContours(img_roi, hullPoint, i, Scalar(0,0,255),2, 8, vector<Vec4i>(),0, Point() );
            approxPolyDP(contours[i],contours_poly[i],3,false);
            boundRect[i]=boundingRect(contours_poly[i]);
            rectangle(img_roi,boundRect[i].tl(),boundRect[i].br(),Scalar(255,0,0),2,8,0);
            minRect[i].points(rect_point);
            for(size_t k=0;k<4;k++){
                line(img_roi,rect_point[k],rect_point[(k+1)%4],Scalar(0,255,0),2,8);
            }
          }
        }
      }
    }
    // Update GUI Window
    imshow("Original_image",img);
    imshow("Gray_image",img_gray);
    imshow("Thresholded_image",img_threshold);
    imshow("ROI",img_roi);
    waitKey(3);

    // Output modified video stream
    //Convert this message to a ROS sensor_msgs::Image message. 
    image_pub_.publish(cv_ptr->toImageMsg());
  }
};

int main(int argc, char** argv)
{
  ros::init(argc, argv, "image_converter");
  ImageConverter ic;
  ros::spin();
  return 0;
}