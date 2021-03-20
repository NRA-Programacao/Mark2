#include <ros/ros.h>
#include <image_transport/image_transport.h>
#include <cv_bridge/cv_bridge.h>
#include <sensor_msgs/image_encodings.h>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <iostream>

using namespace cv;
using namespace std; 

void setLabel(cv::Mat& im, const std::string label, std::vector<cv::Point>& contour)
{
int fontface = cv::FONT_HERSHEY_SIMPLEX;
double scale = 0.4;
int thickness = 1;
int baseline = 0;


cv::Size text = cv::getTextSize(label, fontface, scale, thickness, &baseline);
cv::Rect r = cv::boundingRect(contour);

cv::Point pt(r.x + ((r.width - text.width) / 2), r.y + ((r.height + text.height) / 2));
cv::rectangle(im, pt + cv::Point(0, baseline), pt + cv::Point(text.width, -text.height), CV_RGB(255,255,255), FILLED);
cv::putText(im, label, pt, fontface, scale, CV_RGB(0,0,0), thickness, 8);
}

static const std::string OPENCV_WINDOW = "Image window";


  //Variáveis que realizarão o controle de reconhecimento das cores    
    
    //Preto
    int black_iLowH = 0;
    int black_iHighH = 179;
    int black_iLowS = 0; 
    int black_iHighS = 255;
    int black_iLowV = 0;
    int black_iHighV = 40;

    //Vermelho
    int red_iLowH1 = 0;
    int red_iHighH1 = 10;
    int red_iLowH2 = 160;
    int red_iHighH2 = 179;
    int red_iLowS = 100; 
    int red_iHighS = 255;
    int red_iLowV = 100;
    int red_iHighV = 255;

class ImageConverter
{
  ros::NodeHandle nh_;
  image_transport::ImageTransport it_;
  image_transport::Subscriber image_sub_;
  image_transport::Publisher image_pub_;

public:
  ImageConverter()
    : it_(nh_)
  {
    // Subscribe to input video feed and publish output video feed
    image_sub_ = it_.subscribe("/downward_cam/camera/image", 1, &ImageConverter::imageCallback, this);

    cv::namedWindow(OPENCV_WINDOW);
  }

  ~ImageConverter()
  {
    cv::destroyWindow(OPENCV_WINDOW);
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

    // Codigo de detecção de cores + simbolos
    Mat imgOriginal = cv_ptr->image;
    Mat imgHSV, imgGaussianBlur,imgContour;
    GaussianBlur(imgOriginal, imgGaussianBlur, Size(9, 9), 0, 0, BORDER_DEFAULT);  //Aplica o efeito gaussiano na imagem original
    cvtColor(imgGaussianBlur, imgHSV, COLOR_BGR2HSV);  //Converte as cores de BGR para HSV

    //Reconhecimento do preto
    Mat black;
    inRange(imgHSV, Scalar(black_iLowH, black_iLowS, black_iLowV), Scalar(black_iHighH, black_iHighS, black_iHighV), black);

    //Reconhecimento do vermelho
    Mat red_low, red_high, red;
    inRange(imgHSV, Scalar(red_iLowH1, red_iLowS, red_iLowV), Scalar(red_iHighH1, red_iHighS, red_iHighV), red_low); //Reconhecimento do primeiro intervalo de vermelho
    inRange(imgHSV, Scalar(red_iLowH2, red_iLowS, red_iLowV), Scalar(red_iHighH2, red_iHighS, red_iHighV), red_high);  //Reconhecimento do segundo intervalo de vermelho
    addWeighted(red_low, 1.0, red_high, 1.0, 0.0, red);  //Junta os dois intervalos de vermelho 


    //Filtros morfológicos (remove pequenos objetos do primeiro plano)
    erode(red, red, getStructuringElement(MORPH_ELLIPSE, Size(5, 5)) );
    dilate(red, red, getStructuringElement(MORPH_ELLIPSE, Size(5, 5)) );

    //detecção de simbolos
    vector<vector<Point>> contoursRed;
    vector<vector<Point>> contoursBlack;
    vector <Point> contoursRedAprox;
    vector <Point> contoursBlackAprox;
    vector<Vec4i> hierarchyRed;
    vector<Vec4i> hierarchyBlack;
    Scalar contoursColorRed(0,0,255),contoursColorBlack(255,0,0);
    findContours(red,contoursRed,RETR_TREE,CHAIN_APPROX_SIMPLE);
    findContours(black,contoursBlack,RETR_TREE,CHAIN_APPROX_SIMPLE);
    int cnt=-1,vtcRed,vtcBlack;
    drawContours(imgOriginal,contoursRed,cnt,contoursColorRed,3,8,hierarchyRed);
    drawContours(imgOriginal,contoursBlack,cnt,contoursColorBlack,3,8,hierarchyBlack);
    for(int i=0; i<contoursRed.size();i++){
        approxPolyDP(contoursRed[i],contoursRedAprox,arcLength(contoursRed[i],true)*0.02,true);
        if((fabs(contourArea(contoursRed[i]))<100)){
            continue;
        }
        vtcRed=contoursRedAprox.size();
        if(vtcRed==12){
            setLabel (imgOriginal,"Primeiro_socorros",contoursRed[i]);
        }
    }
    for(int i=0; i<contoursBlack.size();i++){
        approxPolyDP(contoursBlack[i],contoursBlackAprox,arcLength(contoursBlack[i],true)*0.02,true);
        if((fabs(contourArea(contoursBlack[i]))<100)){
            continue;
        }
        vtcRed=contoursBlackAprox.size();
        if(vtcRed==12){
            setLabel (imgOriginal,"Heliponto",contoursBlack[i]);
        }
    }
    
    // Update GUI Window
    cv::imshow(OPENCV_WINDOW, cv_ptr->image);
    cv::waitKey(3);

    
  }
};

int main(int argc, char** argv)
{
  ros::init(argc, argv, "image_converter");
  ImageConverter ic;
  ros::spin();
  return 0;
}