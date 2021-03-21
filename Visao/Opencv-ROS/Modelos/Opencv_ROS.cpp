//bibliotecas "essenciais"
#include <ros/ros.h>                            //biblioteca padrão do ros 
#include <image_transport/image_transport.h>    //biblioteca pra publicar/receber as sensor_msgs em/de tópicos
#include <cv_bridge/cv_bridge.h>                //biblioteca que faz a "ponte" entre o ROS e Opencv
#include <sensor_msgs/image_encodings.h>        //também serve pra trabalhar com as sensor_msgs
//bibliotecas pra trabalhar com o Opencv
#include <opencv2/imgproc/imgproc.hpp>          
#include <opencv2/highgui/highgui.hpp>
#include <iostream>
//caso adicione mais bibliotecas lembre de fazer as devidas adições no CMakeList.txt e PACKAGE.xml

using namespace cv;
using namespace std; 


//classe que faz a converção entre as imagens do topico de ROS e as imagens do Opencv
class ImageConverter
{
  ros::NodeHandle nh_;                      //caso mude o nome do Node Handle lembre de mudar também na linha 25
  image_transport::ImageTransport it_;      //caso mude o nome do lembre de mudar também na linha 25 e 28
  image_transport::Subscriber image_sub_;   //caso mude o nome do Subscriber lebre de mudar também na linha 28 

public:
  ImageConverter()
    : it_(nh_)
  {
    // Subscriber que recebe o video/imagem que está sendo publicado
    // basta substituir o "/topico_de_video" pelo nome do topico que esta publicando  
    image_sub_ = it_.subscribe("/topico_de_video", 1, &ImageConverter::imageCallback, this);

    namedWindow(OPENCV_WINDOW);     //diz o nome padrão da janela a ser criada 
  }

  ~ImageConverter()
  {
    destroyWindow(OPENCV_WINDOW);
  }
  
  void imageCallback(const sensor_msgs::ImageConstPtr& msg)
  {
    cv_bridge::CvImagePtr cv_ptr;     //onde é criada a variavel que salva o resultado da conversão da imagem, caso queira mudar o nome lembre de alterar na linha 43
    try
    {
      cv_ptr = cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::BGR8);    //é convertido,nesse caso para o padão BGR de 8 bits.
    }
    catch (cv_bridge::Exception& e)
    {
      ROS_ERROR("cv_bridge exception: %s", e.what());
      return;
    }
    //agora a variavel cv_ptr tem contem as informacões da imagem que foi publicada
    //para trabalhar com ela basta criar uma outra variavel tipo std::Mat e atribuir os valores da imagem salva em cv_ptr
    //isso é feito da seguinte maneira: 
    std::Mat imgOpenCV = cv_ptr->image;
    // Adicione seu código de OpenCV abaixo no espaço demarcado 
    //---------------------------------------------------------
    
    //----------(SEU CODIGO AQUI)------------------------------
    
    //---------------------------------------------------------
    //um exmplo simples é o de apenas mostrar a imagem recebida pós covenrcão com uma janela do openCV
     imshow(OPENCV_WINDOW, imgOpenCV);
     waitKey(3);
    //ou
    /*
    imshow(OPENCV_WINDOW, cv_ptr->image);
    waitKey(3);
    */
  }
};

int main(int argc, char** argv)
{
  ros::init(argc, argv, "image_converter");
  ImageConverter ic;
  ros::spin();
  return 0;
}
