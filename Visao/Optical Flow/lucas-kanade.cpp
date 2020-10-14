// Pyramid Lukas-Kanade optical flow 

#include <iostream>
#include <string>
#include <opencv2/core.hpp>
#include <opencv2/highgui.hpp>
#include <opencv2/imgproc.hpp>
#include <opencv2/videoio.hpp>
#include <opencv2/video.hpp>

using namespace std;
using namespace cv;

int main(int argc, char** argv) {
    
    VideoCapture cap("drone-video-test.mp4");  //Abre a webcam
    if ( !cap.isOpened() )  
    {
         cout << "Não foi possível abrir webcam ou video ..." << endl;
         return -1;
    }

    Mat old_frame, old_gray;
    vector<Point2f> p0, p1;

    // Take first frame and find corners in it
    cap >> old_frame;
    cvtColor(old_frame, old_gray, COLOR_BGR2GRAY);
    goodFeaturesToTrack(old_gray, p0, 500, 0.3, 7, Mat(), 7, false, 0.04);
    int tam = p0.size();
    // Create a mask image for drawing purposes
    Mat mask = Mat::zeros(old_frame.size(), old_frame.type());
    
    while (true){
        Mat maskCopy;
        mask.copyTo(maskCopy);

        Mat frame, frame_gray;

        cap >> frame; // Lê um frame do video
        if (frame.empty()){ //Caso não obtenha sucesso
            cout << "Impossivel de ler video" << endl;
            break;
        }   
        cvtColor(frame, frame_gray, COLOR_BGR2GRAY); //Mudando a cor do frame para grayscale
        
        //if(p0.size() < tam){
            goodFeaturesToTrack(frame_gray, p0, 500, 0.3, 7, Mat(), 7, false, 0.04);
        //}
        
        // calculate optical flow
        vector<uchar> status;
        vector<float> err;
        TermCriteria criteria = TermCriteria((TermCriteria::COUNT) + (TermCriteria::EPS), 20, 0.03);
        calcOpticalFlowPyrLK(old_gray, frame_gray, p0, p1, status, err, Size(15,15), 2, criteria);
        vector<Point2f> good_new;
        float dXmedia, dYmedia;
        for(uint i = 0; i < p0.size(); i++){
            // Select good points
            if(status[i] == 1) {
                good_new.push_back(p1[i]);
                // draw the tracks
                line(maskCopy, p0[i], p1[i], Scalar(0, 0, 255), 1, 8);
                circle(frame, p1[i], 2, Scalar(0, 0, 255), -1);
                dXmedia += p1[i].x - p0[i].x;
                dYmedia += p1[i].y - p0[i].y;
            }
        }
        
        dXmedia = dXmedia/p0.size();
        dYmedia = dYmedia/p0.size();

        Mat imgOpticalFlow;
        namedWindow("Optical Flow", WINDOW_NORMAL);
        add(frame, maskCopy, imgOpticalFlow);
        
        //Criando texto
        ostringstream str, str1;
        str << "dX: " << int(dXmedia) << "  dY: " << int(dYmedia);
        if(int(dXmedia) > 1){
            str1 << "Esquerda"; 
            if(int(dYmedia) > 1){
                str1 << " Alto";   
            }   
            else if(int(dYmedia) < -1){
                str1 << " Baixo";
            }
        }
        else if(int(dXmedia) < -1){
            str1 << "Direita";  
            if(int(dYmedia) > 1){
                str1 << " Alto";   
            }   
            else if(int(dYmedia) < -1){
                str1 << " Baixo";
            }  
        }
        else if(int(dYmedia) > 1){
            str1 << "Alto";  
            if(int(dXmedia) > 1){
                str1 << " Esquerda";   
            }   
            else if(int(dXmedia) < -1){
                str1 << " Direita";
            }  
        }
        else if(int(dYmedia) < -1){
            str1 << "Baixo";  
            if(int(dXmedia) > 1){
                str1 << " Esquerda";   
            }   
            else if(int(dXmedia) < -1){
                str1 << " Direita";
            }  
        }
        putText(imgOpticalFlow, //target image
            str1.str(), //text
            cv::Point(10, imgOpticalFlow.rows - 100), //position
            cv::FONT_HERSHEY_DUPLEX,
            2,
            CV_RGB(0, 255, 0), //font color
            2);
        putText(imgOpticalFlow, //target image
            str.str(), //text
            cv::Point(10, imgOpticalFlow.rows - 10), //position
            cv::FONT_HERSHEY_DUPLEX,
            1.5,
            CV_RGB(0, 255, 0), //font color
            2);
        imshow("Optical Flow", imgOpticalFlow);
        

        if (waitKey(60) == 27){ //Espera pela tecla 'esc' ser pressionada, se for o loop é quebrado
            cout << "tecla ESC foi pressionado pelo usuario" << endl;
            break; 
        }
        
        // Now update the previous frame and previous points
        old_gray = frame_gray.clone();
        p0 = good_new;
    }   
    
 return 0;
}