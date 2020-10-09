#include <iostream>
#include "opencv2/highgui/highgui.hpp"
#include "opencv2/imgproc/imgproc.hpp"
#include <chrono>

using namespace cv;
using namespace std;
using namespace std::chrono;
void setLabel(cv::Mat& im, const std::string label, std::vector<cv::Point>& contour)
{
int fontface = cv::FONT_HERSHEY_SIMPLEX;
double scale = 0.4;
int thickness = 1;
int baseline = 0;


cv::Size text = cv::getTextSize(label, fontface, scale, thickness, &baseline);
cv::Rect r = cv::boundingRect(contour);

cv::Point pt(r.x + ((r.width - text.width) / 2), r.y + ((r.height + text.height) / 2));
cv::rectangle(im, pt + cv::Point(0, baseline), pt + cv::Point(text.width, -text.height), CV_RGB(255,255,255), CV_FILLED);
cv::putText(im, label, pt, fontface, scale, CV_RGB(0,0,0), thickness, 8);
}

int main( int argc, char** argv ){
    //VideoCapture cap(0); //captura video da webcam
    VideoCapture cap(0);  //abre vídeo de teste
    if ( !cap.isOpened() )  
    {
         cout << "Não foi possível abrir webcam ou video ..." << endl;
         return -1;
    }


    //Variáveis que realizarão o controle de reconhecimento das cores    
    
    //Preto
    int black_iLowH = 0;
    int black_iHighH = 179;
    int black_iLowS = 0; 
    int black_iHighS = 255;
    int black_iLowV = 0;
    int black_iHighV = 40;

    //Amarelo
    int yellow_iLowH = 20;
    int yellow_iHighH = 30;
    int yellow_iLowS = 100; 
    int yellow_iHighS = 255;
    int yellow_iLowV = 100;
    int yellow_iHighV = 255;

    //Vermelho
    int red_iLowH1 = 0;
    int red_iHighH1 = 10;
    int red_iLowH2 = 160;
    int red_iHighH2 = 179;
    int red_iLowS = 100; 
    int red_iHighS = 255;
    int red_iLowV = 100;
    int red_iHighV = 255;
 
    //
    while (true){
        high_resolution_clock::time_point t1 = high_resolution_clock::now(); //Salva o tempo atual do sistema 
        Mat imgOriginal;
        bool bSuccess = cap.read(imgOriginal); // Lê um frame do video
        if (!bSuccess) //Caso não obtenha sucesso
        {
            cout << "Impossivel de ler video" << endl;
            break;
        }

        //Modificações na imagem
        Mat imgHSV, imgGaussianBlur,imgContour;
        GaussianBlur(imgOriginal, imgGaussianBlur, Size(9, 9), 0, 0, BORDER_DEFAULT);  //Aplica o efeito gaussiano na imagem original
        cvtColor(imgGaussianBlur, imgHSV, COLOR_BGR2HSV);  //Converte as cores de BGR para HSV
        
        //Reconhecimento do preto
        Mat black;
        inRange(imgHSV, Scalar(black_iLowH, black_iLowS, black_iLowV), Scalar(black_iHighH, black_iHighS, black_iHighV), black);

        //Reconhecimento do amarelo
        Mat yellow;
        inRange(imgHSV, Scalar(yellow_iLowH, yellow_iLowS, yellow_iLowV), Scalar(yellow_iHighH, yellow_iHighS, yellow_iHighV), yellow);

        //Reconhecimento do vermelho
        Mat red_low, red_high, red;
        inRange(imgHSV, Scalar(red_iLowH1, red_iLowS, red_iLowV), Scalar(red_iHighH1, red_iHighS, red_iHighV), red_low); //Reconhecimento do primeiro intervalo de vermelho
        inRange(imgHSV, Scalar(red_iLowH2, red_iLowS, red_iLowV), Scalar(red_iHighH2, red_iHighS, red_iHighV), red_high);  //Reconhecimento do segundo intervalo de vermelho
        addWeighted(red_low, 1.0, red_high, 1.0, 0.0, red);  //Junta os dois intervalos de vermelho 

        //Filtros morfológicos (remove pequenos objetos do primeiro plano)
        erode(red, red, getStructuringElement(MORPH_ELLIPSE, Size(5, 5)) );
        dilate(red, red, getStructuringElement(MORPH_ELLIPSE, Size(5, 5)) );
        erode(yellow, yellow, getStructuringElement(MORPH_ELLIPSE, Size(5, 5)) );
        dilate(yellow, yellow, getStructuringElement(MORPH_ELLIPSE, Size(5, 5)) );
       

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

        
        


        //Cria e mostra as janelas 
        namedWindow("Original",WINDOW_NORMAL);
        namedWindow("Gaussian Blur", WINDOW_NORMAL);
        namedWindow("Preto", WINDOW_NORMAL);
        namedWindow("Amarelo", WINDOW_NORMAL);
        namedWindow("Vermelho", WINDOW_NORMAL);
        imshow("Original", imgOriginal); 
        imshow("Sobel", imgContour);
        imshow("Preto", black); 
        imshow("Amarelo", yellow);  
        imshow("Vermelho", red); 
         
        
        high_resolution_clock::time_point t2 = high_resolution_clock::now();     //salva o tempo do sisltema no final do loop
        duration<double> time_span = duration_cast<duration<double>>(t2 - t1);   //calcula o tempo utilizado por cada loop
        cout << "It took me " << time_span.count() << " seconds."<<endl;
        if (waitKey(30) == 27) //Espera pela tecla 'esc' ser pressionada, se for o loop é quebrado
        {
            cout << "tecla ESC foi pressionado pelo usuario" << endl;
            break; 
        }
        
        
    }

   return 0;
}
