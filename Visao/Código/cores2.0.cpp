#include <iostream>
#include "opencv2/highgui/highgui.hpp"
#include "opencv2/imgproc/imgproc.hpp"
#include <chrono>

using namespace cv;
using namespace std;
using namespace std::chrono;

int main( int argc, char** argv ){
    //VideoCapture cap(0); //captura video da webcam
    VideoCapture cap("AmongUs.mp4");  //abre vídeo de teste
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
        GaussianBlur(imgOriginal, imgGaussianBlur, Size(3, 3), 0, 0, BORDER_DEFAULT);  //Aplica o efeito gaussiano na imagem original
        cvtColor(imgGaussianBlur, imgHSV, COLOR_BGR2HSV);  //Converte as cores de BGR para HSV


        //Aplica a detecção de contornos
        Mat SobelX,SobelY,absSobelX,absSobelY,imgTemp,resultado;
        cvtColor(imgGaussianBlur,imgTemp,COLOR_BGR2GRAY);           //Converte as cores de BGR para escala de cinza
        Sobel(imgTemp,SobelX,CV_64F,1,0);                           //aplica o filtro Sobel no eixo x
        Sobel(imgTemp,SobelY,CV_64F,0,1);                           //aplica o filtro Sobel no eixo Y
        convertScaleAbs(SobelX,absSobelX);
        convertScaleAbs(SobelY,absSobelY);
        bitwise_or(absSobelX,absSobelY,imgContour);                 //"Soma" as imagens com o filtro em x e y
    
        
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
       
        //Cria e mostra as janelas 
        namedWindow("Original",WINDOW_NORMAL);
        namedWindow("Gaussian Blur", WINDOW_NORMAL);
        namedWindow("Sobel", WINDOW_NORMAL);
        namedWindow("Preto", WINDOW_NORMAL);
        namedWindow("Amarelo", WINDOW_NORMAL);
        namedWindow("Vermelho", WINDOW_NORMAL);
        imshow("Original", imgOriginal); 
        imshow("Gaussian Blur", imgGaussianBlur);
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