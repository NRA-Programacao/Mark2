//Reference:https://www.learnopencv.com/opencv-qr-code-scanner-c-and-python/

// River 1x2 SPFC (AMANHA)

#include <iostream>
#include <algorithm>
#include <vector>
#include <zbar.h>

#include <opencv2/core.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc/imgproc.hpp>

using namespace std;
using namespace cv;
using namespace zbar;

typedef struct{
  string type;
  string data;
  vector <Point> location;
}decodedObject;

int i = 0;

// Find and decode barcodes and QR codes
void decode(Mat &im, vector<decodedObject>&decodedObjects){

  // Create zbar scanner
  ImageScanner scanner;

  // Configure scanner
  scanner.set_config(ZBAR_QRCODE, ZBAR_CFG_ENABLE, 1);

  // Convert image to grayscale
  Mat imGray;
  cvtColor(im, imGray,CV_BGR2GRAY);

  // Wrap image data in a zbar image
  Image image(im.cols, im.rows, "Y800", (uchar *)imGray.data, im.cols * im.rows);

  // Scan the image for barcodes and QRCodes
  int n = scanner.scan(image);

  // Print results
  for(Image::SymbolIterator symbol = image.symbol_begin(); symbol != image.symbol_end(); ++symbol){
    decodedObject obj;

    obj.type = symbol->get_type_name();
    obj.data = symbol->get_data();

    // Print type and data
    cout << "Type : " << obj.type << endl;
    cout << "Data [" << i << "]: " << obj.data << endl << endl;
    decodedObjects.push_back(obj);
    i++;
  }
}

int main(int argc, char *argv[]){

  // Read image
  VideoCapture cap(0);  //Abre a webcam
    if ( !cap.isOpened() )  
    {
         cout << "Não foi possível abrir webcam ou video ..." << endl;
         return -1;
    }

  while (true){
    Mat img;
    bool bSuccess = cap.read(img); // Lê um frame do video
    if (!bSuccess) //Caso não obtenha sucesso
    {
      cout << "Impossivel de ler video" << endl;
      break;
    }

    // Variable for decoded objects
    vector<decodedObject> decodedObjects;
    
    // Find and decode barcodes and QR codes
    decode(img, decodedObjects);

    //Mostra a imagem
    namedWindow("Video", WINDOW_NORMAL);
    imshow("Video", img);

    if (waitKey(30) == 27){ //Espera pela tecla 'esc' ser pressionada, se for o loop é quebrado
      cout << "tecla ESC foi pressionado pelo usuario" << endl;
      break; 
    }
  }

  return 0;

}