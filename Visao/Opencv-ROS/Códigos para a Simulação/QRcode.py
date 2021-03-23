#!/usr/bin/env python3
#mudar o python3 para a sua versão do python
from __future__ import print_function
import rospy                                  #biblioteca padão para trabalhar com ros em python
import cv2                                    #biblioteca para trabalhar Opencv
from std_msgs.msg import String               #biblioteca para trabalhar com as std_msgs (strings)
from sensor_msgs.msg import Image             #biblioteca para trabalhar com  as sensor_msgs (imagens)
from cv_bridge import CvBridge, CvBridgeError #biblioteca que faz a conversão da imagem publicada em ROS para Opencv
import sys
import pyzbar.pyzbar as pyzbar
import numpy as np


bridge = CvBridge()    #objeto que faz a transformação entre as imagens do ROS e Opencv

def decode(im) : 
  # Find barcodes and QR codes
  decodedObjects = pyzbar.decode(im)

  # Print results
  for obj in decodedObjects:
    print('Type : ', obj.type)
    print('Data : ', obj.data,'\n')
    
  return decodedObjects


# Display barcode and QR code location  
def display(im, decodedObjects):
  # Loop over all decoded objects
  for decodedObject in decodedObjects: 
    points = decodedObject.polygon

    # If the points do not form a quad, find convex hull
    if len(points) > 4 : 
      hull = cv2.convexHull(np.array([point for point in points], dtype=np.float32))
      hull = list(map(tuple, np.squeeze(hull)))
    else : 
      hull = points
    
    # Number of points in the convex hull
    n = len(hull)

    # Draw the convext hull
    for j in range(0,n):
      cv2.line(im, hull[j], hull[ (j+1) % n], (255,0,0), 3)

  # Display results 
  cv2.namedWindow("janela", cv2.WINDOW_FREERATIO)
  cv2.imshow("janela", im)
  cv2.waitKey(1)

  
#função de callback do SUbscriber
def image_callback(ros_image):
  global bridge
  #c
  try:
    cv_image = bridge.imgmsg_to_cv2(ros_image, "bgr8")    #faz a converção entre a imagem em ROS e a de Opencv bgr de 8 bits
  except CvBridgeError as e:
      print(e)
  #a partir de agora a variavel cv_image é uma imagem no padrão Opencv
  #como exemplo é possivel mostrar ela em uma janela com comandos do Opencv
  
  decodedObjects = decode(cv_image)
  display(cv_image, decodedObjects)


  
def main(args):
  rospy.init_node('image_converter', anonymous=True)                        #inicia o nó
  #cria o Subscriber que vai receber a imagem
  #para mudar qual tópico ele deve se subscrever basta alterar "/topico_video" pelo nome do tópico desejado  
  image_sub = rospy.Subscriber("/downward_cam/camera/image",Image, image_callback)
  try:
    rospy.spin()
  except KeyboardInterrupt:
    print("Shutting down")
  cv2.destroyAllWindows()

if __name__ == '__main__':
    main(sys.argv)
