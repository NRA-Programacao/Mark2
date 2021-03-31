#!/usr/bin/env python3
#mudar o python3 para a sua versão do python
import rospy                                  #biblioteca padão para trabalhar com ros em python
import cv2                                    #biblioteca para trabalhar Opencv
from std_msgs.msg import String               #biblioteca para trabalhar com as std_msgs (strings)
from sensor_msgs.msg import Image             #biblioteca para trabalhar com  as sensor_msgs (imagens)
from cv_bridge import CvBridge, CvBridgeError #biblioteca que faz a conversão da imagem publicada em ROS para Opencv
import sys

bridge = CvBridge()    #objeto que faz a transformação entre as imagens do ROS e Opencv

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
  
  cv2.imshow("Image window", cv_image)
  cv2.waitKey(3)

  
def main(args):
  rospy.init_node('image_converter', anonymous=True)                        #inicia o nó
  #cria o Subscriber que vai receber a imagem
  #para mudar qual tópico ele deve se subscrever basta alterar "/topico_video" pelo nome do tópico desejado  
  image_sub = rospy.Subscriber("/topico_video",Image, image_callback)
  try:
    rospy.spin()
  except KeyboardInterrupt:
    print("Shutting down")
  cv2.destroyAllWindows()

if __name__ == '__main__':
    main(sys.argv)
