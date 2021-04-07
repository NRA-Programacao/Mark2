#!/usr/bin/env python3
import numpy as np
from std_msgs.msg import String
import rospy
import sys


def Qrcode_callback(qrcodeInfo):
    print((qrcodeInfo.data))


def main(args):
    rospy.init_node('QRcode', anonymous=True)  # inicia o nó
    # para mudar qual tópico ele deve se subscrever basta alterar "/topico_video" pelo nome do tópico desejado
    qrCode_sub = rospy.Subscriber(
        "/qrcode/informacao", String, Qrcode_callback)
    try:
        rospy.spin()
    except KeyboardInterrupt:
        print("Shutting down")


if __name__ == '__main__':
    main(sys.argv)
