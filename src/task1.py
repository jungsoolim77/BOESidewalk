import exifread
import numpy as np
import cv2
from matplotlib import pyplot as plt
import datetime as datetime

class BOESidewalk():
    def __init__(self, directory):
        self.directory = directory

    def readImage(self, imagefile):
        img = cv2.imread(self.directory+imagefile)
        plt.imshow(img, cmap = 'gray', interpolation = 'bicubic')
        plt.xticks([]), plt.yticks([])  # to hide tick values on X and Y axis
        plt.show()

    def splitMergeImage(self, imagefile):
        img = cv2.imread(self.directory + imagefile)
        b,g,r = cv2.split(img)
        img2 = cv2.merge([r,g,b])
        plt.subplot(121);plt.imshow(img)
        plt.subplot(122);plt.imshow(img2)
        plt.show()
        cv2.imshow('bgr image',img) # expects true color
        cv2.imshow('rgb image',img2) # expects distorted color
        cv2.waitKey(0)
        cv2.destroyAllWindows()

    # Load an color image in grayscale
    def grascaleImage(self, imagefile):
        img = cv2.imread(self.directory + imagefile, cv2.IMREAD_COLOR)
        imgWindow = 'image'
        cv2.namedWindow(imgWindow, cv2.WINDOW_NORMAL)
        cv2.imshow(imgWindow,img)
        cv2.waitKey(0)
        cv2.imwrite(self.directory + 'test.jpg',img)
        grayimg = cv2.imread(self.directory + 'test.jpg')
        plt.imshow(grayimg)
        plt.xticks([]), plt.yticks([])  # to hide tick values on X and Y axis
        plt.show()
        # cv2.destroyAllWindows()
        cv2.destroyWindow(imgWindow)

    def toDegree(self, data):
        l = list((str(data).replace('[', '')).replace(']', '').split(','))
        s = l[2].split('/')
        return float(l[0]) + (1/60)*float(l[1]) + (float(s[0])/float(s[1]))*(1/(60*60))

    def readEXIF(self, imagefile):
        # open image file for reading (binary mode)
        f = open(self.directory + imagefile, 'rb')
        tags = exifread.process_file(f)
        print(tags.keys())
        latref = tags.get('GPS GPSLatitudeRef')
        lat = self.toDegree(tags.get('GPS GPSLatitude'))
        lonref = tags.get('GPS GPSLongitudeRef')
        lon = self.toDegree(tags.get('GPS GPSLongitude'))
        imake = tags.get('Image Make')
        print('Latitude:', latref, lat, ' Longitude:', lonref, lon, imake)
        imageTime = datetime.datetime.strptime(str(tags.get('Image DateTime')), '%Y:%m:%d %H:%M:%S')
        print('Date-Time of Image:', imageTime)

if __name__ == "__main__":
    boesidewalk = BOESidewalk(directory='C:/Users/pdmuser/PycharmProjects/BOESidewalk/images/GoPro/')
    # boesidewalk.readImage('GF120743.jpg')
    # boesidewalk.splitMergeImage('GF120743.jpg')
    # boesidewalk.grascaleImage('GF120743.jpg')
    boesidewalk.readEXIF('GF120743.jpg')
