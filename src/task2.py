import cv2
import numpy as np
from matplotlib import pyplot as plt
from skimage import data, io, filters

class BOESidewalkAI():
    def __init__(self, directory):
        self.directory = directory

    def edgeDection(self,filename):
        img = cv2.imread(self.directory + filename,0)
        edges = cv2.Canny(img,100,200)

        plt.subplot(121),plt.imshow(img,cmap = 'gray')
        plt.title('Original Image'), plt.xticks([]), plt.yticks([])
        plt.subplot(122),plt.imshow(edges,cmap = 'gray')
        plt.title('Edge Image'), plt.xticks([]), plt.yticks([])
        plt.show()

    def segBinary(self, filename):
        img = cv2.imread(self.directory + filename)
        gray = cv2.cvtColor(img, cv2.COLOR_BGR2GRAY)
        ret, thresh = cv2.threshold(gray, 0, 255, cv2.THRESH_BINARY_INV + cv2.THRESH_OTSU)
        # noise removal
        kernel = np.ones((3, 3), np.uint8)
        opening = cv2.morphologyEx(thresh, cv2.MORPH_OPEN, kernel, iterations=2)

        # sure background area
        sure_bg = cv2.dilate(opening, kernel, iterations=3)

        # Finding sure foreground area
        dist_transform = cv2.distanceTransform(opening, cv2.DIST_L2, 5)
        ret, sure_fg = cv2.threshold(dist_transform, 0.7 * dist_transform.max(), 255, 0)

        # Finding unknown region
        sure_fg = np.uint8(sure_fg)
        unknown = cv2.subtract(sure_bg, sure_fg)
        # Marker labelling
        ret, markers = cv2.connectedComponents(sure_fg)

        # Add one to all labels so that sure background is not 0, but 1
        markers = markers + 1

        # Now, mark the region of unknown with zero
        markers[unknown == 255] = 0
        markers = cv2.watershed(img, markers)
        img[markers == -1] = [255, 0, 0]

        plt.subplot(121), plt.imshow(img, cmap='gray')
        plt.title('Original Image'), plt.xticks([]), plt.yticks([])
        plt.subplot(122), plt.imshow(markers, cmap='gray')
        plt.title('Segmentation Image'), plt.xticks([]), plt.yticks([])
        plt.show()

    def segGrabCut(self, filename1, filename2):
        img = cv2.imread(self.directory + filename1)
        mask = np.zeros(img.shape[:2], np.uint8)

        bgdModel = np.zeros((1, 65), np.float64)
        fgdModel = np.zeros((1, 65), np.float64)

        rect = (50, 50, 450, 290)
        cv2.grabCut(img, mask, rect, bgdModel, fgdModel, 5, cv2.GC_INIT_WITH_RECT)

        mask2 = np.where((mask == 2) | (mask == 0), 0, 1).astype('uint8')
        img = img * mask2[:, :, np.newaxis]

        plt.imshow(img), plt.colorbar(), plt.show()

        # newmask is the mask image I manually labelled
        newmask = cv2.imread(self.directory + filename1,0)

        # whereever it is marked white (sure foreground), change mask=1
        # whereever it is marked black (sure background), change mask=0
        mask[newmask == 0] = 0
        mask[newmask == 255] = 1

        mask, bgdModel, fgdModel = cv2.grabCut(img,mask,None,bgdModel,fgdModel,5,cv2.GC_INIT_WITH_MASK)

        mask = np.where((mask==2)|(mask==0),0,1).astype('uint8')
        img = img*mask[:,:,np.newaxis]
        plt.imshow(img),plt.colorbar(),plt.show()

if __name__ == "__main__":
    boesidewalkAI = BOESidewalkAI(directory='C:/Users/pdmuser/PycharmProjects/BOESidewalk/images/GoPro/')
    boesidewalkAI.edgeDection('images.jfif')
    # boesidewalkAI.segBinary('test.jpg')
    # boesidewalkAI.segGrabCut('images.jfif', 'images2.jpg')
