import os
import sys
import numpy as np
import matplotlib.pyplot as plt
import  matplotlib.patches as mpatches

from skimage.io import imread
from skimage.color import rgb2gray
from skimage.morphology import watershed, dilation, square
from skimage.measure import label, regionprops
from skimage.filters import sobel, gaussian
from skimage.transform import resize
from scipy import ndimage as ndi

from scipy import ndimage as ndi


class rgbSegmentor():
    '''
    Takes in a rgb image and return a list of images that can be fed into deep learning for prediction
    '''
    #PARAMETERS
    def __init__(self, return_dict=True):
        '''
        initialize parameters
        '''
        self.dialation_size = 10
        self.lower_marker_bound = 0.1
        self.higher_marker_bound = 0.4
        self.connectivity = 1.5 #btw [1,2]
        self.area_threshold = 0.05
        self.obj_num = 0
        self.seg_size = 256
        self.return_dict = return_dict
        return
    
    def segment(self, img):
        '''
        segment the img
        '''
        img_gray = rgb2gray(img)
        map = sobel(img_gray)
        markers = np.zeros_like(img_gray)
        markers[img_gray <self.lower_marker_bound] = 1
        markers[img_gray >self.higher_marker_bound] = 2
        binary_seg = watershed(map, markers)
        #dilates the image to shrink the holes in the labels
        conv_img = dilation(binary_seg, square(self.dialation_size))
        #remove the holes in the label
        conv_img = ndi.binary_fill_holes(conv_img - 1)
        labeled_img, num = label(conv_img, connectivity=self.connectivity, return_num=True)
        rect_list = self.rect_bound(img, labeled_img)
        return rect_list
    
    def rect_bound(self, img, seg):
        '''
        ignore small patches and return the segmentation patches of (256,256) with 0 padding around the image
        '''
        rect_list = []
        for region in regionprops(seg):
            if region.area > (img.shape[0] * img.shape[1]) * self.area_threshold:
                self.obj_num += 1
                minr, minc, maxr, maxc = region.bbox
                rect = self.add_padding(img[minr:maxr, minc:maxc, :])
                rect_list.append(rect)
            else:
                pass

        return rect_list
            
    def add_padding(self, rect):
        bg = np.zeros([self.seg_size, self.seg_size, 3])
        if np.max([rect.shape[1], rect.shape[0]]) < self.seg_size:
            pass
        else:
            #if the max dim of image is greater than self.seg_size, it need to be resize
            if rect.shape[1] > rect.shape[0]:
                im = resize(rect, (int(self.seg_size * rect.shape[0] / rect.shape[1]), self.seg_size), preserve_range=True)
            else:
                im = resize(rect, (self.seg_size, int(self.seg_size * rect.shape[1] / rect.shape[0])), preserve_range=True)
                
        startX = int((self.seg_size - rect.shape[0]) / 2)
        startY =  int((self.seg_size - rect.shape[1]) / 2)
        bg[startX:startX + rect.shape[0], startY: startY + rect.shape[1], :] = rect[:,:,:]
        return bg

if __name__ == "__main__":
    fname = '/home/hh162/Documents/code/skimage_segmentation/items1.JPG'
    img = imread(fname)
    plt.figure()
    plt.imshow(img)
    segmentor = rgbSegmentor(return_dict=True)
    rect_list = segmentor.segment(img)
    for i, rect in enumerate(rect_list):
        plt.figure()
        plt.imshow(rect)
        print(rect.shape)
        plt.title('item' + str(i))
        plt.show()
