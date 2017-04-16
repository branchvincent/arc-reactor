import numpy as np
import cv2
import skimage
from scipy.signal import savgol_filter

def removeBackground(img, kernel_size=10):
    blurred = cv2.GaussianBlur(img, (5,5), 0)
    edgeImg = np.max( np.array([ edgedetect(blurred[:,:, 0]), edgedetect(blurred[:,:, 1]), edgedetect(blurred[:,:, 2]) ]), axis=0 )
    mean = np.mean(edgeImg)
    edgeImg[edgeImg <= mean] = 0;
    edgeImg_8u = np.asarray(edgeImg, np.uint8)
    significant = findSignificantContours(img, edgeImg_8u)
    contour = significant[0]

    # Mask
    mask = edgeImg.copy()
    mask[mask > 0] = 0
    cv2.fillPoly(mask, significant, 255);
    mask = mask.astype('uint8')
    SIZE = 10
    kernel = np.ones((SIZE,SIZE),np.uint8)
    mask = cv2.erode(mask, kernel, iterations = 1)
    mask = cv2.morphologyEx(mask, cv2.MORPH_OPEN, kernel)
    mask = cv2.dilate(mask,kernel,iterations = 1)
    r, c = np.where(mask>0)
    rmin = np.min(r); rmax = np.max(r); cmin = np.min(c); cmax = np.max(c);
    #Mask the original image
    ans = cv2.bitwise_and(img,img.copy(),mask=mask)
    return ans[rmin:rmax, cmin:cmax, :]

def edgedetect (channel):
    sobelX = cv2.Sobel(channel, cv2.CV_16S, 1, 0)
    sobelY = cv2.Sobel(channel, cv2.CV_16S, 0, 1)
    sobel = np.hypot(sobelX, sobelY)
    sobel[sobel > 255] = 255;
    return sobel

    return sobel

def findSignificantContours (img, edgeImg):
    image, contours, heirarchy = cv2.findContours(edgeImg, cv2.RETR_TREE, cv2.CHAIN_APPROX_SIMPLE)

    # Find level 1 contours
    level1 = []
    for i, tupl in enumerate(heirarchy[0]):
        # Each array is in format (Next, Prev, First child, Parent)
        # Filter the ones without parent
        if tupl[3] == -1:
            tupl = np.insert(tupl, 0, [i])
            level1.append(tupl)
            
    significant = []
    tooSmall = edgeImg.size * 5 / 100 # If contour isn't covering 5% of total area of image then it probably is too small
    for tupl in level1:
        contour = contours[tupl[0]];
        area = cv2.contourArea(contour)
        if area > tooSmall:
            significant.append([contour, area])

    significant.sort(key=lambda x: x[1])
    return [x[0] for x in significant];

