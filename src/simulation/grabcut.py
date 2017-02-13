'''
===============================================================================

Key '0' - To select areas of sure background
Key '1' - To select areas of sure foreground
Key '2' - To select areas of probable background
Key '3' - To select areas of probable foreground

Key 'n' - To update the segmentation
Key 'r' - To reset the setup
Key 's' - To save the results
===============================================================================
'''

import numpy as np
import cv2
import sys
from pensive.client import PensiveClient

class GrabObject():

    global BLUE, RED, GREEN, BLACK, WHITE
    #global BRAW_BG, DRAW_FG, DRAW_PR_FG, DRAW_PR_BG
    #global self.img,img2,drawing,value,mask,rectangle,rect,rect_or_mask,ix,iy,rect_over

    BLUE = [255,0,0]        # rectangle color
    RED = [0,0,255]         # PR BG
    GREEN = [0,255,0]       # PR FG
    BLACK = [0,0,0]         # sure BG
    WHITE = [255,255,255]   # sure FG

    DRAW_BG = {'color' : BLACK, 'val' : 0}
    DRAW_FG = {'color' : WHITE, 'val' : 1}
    DRAW_PR_FG = {'color' : GREEN, 'val' : 3}
    DRAW_PR_BG = {'color' : RED, 'val' : 2}

    def __init__(self, pic=None, store=None):
        
        self.store = store or PensiveClient().default()

        self.DRAW_BG = {'color' : BLACK, 'val' : 0}
        self.DRAW_FG = {'color' : WHITE, 'val' : 1}
        self.DRAW_PR_FG = {'color' : GREEN, 'val' : 3}
        self.DRAW_PR_BG = {'color' : RED, 'val' : 2}

        # setting up flags
        self.rect = (0,0,1,1)
        self.drawing = False         # flag for drawing curves
        self.rectangle = False       # flag for drawing rect
        self.rect_over = False       # flag to check if rect drawn
        self.rect_or_mask = 100      # flag for selecting rect or mask mode
        self.value = self.DRAW_FG         # drawing initialized to FG
        self.thickness = 3           # brush thickness

       

    def run(self):
        #self.img = cv2.imread(self.filename)
        self.img = self.store.get('/camera/camera1/color_image')
        self.img2 = self.img.copy()                               # a copy of original image
        self.mask = np.zeros(self.img.shape[:2],dtype = np.uint8) # mask initialized to PR_BG
        self.output = np.zeros(self.img.shape,np.uint8)           # output image to be shown

        self.rect = (0,0,1,1)
        self.drawing = False
        self.rectangle = False
        self.rect_or_mask = 100
        self.rect_over = False
        self.value = self.DRAW_FG

        # input and output windows
        cv2.namedWindow('output')
        cv2.namedWindow('input')
        cv2.setMouseCallback('input',self._onmouse)
        cv2.moveWindow('input',self.img.shape[1]+10,90)

        while(1):

            cv2.imshow('output',self.output)
            cv2.imshow('input',self.img)
            k = cv2.waitKey(1)

            # key bindings
            if k == 27:         # esc to exit
                break
            elif k == ord('0'): # BG drawing
                print(" mark background regions with left mouse button \n")
                self.value = self.DRAW_BG
            elif k == ord('1'): # FG drawing
                print(" mark foreground regions with left mouse button \n")
                self.value = self.DRAW_FG
            elif k == ord('2'): # PR_BG drawing
                self.value = self.DRAW_PR_BG
            elif k == ord('3'): # PR_FG drawing
                self.value = self.DRAW_PR_FG
            elif k == ord('s'): # save image
                print "key is s"
                self.bar = np.zeros((self.img.shape[0],5,3),np.uint8)
                self.res = np.hstack((self.img2,self.bar,self.img,self.bar,self.output))
                #cv2.imwrite('grabcut_output.png',self.res)
                self.store.put('/camera/camera1/mask', self.output)
                print(" Result saved to database \n")
            elif k == ord('r'): # reset everything
                print("resetting \n")
                self.rect = (0,0,1,1)
                self.drawing = False
                self.rectangle = False
                self.rect_or_mask = 100
                self.rect_over = False
                self.value = self.DRAW_FG
                self.img = self.img2.copy()
                self.mask = np.zeros(self.img.shape[:2],dtype = np.uint8) # mask initialized to PR_BG
                self.output = np.zeros(self.img.shape,np.uint8)           # output image to be shown
            elif k == ord('n'): # segment the image
                print(""" For finer touchups, mark foreground and background after pressing keys 0-3
                and again press 'n' \n""")
                if (self.rect_or_mask == 0):         # grabcut with rect
                    self.bgdmodel = np.zeros((1,65),np.float64)
                    self.fgdmodel = np.zeros((1,65),np.float64)
                    cv2.grabCut(self.img2,self.mask,self.rect,self.bgdmodel,self.fgdmodel,1,cv2.GC_INIT_WITH_RECT)
                    self.rect_or_mask = 1
                elif self.rect_or_mask == 1:         # grabcut with mask
                    self.bgdmodel = np.zeros((1,65),np.float64)
                    self.fgdmodel = np.zeros((1,65),np.float64)
                    cv2.grabCut(self.img2,self.mask,self.rect,self.bgdmodel,self.fgdmodel,1,cv2.GC_INIT_WITH_MASK)

            self.mask2 = np.where((self.mask==1) + (self.mask==3),255,0).astype('uint8')
            self.output = cv2.bitwise_and(self.img2,self.img2,mask=self.mask2)

    def _onmouse(self,event,x,y,flags,param):
        #global self.img,img2,drawing,value,mask,rectangle,rect,rect_or_mask,ix,iy,rect_over

        # Draw Rectangle
        if event == cv2.EVENT_RBUTTONDOWN:
            self.rectangle = True
            self.ix = x
            self.iy = y

        elif event == cv2.EVENT_MOUSEMOVE:
            if self.rectangle == True:
                self.img = self.img2.copy()
                cv2.rectangle(self.img,(self.ix,self.iy),(x,y),BLUE,2)
                self.rect = (min(self.ix,x),min(self.iy,y),abs(self.ix-x),abs(self.iy-y))
                self.rect_or_mask = 0

        elif event == cv2.EVENT_RBUTTONUP:
            self.rectangle = False
            self.rect_over = True
            cv2.rectangle(self.img,(self.ix,self.iy),(x,y),BLUE,2)
            self.rect = (min(self.ix,x),min(self.iy,y),abs(self.ix-x),abs(self.iy-y))
            self.rect_or_mask = 0
            print(" Now press the key 'n' a few times until no further change \n")

        # draw touchup curves

        if event == cv2.EVENT_LBUTTONDOWN:
            if self.rect_over == False:
                print("first draw rectangle \n")
            else:
                self.drawing = True
                cv2.circle(self.img,(x,y),self.thickness,self.value['color'],-1)
                cv2.circle(self.mask,(x,y),self.thickness,self.value['val'],-1)

        elif event == cv2.EVENT_MOUSEMOVE:
            if self.drawing == True:
                cv2.circle(self.img,(x,y),self.thickness,self.value['color'],-1)
                cv2.circle(self.mask,(x,y),self.thickness,self.value['val'],-1)

        elif event == cv2.EVENT_LBUTTONUP:
            if self.drawing == True:
                self.drawing = False
                cv2.circle(self.img,(x,y),self.thickness,self.value['color'],-1)
                cv2.circle(self.mask,(x,y),self.thickness,self.value['val'],-1)


#############################################################
if __name__ == '__main__':

    # print documentation
    print(__doc__)

    # Loading images
    if len(sys.argv) == 2:
        filename = sys.argv[1] # for drawing purposes
    else:
        print("No input image given, so loading default image, ../data/lena.jpg \n")
        print("Correct Usage: python grabcut.py <filename> \n")
        filename = '../data/lena.jpg'

    self.img = cv2.imread(filename)
    img2 = self.img.copy()                               # a copy of original image
    mask = np.zeros(self.img.shape[:2],dtype = np.uint8) # mask initialized to PR_BG
    output = np.zeros(self.img.shape,np.uint8)           # output image to be shown

    # input and output windows
    cv2.namedWindow('output')
    cv2.namedWindow('input')
    cv2.setMouseCallback('input',self._onmouse)
    cv2.moveWindow('input',self.img.shape[1]+10,90)

    print(" Instructions: \n")
    print(" Draw a rectangle around the object using right mouse button \n")

    while(1):

        cv2.imshow('output',output)
        cv2.imshow('input',self.img)
        k = cv2.waitKey(1)

        # key bindings
        if k == 27:         # esc to exit
            cv2.destroyAllWindows()
            break
        elif k == ord('0'): # BG drawing
            print(" mark background regions with left mouse button \n")
            value = DRAW_BG
        elif k == ord('1'): # FG drawing
            print(" mark foreground regions with left mouse button \n")
            value = DRAW_FG
        elif k == ord('2'): # PR_BG drawing
            value = DRAW_PR_BG
        elif k == ord('3'): # PR_FG drawing
            value = DRAW_PR_FG
        elif k == ord('s'): # save image
            bar = np.zeros((self.img.shape[0],5,3),np.uint8)
            res = np.hstack((img2,bar,self.img,bar,output))
            cv2.imwrite('grabcut_output.png',res)
            print(" Result saved as image \n")
        elif k == ord('r'): # reset everything
            print("resetting \n")
            rect = (0,0,1,1)
            drawing = False
            rectangle = False
            rect_or_mask = 100
            rect_over = False
            value = DRAW_FG
            self.img = img2.copy()
            mask = np.zeros(self.img.shape[:2],dtype = np.uint8) # mask initialized to PR_BG
            output = np.zeros(self.img.shape,np.uint8)           # output image to be shown
        elif k == ord('n'): # segment the image
            print(""" For finer touchups, mark foreground and background after pressing keys 0-3
            and again press 'n' \n""")
            if (rect_or_mask == 0):         # grabcut with rect
                bgdmodel = np.zeros((1,65),np.float64)
                fgdmodel = np.zeros((1,65),np.float64)
                cv2.grabCut(img2,mask,rect,bgdmodel,fgdmodel,1,cv2.GC_INIT_WITH_RECT)
                rect_or_mask = 1
            elif rect_or_mask == 1:         # grabcut with mask
                bgdmodel = np.zeros((1,65),np.float64)
                fgdmodel = np.zeros((1,65),np.float64)
                cv2.grabCut(img2,mask,rect,bgdmodel,fgdmodel,1,cv2.GC_INIT_WITH_MASK)

        mask2 = np.where((mask==1) + (mask==3),255,0).astype('uint8')
        output = cv2.bitwise_and(img2,img2,mask=mask2)

    cv2.destroyAllWindows()

