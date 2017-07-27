import cv2
import skimage.transform
import os
import sys
def load_amazon_images(directory):

    for subdir in os.listdir(directory):
        
        path=directory+ "/" + subdir

        files=next(os.walk(path))[2] #files in the directory for the class corresponding to subdir

        for f in files:
            if f[-3:]=='png': #load the png images
                im = cv2.imread(path+'/'+f)[:,:,::-1]#.astype(np.uint8)
                if im is None:
                    continue
                resized=skimage.transform.resize(im, (512,512), preserve_range=True)
                cv2.imwrite(path+'/'+f, resized)
        print('finished loading images: '+ subdir)

if __name__ == "__main__":
    load_amazon_images(sys.argv[1])