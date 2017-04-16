'''
The script resize images in a given directory by 1/sz and output the files into another directory.
'''

import os
import numpy as np
import cv2
from matplotlib import pyplot as plt
import remove_background
from remove_background import removeBackground as rb

def prop(in_dir, out_dir, sz, remove=False):
    for root, subs, files in os.walk(in_dir):
        if subs == []:
            for f in files:
                if 'png' in f or 'jpg' in f:
                    img = cv2.imread(os.path.join(root, f))
                    w,h, _ = img.shape
                    img = cv2.resize(img, (h//sz, w//sz))
                    if remove:
                        masked = rb(img)
                    else:
                        mask = img
                    dir = root.replace(in_dir,'')
                    target = os.path.join(out_dir, dir, f)
                    if not os.path.exists(os.path.join(out_dir, dir)):
                        os.mkdir(os.path.join(out_dir, dir))
                    cv2.imwrite(target, masked)
    return

if __name__ == '__main__':
    sz = 3
    in_dir ='../data/salt_test'
    out_dir = '../data/salt_test_' + str(int(3000//sz))
    if not os.path.exists(out_dir):
        os.mkdir(out_dir)
    prop(in_dir, out_dir, sz, remove=True)
    print('images are processed')
