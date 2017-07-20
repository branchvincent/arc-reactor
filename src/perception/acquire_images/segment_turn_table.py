import time
import os
import fcntl
import sys
import re
import cv2
import numpy as np
import math

def process_directory(root_dir, folder, seg_folder, dict_of_times_item_has_been_seen):
    skip = 4
    m = re.search(r'\d+', directory).span()[0]
    item_name = folder[0:m]
    fnames = sorted(os.listdir(root_dir + "/" + folder))
    for i in range(1,len(fnames),skip):

        if "lockfile" in fnames[i]:
            continue

        #read in the image
        im = cv2.imread(root_dir + "/" + folder + "/" + fnames[i])
        #convert to lab
        im_lab = cv2.cvtColor(im, cv2.COLOR_BGR2LAB)
        
        #skip blank images
        if np.all(im == 0):
            continue
        
        mask = mask_fun(im_lab)
        
        if mask is None:
            continue
            
        mask = mask.astype('uint8')
        if fnames[i].startswith('1'):
            mask[0:40,:] = 0
        
    
        masked_bgr = np.zeros(im.shape).astype('uint8')
        masked_bgr[:,:,0] = im[:,:,0]*mask
        masked_bgr[:,:,1] = im[:,:,1]*mask
        masked_bgr[:,:,2] = im[:,:,2]*mask
        
        #write it out
        cv2.imwrite(seg_folder + "/" + item_name + "/" + str(dict_of_times_item_has_been_seen[item_name]) + ".png",masked_bgr)
        
        dict_of_times_item_has_been_seen[item_name] += 1

    return dict_of_times_item_has_been_seen

def mask_fun(input_lab_image):

    meanL0 = input_lab_image[400:410,0:10,0].mean()
    meanA0 = input_lab_image[400:410,0:10,1].mean()
    meanB0 = input_lab_image[400:410,0:10,2].mean()
    
    gs = cv2.ximgproc.segmentation.createGraphSegmentation()
    gs.setSigma(1.8)
    gs.setK(840)
    gs.setMinSize(3760)
    
    l_img = gs.processImage(input_lab_image)

    max_dist = -1
    best_ind = -1
    for i in range(l_img.max()+1):
        m = l_img == i #mask of cc
        tmp = input_lab_image[np.where(m)] - [meanL0, meanA0, meanB0]
        tmp = tmp[:,0]**2 + tmp[:,1]**2 + tmp[:,2]**2
        tmp = np.sqrt(tmp)
        mean_dist = tmp.mean()
        
        dist_from_mid = math.sqrt((np.where(m)[0].mean()- 240)**2 + (np.where(m)[1].mean() - 320)**2)
        if mean_dist > max_dist and dist_from_mid < 100:
            y,x = np.where(m)
            xy = np.vstack((x,y))
            x,y,w,h = cv2.boundingRect(xy.transpose())
            if w < 600 or h < 400:
                max_dist = mean_dist
                best_ind = i
    
    if best_ind != -1:
        return l_img == best_ind
    else:
        return l_img == 2
        # return None

if __name__ == '__main__':

    #get the directory where all images will be placed
    images_dir = sys.argv[1]

    #list of directories we've processed
    list_of_proc_dirs = []

    #dictionary of times item has been seen. item name -> number
    dict_of_times_item_has_been_seen = {}

    #create a folder for the segmented output
    try:
        os.mkdir("segmented")
    except:
        pass

    #monitor this directory for new folders
    try:
        while(True):
            dirs_in_img = os.listdir(images_dir)
            for directory in dirs_in_img:
                if not directory in list_of_proc_dirs:

                    #try to get a lock on the lockfile in the directory to make sure the folder is done being written to
                    with open(images_dir + "/" + directory + "/" + 'lockfile', 'w') as fp:
                        try:
                            #try to lock the file
                            fcntl.lockf(fp, fcntl.LOCK_WRITE | fcntl.LOCK_EX)
                        except:
                            #file is locked, directory is not ready, continue
                            # print("File was locked")
                            continue
                        #unlock
                        fcntl.lockf(fp.fileno(), fcntl.LOCK_UN)

                    #process this directory
                    print("Processing {}".format(directory))

                    #what item is this? get location of first digit
                    m = re.search(r'\d+', directory).span()[0]
                    item_name = directory[0:m]
                    if not item_name in dict_of_times_item_has_been_seen:
                        dict_of_times_item_has_been_seen[item_name] = 0
                        #make an output directory
                        os.mkdir("segmented/" + item_name)

                    num_imgs_before = dict_of_times_item_has_been_seen[item_name]
                    dict_of_times_item_has_been_seen = process_directory(images_dir, directory, "segmented", dict_of_times_item_has_been_seen)
                    num_imgs_after = dict_of_times_item_has_been_seen[item_name]

                    print("Processed {} images".format(num_imgs_after-num_imgs_before))
                    list_of_proc_dirs.append(directory)
            time.sleep(0.1)
    except KeyboardInterrupt:
        #quit on cntrl c
        pass