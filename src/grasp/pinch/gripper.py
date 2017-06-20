import numpy as np
import math
import cv2
from scipy.spatial import distance as dist
import pcl
import glob
import json
from scipy.ndimage.filters import maximum_filter as maxf2D
import time


def select_grasp(pc,masks,image,gripperOpenning=0.125,gripperWidth=0.02,jawThickness=0.01,graspingDepth=0.10,threshold=0.4,length_per_pixel=0.002,external_L=0.10,external_W=0.03):
    """
    Return a list of top down grasp locations with grasability score
    Input:
    pc: pointcloud of the original scene in a structured numpy array
    masks: A python list of masks as numpy arrays, each mask has the same dimension as the pointcloud as represent one single segment(areas where value equals True(1)), the length of the list should be equal to the number of the segments to be evaluated
    image: color aligned depth image, if provided, visualization of the grasp sites will be plotted on the color image, otherwise it will be plotted on the depth map
    gripperOpenning, gripperWidth, jawThickness: Pysical character of the parallel gripper, in m
    length_per_pixel: x, ys increment in meter from 1 pixel to the next pixel, suggest using 0.001m for a 0.3m*0.3m bin entirely enclosed in a 480*640 array
    threshold: maximum height to be considered(everything above will be considered the same height as the threshold

    Output:
    center: Coordinate where the center of the jaw edges

    *********
    *       *
    *<----->*  Opening
    *   *   *
  center=(x,y,z)

    Openning: Opening of the gripper before closing around the objects

    angle: angle of rotation, in degrees

    """
    depth_map,corner=pc2depthmap(pc,length_per_pixel,threshold)
    all_Grasp=[]
    grasps_physical=[]
    grasps_info=[]
    for mask_num,mask in enumerate(masks):
        img,segment_points,COG=mask2image(pc,mask,corner,depth_map,length_per_pixel,threshold)
        window_dimension=int(gripperOpenning/length_per_pixel/1.414)
        sampleDistance=int(0.01/length_per_pixel)

        check_depth=graspingDepth/threshold*255
        length_per_pixel_V=threshold/255
        length_per_pixel_H=length_per_pixel
        grasps=getgrasp(depth_map,img,segment_points,sampleDistance,window_dimension,check_depth,length_per_pixel_H,length_per_pixel_V,gripperWidth,jawThickness,external_L,external_W,mask_num)
        grasp_physical=getlocations(grasps,corner,length_per_pixel,threshold,COG)
        all_Grasp.append(grasps)
        grasps_physical.append(grasp_physical)
        show=depth_map.copy()*3

    for i,grasps_mask in enumerate(all_Grasp):
        for j,grasp in enumerate(grasps_mask):
            cv2.line(show, (int(grasp[0][0]),int(grasp[0][1])), (int(grasp[1][0]),int(grasp[1][1])), 255)
            grasp_info={'index':grasps_physical[i][j][8],'center':grasps_physical[i][j][0],'opening':grasps_physical[i][j][1],'tip_height':grasps_physical[i][j][2],'rotation':-grasps_physical[i][j][3],'score':grasps_physical[i][j][4],'max_Grasp_depth':grasps_physical[i][j][5],'distance_from_COG':grasps_physical[i][j][6],'Blockage_percent':grasps_physical[i][j][7]}
            grasps_info.append(grasp_info)
            print grasp_info
            #cv2.imshow("Original", show)
	    #cv2.imshow("color",image)
            k = cv2.waitKey(0)
            if k == 27:         # wait for ESC key to exit
                cv2.destroyAllWindows()
    #[(center_X,center_Y),openning,tip_height,rotation,score,max_Grasp_depth,distancefromCOG,blockage_percent]
    return grasps_info
def getlocations(grasps,corner,length_per_pixel,threshold,COG):
    grasp_list=[]
    for grasp in grasps:
        pixel_top,pixel_bottom,rotation,max_Grasp_depth,tip_height,blockage_percent,mask_num=grasp
        center=(pixel_top+pixel_bottom)/2
        center_X=corner[0]-center[0]*length_per_pixel
        center_Y=corner[1]+center[1]*length_per_pixel
        distancefromCOG=math.sqrt((center_X-COG[0])**2+(center_Y-COG[1])**2)
        tip_height=tip_height*1.0/255*threshold
        max_Grasp_depth=max_Grasp_depth*1.0/255*threshold

        displacement=pixel_top-pixel_bottom
        openning=math.sqrt(displacement[0]**2+displacement[1]**2)*length_per_pixel
        score=10-((blockage_percent-0.1)*10)**2-distancefromCOG*10
        if max_Grasp_depth<0.03:
            score-=((0.03-max_Grasp_depth)*150)**2
        grasp_list.append([(center_X,center_Y),openning,tip_height,rotation,score,max_Grasp_depth,distancefromCOG,blockage_percent,mask_num])

    return grasp_list






def getgrasp(depth_map,img,segment_points,sampleDistance,window_dimension,check_depth,length_per_pixel_H,length_per_pixel_V,gripperWidth,jawThickness,external_L,external_W,mask_num):

    #get the windows with more than 100 points
    image=img.copy()
    grasps=[]

    image[image>0]=np.uint8(1)
    rollingwindows=rolling_window(image,(window_dimension,window_dimension),asteps=(sampleDistance,sampleDistance))
    point_sum=rollingwindows.sum((3,2))

    allwindows=np.nonzero(point_sum > 500)

    for i in range(len(allwindows[0])):
        x_Start=allwindows[1][i]*sampleDistance
        y_Start=allwindows[0][i]*sampleDistance
        window=img[y_Start:y_Start+window_dimension,x_Start:x_Start+window_dimension]
        max_height=np.max(reject_outliers(window[np.nonzero(window)]))
        min_height=np.min(reject_outliers(window[np.nonzero(window)]))
        increment=int(check_depth/10.0)
        for depth_evaluating in np.arange(min_height,max_height-increment,increment):

            #create a image where only points above certain height in a sliding window is visible
            a=np.full((window_dimension,window_dimension),220,dtype="uint8")
            indices_window=(window > depth_evaluating).nonzero()
            a[indices_window]=np.uint8(60)

            cnt_exist,contour=get_contour(a)
            if cnt_exist:
                rect = cv2.minAreaRect(contour)
                (center_x_window,center_y_window),(length_window,width_window),angle_window=rect
                rect=((center_x_window+x_Start,center_y_window+y_Start),(length_window,width_window),angle_window)
                grasp_found,pixel_top,pixel_bottom,rotation,max_Grasp_depth,blockage_percent=rate_grasp(depth_map,rect,depth_evaluating,length_per_pixel_H,length_per_pixel_V,gripperWidth,jawThickness,check_depth,external_L,external_W)
                if grasp_found:
                    grasps.append((pixel_top,pixel_bottom,rotation,max_Grasp_depth,depth_evaluating-max_Grasp_depth,blockage_percent,mask_num))
    return grasps





def rate_grasp(depth_map,rect,depth,pixel_length_horizontal,pixel_length_vertical,gripperWidth,jawThickness,max_Depth,external_L,external_W):
    """
    rate a grasp at the selected minimum area rectangle position
    """

    (center_x,center_y),(length,width),rotation=rect
    #rotate the depth map so that the grasping position will be horizontal in the depth map
    #the gripper position being evaluated is at the center of the minimum bounding rectangle, one holding in the
    if length<width:
        rotation=90+rotation
        #switch up length and width
        a=length
        length=width
        width=a

    M = cv2.getRotationMatrix2D((center_x, center_y), rotation, 1.0)

    (h, w) = depth_map.shape[:2]
    rotated_map = cv2.warpAffine(depth_map, M, (w, h))
    tip_length=int(gripperWidth/pixel_length_horizontal)
    tip_width=int(jawThickness/pixel_length_horizontal)

    #check if the gripper can be placed at the height evaluating
    #allow the gripper to extend 2cm to check if there is room to grasp
    extend=int(0.02/pixel_length_horizontal)
    checkarea=rotated_map[int(center_y-width/2-tip_width-extend):int(center_y+width/2+tip_width+extend),int(center_x-tip_length/2):int(center_x+tip_length/2)]

    try:
        M=tip_width

        P,Q=checkarea.shape
        maxes = maxf2D(checkarea, size=(M,Q))
        maxes = maxes[M//2:(M//2)+P-M+1, Q//2:(Q//2)+1]

        #maxes=rolling_window(checkarea,(tip_width,checkarea.shape[1])).max((2,3))
        maxes=maxes.flatten()

    except:
        maxes=np.array([0])

    max_withinfingers=np.amax(maxes)
    max_graspDepth=max_Depth-(max_withinfingers-depth)
    #if can reach further by 1cm at the edge
    reachable=max_graspDepth>0.01/pixel_length_vertical
    #if the gripper can reach this grasp depth
    if reachable:
        #check for area taller the depth evaluating
        max_Tall=maxes.copy()
        max_Tall[max_Tall>depth]=0
        #check for low area that is 5mm below the depth evaluating(handle)
        max_Low=maxes.copy()
        max_Low[max_Low<depth-0.01/pixel_length_vertical]=0
        #get the index range for tall areas and low areas
        location_tall=zero_runs(max_Tall)
        location_low=zero_runs(max_Low)
        #get the index range for tall areas and low areas
        location_tall=zero_runs(max_Tall)
        location_low=zero_runs(max_Low)


        if len(location_low)>1 and len(location_tall)>0:
            if len(location_tall)>1:
                #save only the largest conseutive long area
                to_Filter=location_tall.copy()
                location_tall=[]
                max_length=0
                for tall_area in to_Filter:
                    if tall_area[1]-tall_area[0]>max_length:
                        max_length=tall_area[1]-tall_area[0]
                        location_tall=[tall_area]
            #only consider a handle if the thickness of the handle is greater than 5mm
            if location_tall[0][1]-location_tall[0][0]>0.005/pixel_length_horizontal:
                handle_top=0
                handle_bottom=0
                handle_index_top=0
                handle_index_bottom=len(maxes)
                handle_top_bottom_index=0
                for handle in location_low:
                    #if the gripper can extend freely from the closing position outward
                    if handle[1]<location_tall[0][0] and handle[0]>=handle_index_top and handle[1]-handle[0]>0.01/pixel_length_horizontal:
                        handle_index_top=handle[0]
                        handle_top_bottom_index=handle[1]
                        handle_top+=1
                    if handle[0]>location_tall[0][1] and handle[0]<=handle_index_bottom and handle[1]-handle[0]>0.01/pixel_length_horizontal:
                        handle_index_bottom=handle[0]
                        handle_bottom+=1

                if handle_top*handle_bottom>0:

                    max_Grasp_depth=min(max_graspDepth,depth-max(max(maxes[handle_top_bottom_index-int(0.007/pixel_length_horizontal):handle_top_bottom_index-int(0.002/pixel_length_horizontal)]),max(maxes[handle_index_bottom+int(0.002/pixel_length_horizontal):handle_index_bottom+int(0.007/pixel_length_horizontal)])))
                    #max_Grasp_depth*=pixel_length_vertical


                    pixel_top_rotated=np.array([int(center_x),int(center_y-width/2-extend-int(0.005/pixel_length_horizontal)+handle_top_bottom_index)])
                    pixel_bottom_rotated=np.array([int(center_x),int(center_y-width/2-extend+int(0.005/pixel_length_horizontal)+handle_index_bottom)])

                    blockage_percent=getblockage(rotated_map,max_withinfingers,(pixel_top_rotated+pixel_bottom_rotated)/2,pixel_length_horizontal,external_L,external_W)
                    displacement_top=np.array([0,pixel_top_rotated[1]-center_y])
                    displacement_bottom=np.array([0,pixel_bottom_rotated[1]-center_y])

                    theta=np.radians(-rotation)
                    M_inv = np.array([[math.cos(theta), -math.sin(theta)],[math.sin(theta),  math.cos(theta)]])
                    pixel_top_original=np.array([center_x, center_y])+np.dot(displacement_top, M_inv)
                    pixel_bottom_original=np.array([center_x, center_y])+np.dot(displacement_bottom, M_inv)

                    return (True,pixel_top_original,pixel_bottom_original,-rotation,max_Grasp_depth,blockage_percent)


    return (False,None,None,None,None,None)

def getblockage(rotated_map,max_withinfingers,center,pixel_length,external_L,external_W):
    length=external_L/pixel_length
    width=external_W/pixel_length

    checkarea=rotated_map[center[1]-int(length/2):center[1]+int(length/2),center[0]-int(width/2):center[0]+int(width/2)]
    area_height,area_length=checkarea.shape
    if area_height*area_length*1.0/length/width<0.8:
        return 1
    else:
        blockage=np.nonzero(checkarea > max_withinfingers)
        block_percentage=len(blockage[0])*1.0/area_height/area_length
        return block_percentage





def zero_runs(a):
    # Create an array that is 1 where a is 0, and pad each end with an extra 0.
    iszero = np.concatenate(([0], np.equal(a, 0).view(np.int8), [0]))
    absdiff = np.abs(np.diff(iszero))
    # Runs start and end where absdiff is 1.
    ranges = np.where(absdiff == 1)[0].reshape(-1, 2)
    return ranges


def get_contour(image):
    """
    for an given gary scale image, find the contour points of the largest contour
    input: 2D numpy array in with unsiged 8 bit RGB value
    output: contour pixels in the image frame in a opencv contour format
    """
    cnt=None
    isContour=False

    #gray = cv2.GaussianBlur(image, (7, 7), 0)

    # perform edge detection, then perform a dilation + erosion to
    # close gaps in between object edges
    edged = cv2.Canny(image, 70, 240)
    edged = cv2.dilate(edged, None, iterations=1)
    edged = cv2.erode(edged, None, iterations=1)

    #save a copy of image after contour, we will later use to obtain all points enclosed in the contour
    cnts = cv2.findContours(edged.copy(), cv2.RETR_EXTERNAL,cv2.CHAIN_APPROX_NONE)[1]
    if len(cnts)>0:
        isContour=True

        cnt=max(cnts,key=cv2.contourArea)
    return (isContour,cnt)

def mask2image(pc,mask,corner,depth_map,length_per_pixel,threshold):
    """
    Conver the masked area on the pointcloud into 2d image with white background and black only on the object of interest
    """
    x_max,y_min=corner
    segment_npy= pc[mask.nonzero()]

    #remove the all 0 points from the segment

    mask_zero = np.all(segment_npy == 0, axis=1)
    segment_npy = segment_npy[~mask_zero]

    COG=np.mean(segment_npy, axis=0)
    num_pixels_Y,num_pixels_X=depth_map.shape
    img=np.full((num_pixels_Y,num_pixels_X),0,dtype="uint8")
    for point in segment_npy:
        x=(x_max-point[0])/length_per_pixel
        y=(point[1]-y_min)/length_per_pixel

        x=int(min(max(0,x),num_pixels_X-1))
        y=int(min(max(0,y),num_pixels_Y-1))
        img[y][x]=np.uint8(point[2]*1.00/threshold*255)

    return (img,segment_npy,COG)





def pc2depthmap(pointcloud,length_per_pixel,threshold):
    """
    Convert a pointcloud(Structured or unstructured,2D or 3D) into a depth map where each pixel length is specified by length per pixel

    Input:

    pointcloud:pointcloud(Structured or unstructured,2D or 3D) in a numpy array

    length_per_pixel: x, y increment in meter from 1 pixel to the next pixel, suggest using 0.001m for a 0.3m*0.3m bin entirely enclosed in a 480*640 array

    Output:
    depth_map: A de-noised depth map of the dimension x=(x_max-x_min)/length_per_pixel,y=(y_max-y_min)/length_per_pixel, where (x_min,y_min) x,y value for the first pixel in the depth map
    (x_min,y_max) x,y value for the first pixel in the depth map,depth image is arranged as the robot tool is looking over it

    ******************************************************************
    *(x_max,y_min)                                                   *
    *                                                                *
    *                                                                *
    *                                                                *
    *                                                                *
    *                     ** (gripper)                               *
    *                     **                                         *
    *                     **                           (x_min,y_max) *
    ******************************************************************
                          **
                        *robot*
                        *******


    """
    #use a hard threshold to filter out noises above the threshold
    #flatten and reshape the array to a list of points
    pointcloud=pointcloud.flatten()
    pointcloud=np.reshape(pointcloud,(pointcloud.size/3,3))
    #get the boundary of the points in pointcloud's coordinate

    #filter the x and y readings of the pointcloud
    x_raw=pointcloud[:,0]
    y_raw=pointcloud[:,1]
    x_filtered=reject_outliers(x_raw, m = 6)
    y_filtered=reject_outliers(y_raw, m = 6)

    x_min=np.amin(x_filtered)
    x_max=np.amax(x_filtered)
    y_min=np.amin(y_filtered)
    y_max=np.amax(y_filtered)

    num_pixels_X=int((x_max-x_min)/length_per_pixel)
    num_pixels_Y=int((y_max-y_min)/length_per_pixel)

    #Initialize an empty depth map
    depth_map=np.full((num_pixels_Y,num_pixels_X),0,dtype="uint8")
    #filter out high and low points
    pointcloud=pointcloud[pointcloud[:,2]>0]
    pointcloud=pointcloud[pointcloud[:,2]<threshold]
    #fill in the depth map
    for point in pointcloud:
        x=int((x_max-point[0])/length_per_pixel)
        y=int((point[1]-y_min)/length_per_pixel)

        try:
            depth_map[y][x]=max(depth_map[y][x],np.uint8(point[2]*1.00/threshold*255))
        except:
            continue
    #de-noise the raw depth map using median filter
    depth_map=cv2.fastNlMeansDenoising(depth_map,None,10,7,21)
    #convert the denoised depth_map back to the original scale

    return (depth_map,(x_max,y_min))



def reject_outliers(data, m = 6):
    d = np.abs(data - np.median(data))
    mdev = np.median(d)
    s = d/mdev if mdev else 0.
    try:
    	return data[s<m]
    except:
	    return 0

def rolling_window(array, window=(0,), asteps=None, wsteps=None, axes=None, toend=True):
    """Create a view of `array` which for every point gives the n-dimensional
    neighbourhood of size window. New dimensions are added at the end of
    `array` or after the corresponding original dimension.

    Parameters
    ----------
    array : array_like
        Array to which the rolling window is applied.
    window : int or tuple
        Either a single integer to create a window of only the last axis or a
        tuple to create it for the last len(window) axes. 0 can be used as a
        to ignore a dimension in the window.
    asteps : tuple
        Aligned at the last axis, new steps for the original array, ie. for
        creation of non-overlapping windows. (Equivalent to slicing result)
    wsteps : int or tuple (same size as window)
        steps for the added window dimensions. These can be 0 to repeat values
        along the axis.
    axes: int or tuple
        If given, must have the same size as window. In this case window is
        interpreted as the size in the dimension given by axes. IE. a window
        of (2, 1) is equivalent to window=2 and axis=-2.
    toend : bool
        If False, the new dimensions are right after the corresponding original
        dimension, instead of at the end of the array. Adding the new axes at the
        end makes it easier to get the neighborhood, however toend=False will give
        a more intuitive result if you view the whole array.

    Returns
    -------
    A view on `array` which is smaller to fit the windows and has windows added
    dimensions (0s not counting), ie. every point of `array` is an array of size
    window.

    Examples
    --------
    >>> a = np.arange(9).reshape(3,3)
    >>> rolling_window(a, (2,2))
    array([[[[0, 1],
             [3, 4]],
            [[1, 2],
             [4, 5]]],
           [[[3, 4],
             [6, 7]],
            [[4, 5],
             [7, 8]]]])

    Or to create non-overlapping windows, but only along the first dimension:
    >>> rolling_window(a, (2,0), asteps=(2,1))
    array([[[0, 3],
            [1, 4],
            [2, 5]]])

    Note that the 0 is discared, so that the output dimension is 3:
    >>> rolling_window(a, (2,0), asteps=(2,1)).shape
    (1, 3, 2)

    This is useful for example to calculate the maximum in all (overlapping)
    2x2 submatrixes:
    >>> rolling_window(a, (2,2)).max((2,3))
    array([[4, 5],
           [7, 8]])

    Or delay embedding (3D embedding with delay 2):
    >>> x = np.arange(10)
    >>> rolling_window(x, 3, wsteps=2)
    array([[0, 2, 4],
           [1, 3, 5],
           [2, 4, 6],
           [3, 5, 7],
           [4, 6, 8],
           [5, 7, 9]])
    """
    array = np.asarray(array)
    orig_shape = np.asarray(array.shape)
    window = np.atleast_1d(window).astype(int) # maybe crude to cast to int...

    if axes is not None:
        axes = np.atleast_1d(axes)
        w = np.zeros(array.ndim, dtype=int)
        for axis, size in zip(axes, window):
            w[axis] = size
        window = w

    # Check if window is legal:
    if window.ndim > 1:
        raise ValueError("`window` must be one-dimensional.")
    if np.any(window < 0):
        raise ValueError("All elements of `window` must be larger then 1.")
    if len(array.shape) < len(window):
        raise ValueError("`window` length must be less or equal `array` dimension.")

    _asteps = np.ones_like(orig_shape)
    if asteps is not None:
        asteps = np.atleast_1d(asteps)
        if asteps.ndim != 1:
            raise ValueError("`asteps` must be either a scalar or one dimensional.")
        if len(asteps) > array.ndim:
            raise ValueError("`asteps` cannot be longer then the `array` dimension.")
        # does not enforce alignment, so that steps can be same as window too.
        _asteps[-len(asteps):] = asteps

        if np.any(asteps < 1):
             raise ValueError("All elements of `asteps` must be larger then 1.")
    asteps = _asteps

    _wsteps = np.ones_like(window)
    if wsteps is not None:
        wsteps = np.atleast_1d(wsteps)
        if wsteps.shape != window.shape:
            raise ValueError("`wsteps` must have the same shape as `window`.")
        if np.any(wsteps < 0):
             raise ValueError("All elements of `wsteps` must be larger then 0.")

        _wsteps[:] = wsteps
        _wsteps[window == 0] = 1 # make sure that steps are 1 for non-existing dims.
    wsteps = _wsteps

    # Check that the window would not be larger then the original:
    if np.any(orig_shape[-len(window):] < window * wsteps):
        raise ValueError("`window` * `wsteps` larger then `array` in at least one dimension.")

    new_shape = orig_shape # just renaming...

    # For calculating the new shape 0s must act like 1s:
    _window = window.copy()
    _window[_window==0] = 1

    new_shape[-len(window):] += wsteps - _window * wsteps
    new_shape = (new_shape + asteps - 1) // asteps
    # make sure the new_shape is at least 1 in any "old" dimension (ie. steps
    # is (too) large, but we do not care.
    new_shape[new_shape < 1] = 1
    shape = new_shape

    strides = np.asarray(array.strides)
    strides *= asteps
    new_strides = array.strides[-len(window):] * wsteps

    # The full new shape and strides:
    if toend:
        new_shape = np.concatenate((shape, window))
        new_strides = np.concatenate((strides, new_strides))
    else:
        _ = np.zeros_like(shape)
        _[-len(window):] = window
        _window = _.copy()
        _[-len(window):] = new_strides
        _new_strides = _

        new_shape = np.zeros(len(shape)*2, dtype=int)
        new_strides = np.zeros(len(shape)*2, dtype=int)

        new_shape[::2] = shape
        new_strides[::2] = strides
        new_shape[1::2] = _window
        new_strides[1::2] = _new_strides

    new_strides = new_strides[new_shape != 0]
    new_shape = new_shape[new_shape != 0]

    return np.lib.stride_tricks.as_strided(array, shape=new_shape, strides=new_strides)


# masks=[]
# image=cv2.imread("bin.png")
# pointcloud=np.load("pc.npy")
# pointcloud=pointcloud.astype(np.float32)

# try:
#     for filename in glob.iglob('perception/*.npy'):
#         mask=np.load('%s' % filename)
#         masks.append(mask)
#         print "doing mask"
# except:
#     print "no numpy masks provided,converting pcd files to masks"

# if len(masks)==0:

#     for filename in glob.iglob('segments/*.pcd'):
#         seg_pcd=pcl.load('%s' % filename)
#         seg_pcd=np.asarray(seg_pcd)
#         seg_pixels_pc=[]
#         for coordinate in seg_pcd:
#             indice = np.where(np.all(pointcloud == coordinate, axis=-1))

#             try:
#                 seg_pixels_pc.append([int(indice[1]),int(indice[0])])
#             except:
#                 continue

#         mask=np.full((480,640),0,dtype="uint8")
#         for index in seg_pixels_pc:
#             mask[index[1]][index[0]]=np.uint8(1)
#         np.save('%s.npy' % filename,mask)
#         masks.append(mask)
#         print "doing pcd"

# #mask=np.full((480,640),1,dtype="uint8")
# #masks.append(mask)
# select_grasp(pointcloud,masks,image,gripperOpenning=0.125,gripperWidth=0.02,jawThickness=0.01,length_per_pixel=0.002)


