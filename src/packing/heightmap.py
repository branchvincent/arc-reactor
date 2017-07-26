import numpy as np
from scipy import ndimage
from scipy.ndimage.filters import maximum_filter as maxf2D
import scipy.ndimage.filters as filters
import scipy.ndimage.morphology as morphology
import cv2
import random
import math
from numpy import unravel_index
import pcl
import glob
import time
import pcl


def pack(bins,pointcloud,BBs,ee_pos,layer_map=None,margin=0.00,max_height=0.4,pixel_length=0.002,rotate=False,stability=False,layer=False,minimum_dimension=0.04):


    """
    pack a item into a selected number of bins and return the center coordinate of the objects when packed

    Input:

    bins: structured or unstructured pointclouds of each bins the item might be packed into as a list of numpy arrays

    pointcloud: pointcloud of the item picked at the inspection station in a numpy array

    ee_pos:position of the end effector at the inspection station in (x,y,z)

    layer_map: A map of the same dimension as the height map that store the layer information from previous placement in the bin, given as a python list of numpy array,  if layer_map equals none, will initialize a layer_map.

    margin: how much margin to be left at the edge of the object in meter

    max_height: the maximum height used for drawing the depth map and the max height to be considered for placing

    length_per_pixel: x, y increment in meter from 1 pixel to the next pixel, suggest using 0.001m for a 0.3m*0.3m bin entirely enclosed in a 480*640 array

    BBs: a list of bounding box that enclose only the area of interest for packing. Each bounding box is a python list of 4 cornor points, e.g.  [[-0.1715, -0.1395, 0], [0.1715, 0.1395, 0.121],[0.1715, -0.1395, 0.121],[-0.1715, 0.1395, 0.121]] , the length of the bounding boxs should be the same length as the number of bins to be considered

    rotate: allow rotation of the object other than 0 and 90 degrees, note that by default rotation is off, however, when the dimension of object doesn't fit inside the storage system given, the rotate is be set to True regardless of the option given to the pack function



    Output:
    location: center coordinate of for the tool. In a python list [x,y,z], if no proper placement was found, return None
    Orientation: rotation angle in degrees(item rotated contour clock wise therefore all degrees are negative)
    layer_map_updated: A updated layer map after the placement of the item
    """
    print time.time()
    depth_maps=[]
    minimum_height=[]
    locations=[]
    orientations=[]
    visuals=[]
    cornor_points=[]
    layer_map_candidates=[]
    orders=[]
    #delete this

    item,offset,rotate_angle=get_object_dimension(pointcloud,pixel_length,ee_pos,minimum_dimension)

    #initialize the layer_map if one doesn't exsist
    #or if the length of the layer map is different from the number of bins current evaluating(likely forgot to clear layer_map after one run)
    if layer_map is None or len(layer_map)!=len(bins):
        layer_map=[]
        for i in range(len(bins)):
            layer_map.append([])

    for indice,bin in enumerate(bins):
        depth_map,cornor_point,bin_layer_map,z_min,z_max=pc2depthmap(indice,bin,max_height,pixel_length,BBs[indice],layer_map[indice])
        layer_map[indice]=bin_layer_map
        depth_maps.append(depth_map)
        cornor_points.append(cornor_point)

    for order,depth_map in enumerate(depth_maps):
        index,orientation,height,visualization,bin_layer_map=find_placement(order,depth_map,item,margin,pixel_length,rotate,stability,layer,layer_map[order],max_height)

        if height!=255:
            orders.append(order)
            z=height/255.0*max_height+item[2]+z_min
            x=cornor_points[order][0]+index[0]*pixel_length
            y=cornor_points[order][1]-index[1]*pixel_length
            minimum_height.append(height)
            visuals.append(visualization)
            locations.append([x,y,z])
            layer_map_candidates.append(bin_layer_map)
            orientations.append(orientation)
    if len(minimum_height)>0:
        #get the placement that result in the lowest stack height
        best_index=minimum_height.index(min(minimum_height))
        location=locations[best_index]
        orientation=orientations[best_index]
        image_show=visuals[best_index]
        layermap2update=layer_map_candidates[best_index]
        order=orders[best_index]
        layer_map[order]=layermap2update

        #make sure the bottom of the object is less than 20cm away from the bottom of the bin

        tool_location=get_location(location,-orientation-rotate_angle,offset)

        print (order, location,tool_location,-orientation-rotate_angle)

        # cv2.imshow("denoised",image_show*3)
        # k = cv2.waitKey(0)
        # if k == 27:         # wait for ESC key to exit
        #     cv2.destroyAllWindows()
        print time.time()
        return (order, tool_location,-orientation-rotate_angle,layer_map)

    else:
        print "Try placement with reduced dimension and remove margin"
        item[0]=max(pixel_length,item[0]-0.01)
        item[1]=max(pixel_length,item[1]-0.01)

        for order,depth_map in enumerate(depth_maps):
            index,orientation,height,visualization,bin_layer_map=find_placement(order,depth_map,item,0,pixel_length,rotate,stability,layer,layer_map[order],max_height)

            if height!=255:
                orders.append(order)
                z=height/255.0*max_height+item[2]+z_min
                x=cornor_points[order][0]+index[0]*pixel_length
                y=cornor_points[order][1]-index[1]*pixel_length
                minimum_height.append(height)
                visuals.append(visualization)
                locations.append([x,y,z])
                layer_map_candidates.append(bin_layer_map)
                orientations.append(orientation)
        if len(minimum_height)>0:
            #get the placement that result in the lowest stack height
            best_index=minimum_height.index(min(minimum_height))
            location=locations[best_index]
            orientation=orientations[best_index]
            image_show=visuals[best_index]
            layermap2update=layer_map_candidates[best_index]
            order=orders[best_index]
            layer_map[order]=layermap2update


            tool_location=get_location(location,-orientation-rotate_angle,offset)

            print (order, location,tool_location,-orientation-rotate_angle)
            # cv2.imshow("denoised",image_show*3)
            # k = cv2.waitKey(0)
            # if k == 27:         # wait for ESC key to exit
            #     cv2.destroyAllWindows()

            return (order, tool_location,-orientation-rotate_angle,layer_map)

        else:
            print "Item too large, place in the largest bin possible"
            BB_idx,largestBBcenter,rotation=placeOversized(BBs,item)
            tool_location=get_location(largestBBcenter,-rotation-rotate_angle,offset)
            return (BB_idx,tool_location,-rotation-rotate_angle,layer_map)

def placeOversized(BBs,item):
    max_bin=None
    max_index=0
    location=None
    rotate=0
    for index,BB in enumerate(BBs):

        x_min=min(BB[0][0], BB[1][0],BB[2][0], BB[3][0])
        x_max=max(BB[0][0], BB[1][0],BB[2][0], BB[3][0])
        y_min=min(BB[0][1], BB[1][1],BB[2][1], BB[3][1])
        y_max=max(BB[0][1], BB[1][1],BB[2][1], BB[3][1])
        length=x_max-x_min
        width=y_max-y_min
        if max_bin is None or length*width>max_bin:
            max_bin=length*width
            max_index=index
    x=(BBs[max_index][0][0]+BBs[max_index][1][0]+BBs[max_index][2][0]+BBs[max_index][3][0])/4
    y=(BBs[max_index][0][1]+BBs[max_index][1][1]+BBs[max_index][2][1]+BBs[max_index][3][1])/4
    z=max(BBs[max_index][0][2], BBs[max_index][1][2],BBs[max_index][2][2], BBs[max_index][3][2])

    if (length-width)*(item[0]-item[1])<0:
        rotate=90
    return (max_index,[x,y,z],rotate)


def get_location(center,angle,offset):
    """
    get the tool position off the center
    """

    theta=math.radians(angle)

    X=center[0]+offset[0]*math.cos(theta) - offset[1]*math.sin(theta)
    Y=center[1]+(offset[0]*math.sin(theta) + offset[1]*math.cos(theta))
    Z=center[2]+offset[2]

    return [X,Y,Z]



def get_object_dimension(pointcloud,pixel_length,ee_pos,minimum_D):
    """
    use a structured or unstructured pointcloud collected at the inspection station to estimate object goemetry

    input:
    pointcloud: structured or unstructured pointcloud of object in a numpy array
    output:
    boundingbox: x,y,z of the object bounding volume in a python list [length,width,height]
    """
    #use a hard threshold to filter out noises above the threshold
    #flatten and reshape the array to a list of points
    pointcloud=pointcloud.flatten()
    pointcloud=np.reshape(pointcloud,(pointcloud.size/3,3)).astype(np.float32)
    p=pcl.PointCloud(pointcloud)
    fil = p.make_statistical_outlier_filter()
    fil.set_mean_k (50)
    fil.set_std_dev_mul_thresh (1.0)
    pointcloud=fil.filter()
    pointcloud=np.asarray(pointcloud)
    #get the boundary of the points in pointcloud's coordinate

    #filter the x and y readings of the pointcloud
    x_filtered=pointcloud[:,0]
    y_filtered=pointcloud[:,1]
    z_filtered=pointcloud[:,2]

    x_min=np.amin(x_filtered)
    x_max=np.amax(x_filtered)
    y_min=np.amin(y_filtered)
    y_max=np.amax(y_filtered)
    z_min=np.amin(z_filtered)
    z_max=np.amax(z_filtered)


    #map the pointcloud into a 2d imagee
    num_pixels_x=int((x_max-x_min)/pixel_length)
    num_pixels_y=int((y_max-y_min)/pixel_length)
    #save 20 pixels margin(for finding contours)
    image=np.full((num_pixels_y+40,num_pixels_x+40),255,dtype="uint8")
    points2map=pointcloud[(pointcloud[:,0] >x_min) & (pointcloud[:,0] <x_max)  & (pointcloud[:,1] > y_min)  & (pointcloud[:,1] < y_max)][:,[0,1]]
    points2map[:,0]-=x_min
    points2map[:,1]=y_max-points2map[:,1]
    points2map[:,0]/=pixel_length
    points2map[:,1]/=pixel_length
    points2map[:,0]+=20
    points2map[:,1]+=20

    center_coordinate=[20+int((ee_pos[0]-x_min)/pixel_length),20+int((y_max-ee_pos[1])/pixel_length)]

    points2map_index=points2map.astype(int)
    points2map=[[t[1], t[0]] for t in points2map_index]

    image[tuple(np.array(points2map).T)]=np.uint8(0)

    points2map_index = np.array(points2map_index).reshape((-1,1,2)).astype(np.int32)
    rect=cv2.minAreaRect(points2map_index)
    center_rect,dimension_rect,rotation_rect=rect

    # box = cv2.boxPoints(rect)
    # box = np.int0(box)
    # cv2.drawContours(image,[box],0,(0,0,255),2)
    center_offset=((center_coordinate[0]-center_rect[0])*pixel_length,(-center_coordinate[1]+center_rect[1])*pixel_length,ee_pos[2]-z_max)


    # try:
    #     image[int(center_rect[1])-3:int(center_rect[1])+3,int(center_rect[0])-3:int(center_rect[0])+3]=np.uint8(160)
    #     image[center_coordinate[1]-3:center_coordinate[1]+3,center_coordinate[0]-3:center_coordinate[0]+3]=np.uint8(100)
    #     cv2.imshow("object",image)
    #     k = cv2.waitKey(0)
    #     if k == 27:
    #         cv2.destroyAllWindows()

    # except:
    #     print center_offset

    return ([max(minimum_D,dimension_rect[0]*pixel_length),max(minimum_D,dimension_rect[1]*pixel_length),z_max-z_min],center_offset,-rotation_rect)


def find_placement(order,depth_map,item,margin,pixel_length,rotate,stability,layer,layer_map,max_height):

    """
    Pack an object using a depth map
    Input:

    depth_map: an 2D unsigned int8 numpy array containing the depth information of the bin
    items: 3d bouding box of the object to be placed as a python list [length,width,height]
    margin: how much margin to be left at the edge of the object in m

    length_per_pixel: x, y increment in meter from 1 pixel to the next pixel, suggest using 0.001m for a 0.3m*0.3m bin entirely enclosed in a 480*640 array


    Output:
    index: center of the candidate placements in the depth map (not rotated) as a list of python list [[x1,y1,angle1],[x2,y2,angle2]...]
    max_height: canditates of stack hight of the object as an interger between 0-255 as a python list
    """

    #by default packer will find placement for object both vertically or horizontally, therefore setting increment from 0 to 90 degrees to be 90
    increment = 90

    #if user allow object rotation, set the increment to 10
    if rotate:
        increment = 10


    exceed_height=True
    index=[]
    min_scores=[]
    stack_height=255
    location=[]
    layer_copies=[]
    copies=[]
    orientation=0
    score=[]
    min_height=[]
    histogram=None
    #get the window size for the object(Adding the margin in)
    N=int((item[0]+2*margin)/pixel_length)
    M=int((item[1]+2*margin)/pixel_length)

    #If object doesn't fit in the designated storage system, force rotation on
    if max(M,N)>max(depth_map.shape):
        increment = 10

    for angle in np.arange(0, 171, increment):
        matrix,depth_rotated=rotate_bound(depth_map, angle)
        mat2,layer_rotated=rotate_bound_black(layer_map.copy(), angle)
        P,Q=depth_rotated.shape

        try:
            #get the maximum in each rolling window for height
            maxs = maxf2D(depth_rotated, size=(M,N))
            RW_max_H=maxs[M//2:(M//2)+P-M+1, N//2:(N//2)+Q-N+1]
            score=RW_max_H.copy()

            if stability or layer:
                layer_windows = maxf2D(layer_rotated, size=(M,N))
                layer_maxes=layer_windows[M//2:(M//2)+P-M+1, N//2:(N//2)+Q-N+1]
                index_minH,min_score=updatescore(score,layer_maxes,depth_rotated,stability,(N,M),layer,pixel_length)

            else:
                index_minH=unravel_index(RW_max_H.argmin(), RW_max_H.shape)
                min_score=RW_max_H[index_minH[0]][index_minH[1]]
            H_min=RW_max_H[index_minH[0]][index_minH[1]]
            if H_min<255:
                exceed_height=False
                if len(min_scores) == 0 or min_score<min(min_scores):
                    min_scores.append(min_score)
                    min_height.append(H_min)
                    index.append([index_minH[1]+N/2,index_minH[0]+M/2,angle])
                    #obtain a copy for visualization
                    copy=depth_rotated.copy()
                    layer_copy=layer_rotated.copy()
                    copy[int(index_minH[0]):int(index_minH[0]+M),int(index_minH[1]):int(index_minH[1]+N)]=np.uint8(80)
                    layer_copy[int(index_minH[0]):int(index_minH[0]+M),int(index_minH[1]):int(index_minH[1]+N)]+=np.uint8(1)
                    layer_copies.append(layer_copy)
                    copies.append(copy)



        except:
            continue

    if len(min_scores)>0:
        indice=np.argmin(min_scores)
        indices_original=mapindexback2original(depth_map.shape,copies[indice].shape,index[indice])
        location=indices_original
        orientation=index[indice][2]
        stack_height=min_height[indice]

        Mat,histogram=rotate_bound(copies[indice],-orientation)
        Mat2,layer_bound=rotate_bound_black(layer_copies[indice],-orientation)

        # Back rotate the copy for visualization
        (h, w) = histogram.shape[:2]
        (cX, cY) = (w // 2, h // 2)
        original_shape=depth_map.shape
        histogram=histogram[cY-original_shape[0]/2:cY+original_shape[0]/2,cX-original_shape[1]/2:cX+original_shape[1]/2]
        layer_map=layer_bound[cY-original_shape[0]/2:cY+original_shape[0]/2+1,cX-original_shape[1]/2:cX+original_shape[1]/2+1]
        histogram[indices_original[1]][indices_original[0]]=np.uint8(255)

    else:
        print "No placement for the item found"
	if exceed_height:
            print ('Item has a dimension greater than the dimension of bin {}.' .format(order))



    return (location,-orientation,stack_height,histogram,layer_map)


def pc2depthmap(order,pointcloud,threshold,length_per_pixel,BB,layer_map):
    """
    Convert a pointcloud(Structured or unstructured,2D or 3D) into a depth map where each pixel length is specified by length per pixel, and all Z reading below threshold will be converted to unsigned int from 0 to 255, all depth readings above threshold will be considered noise and assign a value of 0

    Input:

    pointcloud:pointcloud(Structured or unstructured,2D or 3D) in a numpy array

    threshold: maximum height to be considered in meters, e.g., for the storage system with a maximum height of 0.3m, the thereshold could be set up at 0.4m as anything higher than 0.4m is likely to be noise

    length_per_pixel: x, y increment in meter from 1 pixel to the next pixel, suggest using 0.001m for a 0.3m**0.3m bin entirely enclosed in a 480*640 array

    BBs: Bounding box as a python list of 2 cornor points, e.g.  [[-0.1715, -0.1395, 0], [0.1715, 0.1395, 0.121]] containing only the area of interest in a given pointcloud

    Output: a de-noised depth map of the dimension x=(x_max-x_min)/length_per_pixel,y=(y_max-y_min)/length_per_pixel
    (x_min,y_max) x,y value for the first pixel in the depth map

    """


    #if the layer map is empty, initialize the layer map
    #flatten and reshape the array to a list of points
    pointcloud=pointcloud.flatten()
    pointcloud=np.reshape(pointcloud,(pointcloud.size/3,3))
    #get the boundary of the points in pointcloud's coordinate

    #filter the x and y readings of the pointcloud

    x_min=min(BB[0][0], BB[1][0],BB[2][0], BB[3][0])
    x_max=max(BB[0][0], BB[1][0],BB[2][0], BB[3][0])
    y_min=min(BB[0][1], BB[1][1],BB[2][1], BB[3][1])
    y_max=max(BB[0][1], BB[1][1],BB[2][1], BB[3][1])
    z_min=min(BB[0][2], BB[1][2],BB[2][2], BB[3][2])
    z_max=max(BB[0][2], BB[1][2],BB[2][2], BB[3][2])

    if z_max+0.05>threshold:
        threshold=z_max+0.05
    count=0
    num_pixels_X=int((x_max-x_min)/length_per_pixel)
    num_pixels_Y=int((y_max-y_min)/length_per_pixel)

    #Initialize an empty depth map
    depth_map=np.full((num_pixels_Y,num_pixels_X),255,dtype="uint8")

    #initialize  layer_map if one doesn't exsist
    if len(layer_map) == 0 or layer_map.shape!=depth_map.shape:
        print "Layer map is either None or doesn't match the shape of the bin, creating a new layer_map"
        layer_map=np.full((num_pixels_Y,num_pixels_X),0,dtype="uint8")

    bounding_Rectangle=[]
    for bounding_point in BB:
        bounding_Rectangle.append([int((bounding_point[0]-x_min)/length_per_pixel),int((-bounding_point[1]+y_max)/length_per_pixel)])
    bounding_Rectangle = np.array(bounding_Rectangle).reshape((-1,1,2)).astype(np.int32)
    bounding_Rectangle = cv2.convexHull(bounding_Rectangle)
    cv2.fillConvexPoly(depth_map, bounding_Rectangle,0)


    #fill in the depth map
    for point in pointcloud[0::2]:
        x=(point[0]-x_min)/length_per_pixel
        y=(y_max-point[1])/length_per_pixel

        x=int(min(max(0,x),num_pixels_X-1))
        y=int(min(max(0,y),num_pixels_Y-1))
        #filter out high and low points
        if point[2]<threshold:
                depth_map[y][x]=np.uint8(max(depth_map[y][x],(point[2]-z_min)*1.00/threshold*255))

    #de-noise the raw depth map using median filter
    depth_map=cv2.fastNlMeansDenoising(depth_map,None,20,11,21)

    # cv2.imshow("bin",depth_map)
    # k = cv2.waitKey(0)
    # if k == 27:
    #     cv2.destroyAllWindows()

    return (depth_map,(x_min,y_max),layer_map,z_min,z_max)

def detect_local_minima(arr):
    # http://stackoverflow.com/questions/3684484/peak-detection-in-a-2d-array/3689710#3689710
    """
    Takes an array and detects the troughs using the local minimum filter.
    return the coordinate of the pixels for the local minimum
    """
    # define an connected neighborhood
    # http://www.scipy.org/doc/api_docs/SciPy.ndimage.morphology.html#generate_binary_structure
    neighborhood = morphology.generate_binary_structure(len(arr.shape),2)
    # apply the local minimum filter; all locations of minimum value
    # in their neighborhood are set to 1
    # http://www.scipy.org/doc/api_docs/SciPy.ndimage.filters.html#minimum_filter
    local_min = (filters.minimum_filter(arr, footprint=neighborhood)==arr)
    # In order to isolate the minimums we must remove the background from the mask.
    # we create the mask of the background(we use white background when rotating the image)
    background = (arr==255)
    eroded_background = morphology.binary_erosion(
        background, structure=neighborhood, border_value=1)
    # we obtain the final mask, containing only local minimums,
    # by removing the background from the local_min mask
    detected_minima = local_min - eroded_background
    return np.where(detected_minima)

def reject_outliers(data, m = 3.5):
    d = np.abs(data - np.median(data))
    mdev = np.median(d)
    s = d/mdev if mdev else 0.
    return data[s<m]

def rotate_bound(image, angle):
    """
    A modification to the image rotation algorithms in opencv, the original wrap affine method in opencv will keep the original dimension of the given image during rotation therefore cutting the edge off. The rotate_bound method will keep imcrease the dimension of the image and conserve all the edges.
    input:
    image: image needed to be rotated in an 2D numpy array
    angle: rotating angle in degree

    output:
    the "wrapped" rotated image in a 2D numpy array
    """


    # grab the dimensions of the image and then determine the
    (h, w) = image.shape[:2]
    (cX, cY) = (w // 2, h // 2)
    # center


    # grab the rotation matrix (applying the negative of the
    # angle to rotate clockwise), then grab the sine and cosine
    # (i.e., th rotation components of the matrix)
    M = cv2.getRotationMatrix2D((cX, cY), -angle, 1.0)

    cos = np.abs(M[0, 0])
    sin = np.abs(M[0, 1])

    # compute the new bounding dimensions of the image
    nW = int((h * sin) + (w * cos))
    nH = int((h * cos) + (w * sin))

    # adjust the rotation matrix to take into account translation
    M[0, 2] += (nW / 2) - cX
    M[1, 2] += (nH / 2) - cY
    # perform the actual rotation and return the image and fill the blank with white background
    return (M,cv2.warpAffine(image, M, (nW,nH), borderValue=255))


def rotate_bound_black(image, angle):
    """
    A modification to the image rotation algorithms in opencv, the original wrap affine method in opencv will keep the original dimension of the given image during rotation therefore cutting the edge off. The rotate_bound method will keep imcrease the dimension of the image and conserve all the edges.
    input:
    image: image needed to be rotated in an 2D numpy array
    angle: rotating angle in degree

    output:
    the "wrapped" rotated image in a 2D numpy array
    """


    # grab the dimensions of the image and then determine the
    (h, w) = image.shape[:2]
    (cX, cY) = (w // 2, h // 2)
    # center


    # grab the rotation matrix (applying the negative of the
    # angle to rotate clockwise), then grab the sine and cosine
    # (i.e., the rotation components of the matrix)
    M = cv2.getRotationMatrix2D((cX, cY), -angle, 1.0)

    cos = np.abs(M[0, 0])
    sin = np.abs(M[0, 1])

    # compute the new bounding dimensions of the image
    nW = int((h * sin) + (w * cos))
    nH = int((h * cos) + (w * sin))

    # adjust the rotation matrix to take into account translation
    M[0, 2] += (nW / 2) - cX
    M[1, 2] += (nH / 2) - cY
    # perform the actual rotation and return the image and fill the blank with black background
    return (M,cv2.warpAffine(image, M, (nW,nH)))

def mapindexback2original(D_original,D_rotated,index_angle):

    """
    map a index on the rotated matrix to the original matrix
    """
    h,w=D_original
    H,W=D_rotated
    X,Y,angle=index_angle

    if np.uint8(angle) == np.uint8(0):
        x=X
        y=Y
    elif np.uint8(angle) == np.uint8(90):
        x=Y
        y=h-1-X

    elif np.uint8(angle) < np.uint8(90):
        theta=math.radians(angle)
        y=int((W-X-math.cos(theta)*(math.sin(theta)*w-Y)/math.sin(theta))*math.sin(theta))
        x=int((Y/math.cos(theta)-y)/math.tan(theta))
    else:
        theta=math.radians(angle-90)
        y=int((H-Y-(w*math.sin(theta)-(W-X-1))/math.tan(theta))*math.sin(theta))
        x=int((W-X-1)/math.sin(theta)-y/math.tan(theta))



    return [x,y]

def updatescore(score,layer_score,depth_rotated,stability,item_dimension,layer, pixel_length):
    """
    a function that will update the current max map with stability scoring, this function will select
    """
    index=[0,0]
    maxscore=None
    x,y=item_dimension
    minimum_Depth = np.amin(score)

    #If the minimum stack height is within 2cm, it is likely placed on the floor, so no steability
    #or layer needed to be considered

    #also, if the height is 255(meaning no placement found for this object at this orientation, also skip the checking


    if minimum_Depth<0.02/pixel_length or minimum_Depth==255:
        # this item will likely be placed just on the floor, no need to check stability or layer
        return (unravel_index(score.argmin(), score.shape),minimum_Depth)

    else:

        score=score.astype(np.uint16)
        layer_score=layer_score.astype(np.uint16)
        layer_score=(layer_score*2) **2
        if layer:
            score=np.add(score, layer_score)

        if stability:
            print "Running stability check..."
            sample_distance=10
            #downsample the score matrix, otherwise there will be too many checkings
            score=score[::sample_distance,::sample_distance]

            #consider any placement that are within 20 points(weighs roughly 2-4 cm) of the lowest score(Best score) for stability
            #setting anything above that value to be the maximum score
            indices=np.nonzero(score<np.amin(score)+20)

            for i in range(len(indices[0])):

                support=depth_rotated[indices[0][i]*sample_distance:indices[0][i]*sample_distance+y,indices[1][i]*sample_distance:indices[1][i]*sample_distance+x]
                highpoints=np.nonzero(support>np.amax(support)-0.02/pixel_length)
                highpoints = zip(highpoints[1],highpoints[0])
                contour=np.array(highpoints).reshape((-1,1,2)).astype(np.int32)
                hull = cv2.convexHull(contour)
                support_Area = cv2.contourArea(hull)
                M = cv2.moments(hull)
                if M['m00']!=0:
                    cx = int(M['m10']/M['m00'])
                    cy = int(M['m01']/M['m00'])
                    support_Center = (cx,cy)
                    #assign scores to stability, 15 points for supporting area, 5 points for how close the support is to the center of gravity
                    score_Stability=15*(x*y-support_Area)/(x*y)+5*math.sqrt((cx-x/2)**2+(cy-y/2)**2)/math.sqrt((x/2)**2+(y/2)**2)
                    score[indices[0][i]][indices[1][i]]+=score_Stability
                else:
                    #if the support area is too small, simply adding 20 points to this area
                    score[indices[0][i]][indices[1][i]]+=20



        index=unravel_index(score.argmin(), score.shape)
        if stability:
            index=(index[0]*sample_distance,index[1]*sample_distance)


        return (index, np.amin(score))






# """

# num2pack=30

# items=np.array([[266,294,41],[142,187,30],[89,126,30],[136,217,123],[124,198,30],[119,225,66],[86,227,23],[191,248,11],[72,114,28],[97,96,49],[163,252,68],[135,55,37],[66,124,11],[25,85,30],[102,185,22],[59,144,57],[226,267,72],[104,127,22],[123,330,43],[64,100,39],[108,177,15],[117,113,85],[75,145,27],[108,144,109],[109,186,36],[258,258,50],[79,217,79],[66,201,65],[47,340,47],[136,192,16],[168,201,6],[82,145,53],[69,144,61],[187,398,23],[77,218,77],[93,215,12],[116,132,113],[168,403,85],[170,182,39],[103,269,61]])

# #sort items by their largest face

# for indice,cube in enumerate(items):
#         cube.sort()
#         items[indice]=cube
# cubes=items.tolist()

# #sort this item order from large to small
# cubes.sort(key=lambda tup: tup[1]*tup[2])

# #get the top n items with largest face

# if len(cubes)>num2pack:
#         cubes=cubes[:num2pack]

# #randomly shuffle the order of the items to be packed

# random.shuffle(cubes)


"""

map=None
pointcloud=[]
pc=np.load("pc_masked.py.npy")
for i in range(3):
    pointcloud.append(pc)
#for cube in cubes:
#   print cube
for filename in glob.iglob('test_pointclouds/*.pcd'):

    seg_pcd=pcl.load('%s' % filename)
    item=np.asarray(seg_pcd)
    print filename

    best_index,location,angle,layer_map=pack(pointcloud,item, [[[-0.1715, -0.1395, 0], [0.1715, 0.1395, 0.121]],[[-0.1715, -0.1395, 0], [0.1715, 0.1395, 0.121]],[[-0.1715, -0.1395, 0], [0.1715, 0.1395, 0.121]]],map,rotate=True )
    map=layer_map
"""