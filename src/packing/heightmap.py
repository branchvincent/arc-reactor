import numpy as np
from scipy.ndimage.filters import maximum_filter as maxf2D
import cv2
import random
from numpy import unravel_index

def pack(bins,item,BBs, margin=0.005,max_height=0.6,pixel_length=0.001):
    """
    pack a item into a selected number of bins and return the center coordinate of the objects when packed

    Input:

    bins: structured or unstructured pointclouds of each bins the item might be packed into as a list of numpy arrays
    items: 3d bouding box of the object to be placed as a python list [length,width,height]

    margin: how much margin to be left at the edge of the object in meter

    max_height: the maximum height used for drawing the depth map and the max height to be considered for placing

    length_per_pixel: x, y increment in meter from 1 pixel to the next pixel, suggest using 0.001m for a 0.3m*0.3m bin entirely enclosed in a 480*640 array

    BBs: a list of bounding box that enclose only the area of interest for packing. Each bounding box is a python list of 2 cornor points, e.g.  [[-0.1715, -0.1395, 0], [0.1715, 0.1395, 0.121]] , the length of the bounding boxs should be the same length as the number of bins to be considered



    Output:
    location: center coordinate of for top of the object when packed properly. In a python list [x,y,z], if no proper placement was found, return None
    """
    depth_maps=[]
    minimum_height=[]
    locations=[]

    for index,bin in enumerate(bins):
        depth_map,cornor_point=pc2depthmap(bin,max_height,pixel_length,BBs[index])

        depth_maps.append(depth_map)

    for depth_map in depth_maps:
        index,height=find_placement(depth_map,item,margin,pixel_length)
        if height!=255:
            z=height/255.0*max_height+item[2]
            x=cornor_point[0]+index[0]*pixel_length
            y=cornor_point[1]+index[1]*pixel_length
            minimum_height.append(height)
            locations.append([x,y,z])

    if len(minimum_height)>0:
        location=locations[minimum_height.index(min(minimum_height))]
        print location
        return location

    else:
        return None





def find_placement(depth_map,item,margin,pixel_length):

    """
    Pack an object using a depth map
    Input:

    depth_map: an 2D unsigned int8 numpy array containing the depth information of the bin
    items: 3d bouding box of the object to be placed as a python list [length,width,height]
    margin: how much margin to be left at the edge of the object in m

    length_per_pixel: x, y increment in meter from 1 pixel to the next pixel, suggest using 0.001m for a 0.3m*0.3m bin entirely enclosed in a 480*640 array


    Output:
    index: center of the object in the depth map as a python list [x,y]
    max_height: stack hight of the object as an interger between 0-255
    """

    index=[]
    max_height=255
    #get the window size for the object(Adding the margin in)
    N=int((item[0]+2*margin)/pixel_length)
    M=int((item[1]+2*margin)/pixel_length)
    P,Q=depth_map.shape

    try:
        #get the maximum in each rolling window for height
        maxs = maxf2D(depth_map, size=(M,N))
        RW_max_H=maxs[M//2:(M//2)+P-M+1, N//2:(N//2)+Q-N+1]
        index_minH=unravel_index(RW_max_H.argmin(), RW_max_H.shape)
        H_min=RW_max_H[index_minH[0]][index_minH[1]]

    except:
        index_minH=(0,0)
        H_min=np.uint8(255)

    try:
        maxs = maxf2D(depth_map, size=(N,M))
        RW_max_V=maxs[N//2:(N//2)+P-N+1, M//2:(M//2)+Q-M+1]
        index_minV=unravel_index(RW_max_V.argmin(), RW_max_V.shape)
        V_min=RW_max_V[index_minV[0]][index_minV[1]]
    except:
        index_minV=(0,0)
        V_min=np.uint8(255)

    if  V_min<H_min:
        #Place item vertically, else, please item horizontally
        index=index_minV
        index=[index[1]+int(M/2.0),index[0]+int(N/2.0)]
        if 255-V_min>item[2]/pixel_length:
            max_height=V_min
            depth_map[int(index[1]-N/2.0):int(index[1]+N/2.0),int(index[0]-M/2.0):int(index[0]+M/2.0)]=np.uint8(max_height+item[2]/pixel_length)
    else:
        index=index_minH
        index=[index[1]+int(N/2.0),index[0]+int(M/2.0)]
        if 255-H_min>item[2]/pixel_length:
            max_height=H_min
            depth_map[int(index[1]-M/2.0):int(index[1]+M/2.0),int(index[0]-N/2.0):int(index[0]+N/2.0)]=np.uint8(max_height+item[2]/pixel_length)

    # if max_height!=255:
    #     cv2.imshow("denoised",depth_map)
    #     k = cv2.waitKey(0)
    #     if k == 27:         # wait for ESC key to exit
    #         cv2.destroyAllWindows()
    return (index,max_height)


def pc2depthmap(pointcloud,threshold,length_per_pixel,BB):
    """
    Convert a pointcloud(Structured or unstructured,2D or 3D) into a depth map where each pixel length is specified by length per pixel, and all Z reading below threshold will be converted to unsigned int from 0 to 255, all depth readings above threshold will be considered noise and assign a value of 0

    Input:

    pointcloud:pointcloud(Structured or unstructured,2D or 3D) in a numpy array

    threshold: maximum height to be considered in meters, e.g., for the storage system with a maximum height of 0.3m, the thereshold could be set up at 0.4m as anything higher than 0.4m is likely to be noise

    length_per_pixel: x, y increment in meter from 1 pixel to the next pixel, suggest using 0.001m for a 0.3m*0.3m bin entirely enclosed in a 480*640 array

    BBs: Bounding box as a python list of 2 cornor points, e.g.  [[-0.1715, -0.1395, 0], [0.1715, 0.1395, 0.121]] containing only the area of interest in a given pointcloud

    Output: a de-noised depth map of the dimension x=(x_max-x_min)/length_per_pixel,y=(y_max-y_min)/length_per_pixel
    (x_min,y_min) x,y value for the first pixel in the depth map

    """
    #flatten and reshape the array to a list of points
    pointcloud=pointcloud.flatten()
    pointcloud=np.reshape(pointcloud,(pointcloud.size/3,3))
    #get the boundary of the points in pointcloud's coordinate

    #filter the x and y readings of the pointcloud

    x_raw=pointcloud[:,0]
    y_raw=pointcloud[:,1]
    """
    x_filtered=reject_outliers(x_raw, m = 3.5)
    y_filtered=reject_outliers(y_raw, m = 3.5)

    x_min=np.amin(x_filtered)
    x_max=np.amax(x_filtered)
    y_min=np.amin(y_filtered)
    y_max=np.amax(y_filtered)
    """

    x_min=BB[0][0]
    x_max=BB[1][0]
    y_min=BB[0][1]
    y_max=BB[1][1]

    count=0
    num_pixels_X=int((x_max-x_min)/length_per_pixel)
    num_pixels_Y=int((y_max-y_min)/length_per_pixel)

    #Initialize an empty depth map
    depth_map=np.full((num_pixels_Y,num_pixels_X),0,dtype="uint8")

    #fill in the depth map
    for point in pointcloud:
        x=(point[0]-x_min)/length_per_pixel
        y=(point[1]-y_min)/length_per_pixel

        x=int(min(max(0,x),num_pixels_X-1))
        y=int(min(max(0,y),num_pixels_Y-1))
        #filter out high and low points

        if point[2]>threshold or point[2]<0:
            z=np.uint8(0)
        else:
            depth_map[y][x]=max(depth_map[y][x],np.uint8(point[2]*1.00/threshold*255))

    #de-noise the raw depth map using median filter
    depth_map=cv2.fastNlMeansDenoising(depth_map,None,10,7,21)

    return (depth_map,(x_min,y_min))

def reject_outliers(data, m = 3.5):
    d = np.abs(data - np.median(data))
    mdev = np.median(d)
    s = d/mdev if mdev else 0.
    return data[s<m]





# num2pack=20

# items=np.array([[266,294,41],[142,187,30],[89,126,30],[136,217,123],[124,198,30],[119,225,66],[86,227,23],[191,248,11],[72,114,28],[97,96,49],[163,252,68],[135,55,37],[66,124,11],[25,85,30],[102,185,22],[59,144,57],[226,267,72],[104,127,22],[123,330,43],[64,100,39],[108,177,15],[117,113,85],[75,145,27],[108,144,109],[109,186,36],[258,258,50],[79,217,79],[66,201,65],[47,319,47],[136,192,16],[168,201,6],[82,145,53],[69,144,61],[187,398,23],[77,218,77],[93,215,12],[116,132,113],[168,403,85],[170,182,39],[103,269,61]])

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




# pointcloud=[]
# pc=np.load("pc_masked.py.npy")
# pointcloud.append(pc)
# for cube in cubes:
#     pack(pointcloud,[0.132,0.116,0.113], [[[-0.1715, -0.1395, 0], [0.1715, 0.1395, 0.121]]] )