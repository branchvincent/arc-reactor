import numpy as np
import math
import cv2
from scipy.spatial import distance as dist
from operator import itemgetter
import pcl
import glob
import json
import time

# planes are rated on a 10 point base system
Score=10


def pcd2image(pointcloud,indices):
    """
    get an image of the segmented plane using the indices of the plane on the original pointcloud
    input: structured numpy array of the pointcloud of the original scene
    induces: indices of the segmented plane on the original pointcloud, in format of a numpy index (array_y,array_x)

    output:
    plane_image: image of the segmented plane where the background color is 220 and the segment color is 0, 10 pixel margin is saved around the image
    start_index: the upper left cornor of the pointcloud where the segment starts-margin

    """
    image_Start_X=np.amin(indices[1])
    image_Finish_X=np.amax(indices[1])

    image_Start_Y=np.amin(indices[0])
    image_Finish_Y=np.amax(indices[0])
    sample_num=20
    #use 20 random samples to calculate the average length per pixel
    sample_indices=(np.random.rand(sample_num)*len(indices[0])).astype(int)
    count=0
    length_sum=0
    for i in range(sample_num/2):
        try:

            pixel_length=np.linalg.norm(np.array([indices[0][sample_indices[i]],indices[1][sample_indices[i]]])-np.array([indices[0][sample_indices[sample_num-1-i]],indices[1][sample_indices[sample_num-1-i]]]))

            point_dist = np.linalg.norm(pointcloud[indices[0][sample_indices[i]]][indices[1][sample_indices[i]]]-pointcloud[indices[0][sample_indices[sample_num-1-i]]][indices[1][sample_indices[sample_num-1-i]]])
            length_per_pixel=point_dist/pixel_length
            length_sum+=length_per_pixel
            count+=1
        except:
            continue
    try:
        length_per_pixel=length_sum/count
    except:
        length_per_pixel=0.001




    plane_image=np.full(pointcloud.shape[:2],220,dtype="uint8")

    plane_image[indices]=np.uint8(0)
    dimension_Y=image_Finish_Y-image_Start_Y
    dimension_X=image_Finish_X-image_Start_X
    background=np.full((dimension_Y+20,dimension_X+20),220,dtype="uint8")
    background[10:10+dimension_Y,10:10+dimension_X]=plane_image[image_Start_Y:image_Finish_Y,image_Start_X:image_Finish_X]
    return (background,np.array([image_Start_X-10,image_Start_Y-10]),length_per_pixel)

def check_size(contour,lengthPerPixel):
    """
    Check if the size of the plane is large enough for the vacuum hose, currently the thereshold is set at 5cm
    Input:
    Contour for the observed plane
    Output:
    Boolean value indicating if the size of the object is large enough to for the vacuum nozzle
    ratio between the smaller dimension of the minimum bounding rectangle over min diameter of the vacuum cup
    Fullness: The extent the given contour fits a rectangle or Ellipse

    """
    MIN_DIAMETER = 0.05
    MIN_FILL = 0.8
    #compute if the shape is close to a rectangle or circle enough, set 80% as threshold
    center_rect,dimension_rect,rotation_rect=cv2.minAreaRect(contour)
    center_circle,axis_circle,rotation_circle=cv2.fitEllipse(contour)
    area=cv2.contourArea(contour)
    FitRect=area/(dimension_rect[0]*dimension_rect[1])
    FitCircle=4*area/(math.pi*axis_circle[0]*axis_circle[1])
    IsPolygon=FitRect>MIN_FILL or FitCircle>MIN_FILL
    ratio=min(dimension_rect)*lengthPerPixel*1.00/MIN_DIAMETER


    return (ratio,max(FitRect,FitCircle))

def check_top(surfacePoints,coeficients,pointcloud):

    """
    check if the top surface of the object is covered by something else by intergrating the depth of the surface point to the plane

    Input:
    surfacePoints in python list
    plane equation in a python tuple

    Output:

    The percentage of coverage that exceeds the max_distance(Default 1cm)
    The average displacement from the measurement to the idealized plane surface

    """
    a,b,c,d=coeficients
    max_distance=0.01
    distance_sum=0
    count=0
    count_over=0
    #sample every 2nd element in all the surface point
    for pixels in surfacePoints[0::2]:
        #if the dept measurement exists for this pixel
        if pointcloud[pixels[1]][pixels[0]][2]!=0:

            point=pointcloud[pixels[1]][pixels[0]]
            distance=(a*point[0]+b*point[1]+c*point[2]+d)/math.sqrt(a**2+b**2+c**2)
            if abs(distance)>max_distance:
                count_over+=1
            distance_sum+=distance
            count+=1

    if count>0:
        percentage=float(count_over)/count
        average_distance=distance_sum/count

        return (percentage,average_distance)
    else:
         return (1,1)

def find_enclosingPixels(contour,plane_indices):
    """
    use test if inside polygon method to get all enclosing pixels inside a given contour

    Input:
    pixels: edge pixels in the format of python list

    Output:
    enclosing pixels: all the pixels contained inside the polygon defined by the edge pixel
    in python list
    plane_enclosed: python list of indices of the plane point that is within the contour found, each index is a list in the format of [indice_y,indice_x]

    """
    enclosing_pixels_pointcloud=[]
    plane_enclosed=[]

    #convert the python list to a contour format opencv recgonizes

    ctr = np.array(contour).reshape((-1,1,2)).astype(np.int32)

    #find the axis alighed bounding box for the contour

    x,y,w,h = cv2.boundingRect(ctr)

    # for all pixels inside the bounding box, check if it's an enclosing point
    for i in range(x,x+w):
        for j in range(y,y+h):

            dist = cv2.pointPolygonTest(ctr,(i,j),False)
            if dist>-1:
                enclosing_pixels_pointcloud.append([i,j])
    for j in range(len(plane_indices[0])):
        dist = cv2.pointPolygonTest(ctr,(plane_indices[1][j],plane_indices[0][j]),False)
        if dist>-1:
            plane_enclosed.append([plane_indices[0][j],plane_indices[1][j]])

    return (enclosing_pixels_pointcloud,plane_enclosed)

def get_enclosingPixels(contour,image):

    cnt= np.array(contour).reshape((-1,1,2)).astype(np.int32)
    mask=np.zeros(image.shape[:2],np.uint8)
    pixelpoints = cv2.findNonZero(mask)
    pixelpoints=pixelpoints.flatten()
    pixelpoints=np.reshape(pixelpoints,(pixelpoints.size/2,2))
    return pixelpoints

def get_mean_intensity(image,contour):
    mask=np.zeros(image.shape[:2],np.uint8)
    cv2.drawContours(mask,[contour],0,255,-1)
    mean_val = cv2.mean(image,mask = mask)

    return mean_val

def find_center(points):


    return np.mean(points, axis=0)

def check_surroundings(contour,pointcloud):

    """
    check  depths along the edge of the segmented plane in the original pointcloud

    Input:

    contour: contour pixels of edge surface in the original pointcloud

    Output:

    percentage_covered: the percentage of the edge confirmed to be covered
    percentage_gap: the percentage of the edge confirmed to have gaps surrounding it
    average_gap_depth: the average depth of the confirmed edge
    average_surrounding_height: the average height of the surrounding objects

    """
    #convert the python list to a contour format opencv recgonizes



    percentage_covered=0
    percentage_gap=0
    average_gap_depth=0
    average_surrounding_height=0

    ctr = np.array(contour).reshape((-1,1,2)).astype(np.int32)

    #find the axis alighed bounding box for the contour

    x,y,w,h = cv2.boundingRect(ctr)

    center_pixel=(int(x+w/2),int(y+h/2))

    # create a list to store the next 10 pixels inline with a vector pointing from
    # the center of the contour to the dege point for each edge point
    surrounding_pixels=[[] for j in xrange(len(contour)) ]
    try:
        for index, edge_pixel in enumerate(contour):
            x_diff=edge_pixel[0]-center_pixel[0]
            y_diff=edge_pixel[1]-center_pixel[1]
            sign_x=1
            sign_y=1
            if x_diff<0:
                sign_x=-1
            if y_diff<0:
                sign_y=-1

            if abs(x_diff)>abs(y_diff):
                for i in range(10):
                    surrounding_x=int(edge_pixel[0]+sign_x*(1+i))
                    surrounding_y=int(edge_pixel[1]+sign_y*(1+i)*abs(y_diff)/abs(x_diff))
                    x_withinFrame=surrounding_x>=0 and surrounding_x<pointcloud.shape[1]
                    y_withinFrame=surrounding_y>=0 and surrounding_y<pointcloud.shape[0]
                    if x_withinFrame and y_withinFrame:
                        surrounding_pixels[index].append((surrounding_x,surrounding_y))
            elif y_diff!=0:
                for i in range(10):
                    surrounding_x=int(edge_pixel[0]+sign_x*(1+i)*abs(x_diff)/abs(y_diff))
                    surrounding_y=int(edge_pixel[1]+sign_y*(1+i))
                    x_withinFrame=surrounding_x>=0 and surrounding_x<pointcloud.shape[1]
                    y_withinFrame=surrounding_y>=0 and surrounding_y<pointcloud.shape[0]
                    if x_withinFrame and y_withinFrame:
                        surrounding_pixels[index].append((surrounding_x,surrounding_y))
    except:
        return (0,1,0,0,0,1)


    edge_gap=0
    edge_above=0
    edge_inconclusive=0
    edge_close=0
    edge_gap_Depth=0
    edge_above_height=0


    #compare the surrounding pixels depth value with the edge depth
    for index,pixels in enumerate(surrounding_pixels):
        missing_points=0
        lower_points=0
        higher_points=0
        depth_gap=0
        depth_above=0

        if pointcloud[contour[index][1]][contour[index][0]][2]==0:
            continue
        else:
            for pixel in pixels:
                gap=pointcloud[contour[index][1]][contour[index][0]][2]-pointcloud[pixel[1]][pixel[0]][2]
                if pointcloud[pixel[1]][pixel[0]][2]==0:
                    missing_points+=1

                elif gap>0.01:
                    lower_points+=1
                    depth_gap+=gap
                elif gap<-0.005:
                    higher_points+=1
                    depth_above-=gap
        if higher_points==0 and lower_points==1 or lower_points>1:
            edge_gap+=1
            edge_gap_Depth+=depth_gap/lower_points

        elif higher_points>2 and lower_points==0:
            edge_above+=1
            edge_above_height+=depth_above/higher_points

        elif missing_points>7:
            edge_inconclusive+=1
        else:
            edge_close+=1
    try:
        gap_percentage=float(edge_gap)/(edge_gap+edge_above+edge_inconclusive+edge_close)
        coverage_percentage=float(edge_above)/(edge_gap+edge_above+edge_inconclusive+edge_close)
        missing_percentage=float(edge_inconclusive)/(edge_gap+edge_above+edge_inconclusive+edge_close)
        close_percentage=float(edge_close)/(edge_gap+edge_above+edge_inconclusive+edge_close)
        try:
            ave_gap=edge_gap_Depth/edge_gap
        except:
            ave_gap=0

        try:

            ave_height=edge_above_height/edge_above
        except:
            ave_height=0


        return (gap_percentage, coverage_percentage, missing_percentage,close_percentage, ave_gap, ave_height)
    except:
        #if no readings are available, return the plane as 100% covered and with a height of 1m
        return (0,1,0,0,0,1)


def check_flatness(segment,mask_num):
    """
    check flatness of the input segment and return the segmented plane

    Input:

    segment: point cloud segment in .pcd

    Output:

    model: the plane equation in a python list [a,b,c,d]
    cloud_plane: The fitted plane in .pcd
    indices: indices of the segmented plane


    """
    global seg_pcd
    global mask_indices
    seg = segment.make_segmenter_normals(ksearch=50)
    seg.set_optimize_coefficients(True)
    seg.set_model_type(pcl.SACMODEL_NORMAL_PLANE)
    seg.set_normal_distance_weight(0.1)
    seg.set_method_type(pcl.SAC_RANSAC)
    seg.set_max_iterations(100)
    seg.set_distance_threshold(0.05)

    indices, model = seg.segment()
    cloud_plane = segment.extract(indices, negative=False)
    seg_pcd=segment.extract(indices,negative=True)
    indices=np.array(indices)
    indices=indices.astype(int)
    indices_pc=(mask_indices[mask_num][0][(indices,)],mask_indices[mask_num][1][(indices,)])
    index_y_remained=np.delete(mask_indices[mask_num][0], indices)
    index_x_remained=np.delete(mask_indices[mask_num][1], indices)
    mask_indices[mask_num]=(index_y_remained,index_x_remained)
    return (model, cloud_plane,indices_pc)




def rate_plane(pc,depth_threshold=0.4,masks=None,img=None):
    """
    Find graspable sites inside the masked region of the pointcloud and rate graspbility with vacuum gripper for each identified plane.

    Input:
    pc: Structured pointcloud in numpy array
    depth_threshold: maximum depth considered in the bin, anything above that depth will be considered noise.
    masks: A python list of numpy arrays where each arry is a mask of the object segment where only the area of interest is represented as 1 and everywhere else is 0. The size of the mask should be the same as the pointcloud. If no mask is provided, the code will segment all the planes it could find and rate them
    img: rgb image alligned to the depth of the scene,if provided, will show visualization of the plane rating on the image, if not provided, will show visualization of the segments alone.
    output: a python dictionary of extracted features of the plane, along with the center coordinates, surface normals, and ratings for graspbility
    format {'segment': filename,'center':center.tolist(),'orientation':plane_equation, 'score':Finalscore, 'flatness':plane_percentage, 'tilting_angle': angle, 'size': ratio, 'convexity':fitness,'close_edge': edge_close, 'gap': edge_gap,'blockage': edge_covered, 'edge_inconclusive':edge_missing, 'gap_Depth':gap_Depth, 'blockage_height':converage_height,'top_coverage':percentage, 'cover_height': average_distance }

    """
    segments=[]
    global seg_pcd
    Plane_info=[]
    global mask_indices
    mask_indices=[]
    show_Segment=img.copy()
    large_masks=[]

    #denoise pointcloud
    pointcloud=pc.astype(np.float32)
    pc_shape=pointcloud.shape
    pointcloud=pointcloud.flatten()
    pointcloud=np.reshape(pointcloud,(pointcloud.size/3,3))
    #filter out depth readings that are higher than the threshold setup
    pointcloud[pointcloud[:,2]>depth_threshold]=np.array([0,0,0],dtype='float32')
    if masks is None:
        segment_npy=pointcloud
        #remove the all zero points from the segment first
        mask0 = np.all(segment_npy == 0, axis=1)
        segment_npy = segment_npy[~mask0]
        segments.append(segment_npy)
    pointcloud=pointcloud.reshape(pc_shape)
    for mask in masks:
        seg_index=mask.nonzero()
        segment_npy= pointcloud[seg_index]
        if len(segment_npy)>1000:
            mask_indices.append(seg_index)
            segments.append(segment_npy)
            large_masks.append(mask)

    candidate_planes=[]
    for maskNum,segment in enumerate(segments):
        #print maskNum
        show_Segment=img.copy()
        #get the center of gravity for each segments
        mask_Size=len(segment)
        COG = [float(sum(col))/len(col) for col in zip(*segment)]
        seg_pcd=pcl.PointCloud(segment)
        first_Segment=True
        while seg_pcd.size>1000:
            deductions = {'Orientation': 0, 'top_Coverage': 0, 'Side_coverage':0, 'Side_height':0, 'size':0, 'Solidity':0,'Off_center':0 }
            try:
                #print "Checkflatness"
                #print time.time()
                plane_equation,plane_pcd,indices_in_pc=check_flatness(seg_pcd,maskNum)

                if plane_pcd.size < 500:
                    #print "Breaking due to plane too small"
                    break
            except:
                #print "somehow failed"
                break

            #print "have a large segment"

            plane = np.asarray(plane_pcd)
            plane=plane.astype(np.float32)
            plane_percentage=len(plane)*1.0/mask_Size
            #print "pcd2image"
            #print time.time()
            image,start_index,lengthPerPixel = pcd2image(pointcloud,indices_in_pc)
            #print lengthPerPixel
            #get the angle from the plane equation
            angle=math.acos(plane_equation[2])
            angle=angle*180.0/math.pi
            if abs(angle)>90:
                angle=180-abs(angle)
            #print "find_contour"
            #print time.time()


            Found_Contour,cnt,image=find_contour(image)
            if Found_Contour:
                intensity=get_mean_intensity(image,cnt)
                #cv2.drawContours(image,cnt, -1, (100), 3)
                #print "chek_Size"
                #print time.time()
                ratio, fitness=check_size(cnt,lengthPerPixel)
                #find the pixels for the plane edge in the original pointcloud
                pixel_in_pointcloud=np.reshape(cnt,(cnt.size/2,2))+start_index
                pixel_in_pointcloud=pixel_in_pointcloud[0::2]
                #Check around the edge of the plane and detect if it is likely covered around the edge
                #print "check_surroundings"
                #print time.time()
                edge_gap, edge_covered, edge_missing, edge_close, gap_Depth, converage_height=check_surroundings(pixel_in_pointcloud,pointcloud)
                #check if the top of the plane is likely covered
                #Use those edge pixels to find all the enclosing pixels inside the contour
                #print "find_enclosing pixels"
                #print time.time()
                enclosing_pixels_pointcloud,segment_indices=find_enclosingPixels(pixel_in_pointcloud,mask_indices[maskNum])
                #print enclosing_pixels_pointcloud
                #Todos: optimize the enclosing pixels function
                #enclosing_pixels_pointcloud=get_enclosingPixels(pixel_in_pointcloud,pointcloud)
                #indices2edges=pixel_in_pointcloud.copy().T
                #indices2edges[0], indices2edges[1] = indices2edges[1], indices2edges[0]
                if len(segment_indices)>0:
                    center=find_center(pointcloud[tuple(np.array(segment_indices).T)])
                    centerPlane2centerSeg=np.linalg.norm(center-COG)
                    #print "check_top"
                    #print time.time()
                    percentage,average_distance=check_top(enclosing_pixels_pointcloud,plane_equation,pointcloud)
                    #print "start evaluating scores"
                    #print time.time()
                    if average_distance<0:
                        center[2]+=average_distance
                    #Here are the priliminary scorings for each features

                    if ratio<1:
                        deductions['size']=(1/ratio)**2*20-20
                    if fitness<0.8:
                        deductions['size']+=(0.8-fitness)*10

                    deductions['Side_coverage']=max(0,edge_close+(edge_covered*10)**2*converage_height*40-edge_gap*gap_Depth*100)


                    if percentage>0.02 and average_distance>0 or average_distance<-0.02:
                        deductions['top_Coverage']=abs(average_distance)*1000*percentage*100*0.5

                    if angle>10:
                        deductions['Orientation']=((angle-10)/10)**2*0.5

                    deductions['Off_center']=centerPlane2centerSeg*(1-plane_percentage)*50
                    deductions['Solidity']=(intensity[0]/50.0)**3
                    Finalscore=Score-sum(deductions.values())

                    #Load all plane information into a python list where each of the sublist is a python dictionary


                    Plane_info.append({'segment': maskNum,'center':center.tolist(),'orientation':plane_equation, 'score':Finalscore, 'flatness':plane_percentage, 'tilting_angle': angle, 'size': ratio, 'convexity':fitness,'close_edge': edge_close, 'gap': edge_gap,'blockage': edge_covered, 'edge_inconclusive':edge_missing, 'gap_Depth':gap_Depth, 'blockage_height':converage_height,'top_coverage':percentage, 'cover_height': average_distance,'distance2COG':centerPlane2centerSeg,'percentageAsPlane':plane_percentage,'mean_intensity':intensity[0],'height':center[2]})

                    #printing information to the terminal
                    """

                    print('Plane angle is {} degrees, taking {} points off.'.format(angle,deductions['Orientation']))
                    print " "
                    print('Plane intensity(solidity) is {} , taking {} points off.'.format(intensity[0],deductions['Solidity']))
                    print " "
                    print('The smaller dimension is {} required, its shape can be approximated as a rectangle or ellipse by {} %, taking {} off.'.format(ratio,fitness*100,deductions['size']))
                    print " "
                    print('Segmented plane is {}cm away from the segment center of the gravity , {} percent of the original can be approximated as a plane. Taking {} off.'.format(centerPlane2centerSeg,plane_percentage*100,deductions['Off_center']))
                    print " "
                    print('{} percent of surroundings are confirmed gaps, with an average dip of {} cm.  {} percent of surroundings are confirmed blockage, with an average height of {} cm. {} percent of surroundings have insufficient readings. {} percent of the surroundings have readings that are close to the edge. Taking {} points off'.format(edge_gap*100,gap_Depth*100,edge_covered*100,converage_height*100,edge_missing*100,edge_close*100, deductions['Side_coverage']))
                    print " "
                    print('{} percent of measurements is above the 1 cm thereshold, with an average displacement of {} cm. Taking {} points off.'.format(percentage*100,average_distance*100,deductions['top_Coverage']))

                    print " "
                    print ("The final score for the plane is {}" .format(Finalscore))
                    print " "
                    print " "
                    print "                                                               Press 'Space' for the next plane                                                                              "
                    print " "


                    try:
                        if Finalscore>-500000:

                        #if uv map and color image exists, map depth pixels to RGB pixels for drawing segments on the original image
                            ctr = np.array(pixel_in_pointcloud).reshape((-1,1,2)).astype(np.int32)

                            current_mask=large_masks[maskNum]
                            current_mask=current_mask.astype(np.uint8)
                            show_Segment=cv2.bitwise_and(show_Segment,show_Segment,mask=current_mask)
                            img2=img

                            M = cv2.moments(ctr)
                            cX = int(M["m10"] / M["m00"])
                            cY = int(M["m01"] / M["m00"])
                            cv2.drawContours(img2,[ctr],0,(0,255,0),2)
                            if Finalscore>-5:
                                cv2.putText(img2,str(Finalscore)[:4], (cX-10 , cY ),cv2.FONT_HERSHEY_SIMPLEX, 0.8, (0, 0, 255), 3)
                            cv2.imshow("masked_Area",show_Segment)
                            cv2.imshow("image",img2)
                            cv2.imshow("segments",image)
                            k = cv2.waitKey(0)
                            if k == 27:

                                cv2.destroyAllWindows()


                            # Otherwise, just show images of the segments

                    except:
                        # Otherwise, just show images of the segments
                        cv2.imshow("segments",image)
                        k = cv2.waitKey(0)
                        if k == 27:         # wait for ESC key to exit

                            cv2.destroyAllWindows()

                    """

    #for all planes with positive scores, arrange the planes in order of height and add punishment for lower planes
    Plane_info = sorted(Plane_info, key=itemgetter('height'), reverse=True)
    order_positive_plane=0
    for order,rated_plane in enumerate(Plane_info):
        if rated_plane['score']>0:
            Plane_info[order]['score']-=order_positive_plane*0.5
            #print('Plane is the {}th in height, taking {} off.'.format(order_positive_plane+1,order_positive_plane*0.5))
            order_positive_plane+=1

    return Plane_info





def find_contour(image):
    find_contour=False
    gray = cv2.GaussianBlur(image, (3, 3), 0)
    # perform edge detection, then perform a dilation + erosion to
    # close gaps in between object edges
    edged = cv2.Canny(gray, 70, 240)

    edged = cv2.dilate(edged, None, iterations=1)
    edged = cv2.erode(edged, None, iterations=1)

    #save a copy of image after contour, we will later use to obtain all points enclosed in the contour

    frame=edged.copy()
    cnts, hierarchy = cv2.findContours(frame, cv2.RETR_EXTERNAL,cv2.CHAIN_APPROX_NONE)[-2:]

    #obtain the outmost contour for the plane
    if len(cnts)>0:
        find_contour=True
        cnt=max(cnts,key=cv2.contourArea)

        return (find_contour,cnt,image)
    else:
        return (find_contour,0,None)




"""
#Begin Checking

print " "
print "################################################################### RUNNING PLANE CHECKING #######################################################################################"
print " "
"""
#print "Start"
#print time.time()
#pointcloud=np.load("pc.npy")
#
#try:
#    img = cv2.imread("bin.png")
#except:
#    print "UV map or RGB info not provided, cannot show contours of segments on the origianl image"
#
#seg_masks=[]
#
#for filename in glob.iglob('perception/*.npy'):
#    seg_mask=np.load('%s' % filename)
#    seg_masks.append(seg_mask)
#
#plane_info=rate_plane(pointcloud,depth_threshold=0.4,masks=seg_masks,img=img)
#print "Finish"
#print time.time()


