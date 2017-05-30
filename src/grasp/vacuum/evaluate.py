import numpy as np
import math
import cv2
from scipy.spatial import distance as dist
import pcl
import glob
import json

# planes are rated on a 10 point base system
Score=10


def pcd2image(plane):
    """
    Convert pointcloud of the extraced plane to 2d image that has a value of either black or white per pixel
    Input:
    Numpy array of Pointcloud of the extracted plane
    Output:
    A 2D black and white image
    A 2D map as python list with the same dimension of image that contains the index to the original point to the numpy array for the plane

    """
    global plane_image,plane_map,lengthPerPixel

    #remove all the (0,0,0) readings from the plane segments
    mask = np.all(plane == 0, axis=1)
    plane = plane[~mask]
    x_min=np.amin(plane[:,0])
    x_max=np.amax(plane[:,0])
    y_min=np.amin(plane[:,1])
    y_max=np.amax(plane[:,1])

    ratio=(y_max-y_min)/(x_max-x_min)
    num_points=len(plane)

    pixelLength_x=int(math.sqrt(num_points/ratio))
    pixelLength_y=int(ratio*pixelLength_x)

    #use 2 times the larger of the x, y pixel length as the pixel dimension for the image

    image_dimension=2*max(pixelLength_x, pixelLength_y)

    lengthPerPixel=(x_max-x_min)/pixelLength_x
    plane_image=np.full((image_dimension,image_dimension,3),220,dtype="uint8")
    plane_map= [[[] for j in xrange(image_dimension)] for i in xrange(image_dimension)]

    #find out the minimum x and y value on the image

    x_edge=-0.5*(x_max-x_min)*image_dimension/pixelLength_x+0.5*(x_max+x_min)
    y_edge=-0.5*(y_max-y_min)*image_dimension/pixelLength_y+0.5*(y_max+y_min)

    for index,point in enumerate(plane):
        pixel_X=max(0,int((point[0]-x_edge)/lengthPerPixel)-1)
        pixel_Y=max(0,int((point[1]-y_edge)/lengthPerPixel)-1)
        plane_image[pixel_Y][pixel_X]=np.array([50,0,70],dtype="uint8")
        plane_map[pixel_Y][pixel_X].append(index)


    return (plane_image,plane_map)

def check_size(contour):
    """
    Check if the size of the plane is large enough for the vacuum hose, currently the thereshold is set at 5cm
    Input:
    Contour for the observed plane
    Output:
    Boolean value indicating if the size of the object is large enough to for the vacuum nozzle
    ratio between the smaller dimension of the minimum bounding rectangle over min diameter of the vacuum cup
    Fullness: The extent the given contour fits a rectangle or Ellipse

    """
    MIN_DIAMETER = 0.035
    MIN_FILL = 0.8
    #compute if the shape is close to a rectangle or circle enough, set 80% as threshold
    center_rect,dimension_rect,rotation_rect=cv2.minAreaRect(contour)
    center_circle,axis_circle,rotation_circle=cv2.fitEllipse(contour)
    area=cv2.contourArea(contour)
    FitRect=area/(dimension_rect[0]*dimension_rect[1])
    FitCircle=4*area/(math.pi*axis_circle[0]*axis_circle[1])
    IsPolygon=FitRect>MIN_FILL or FitCircle>MIN_FILL
    ratio=min(dimension_rect)*lengthPerPixel*1.00/MIN_DIAMETER

    if IsPolygon and ratio>1:
        return (True,ratio,max(FitRect,FitCircle))
    else:
        return (False,ratio,max(FitRect,FitCircle))

def check_top(surfacePoints,coeficients):

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
    for pixels in surfacePoints:
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


def mapPlane_pixel2Pointcloud_pixel(edgepoints):

    """
    Map the contour pixels founded on the 2d projection of the plane back to the corresponding pixels
    on the original pointcloud(including the planes and other objects)

    input:
    edgepoints: Largest contour obtained by CV2.findContour method

    Output:
    edge_Depths: 3d Catersian coordinates of the edge points in python list
    pixel_in_pointcloud: pixel values for the plane edges in the original pointcloud in a python list

    """
    global edge_Depths, pixel_in_pointcloud
    edge_Depths=[]
    pixel_in_pointcloud=[]
    for point in edgepoints:
        pixels=plane_map[point[1]][point[0]]
        if len(pixels)>0:
            edge_Depths.append(plane[pixels[0]])
    #print edge_Depths

    #find the catersian coordinates of the edge points in the original point cloud

    for coordinate in edge_Depths:
        indice = np.where(np.all(pointcloud == coordinate, axis=-1))

        try:
            pixel_in_pointcloud.append([int(indice[1]),int(indice[0])])
        except:
            continue

    return (edge_Depths,pixel_in_pointcloud)

def find_enclosingPixels(contour):
    """
    use test if inside polygon method to get all enclosing pixels inside a given contour

    Input:
    pixels: edge pixels in the format of python list

    Output:
    enclosing pixels: all the pixels contained inside the polygon defined by the edge pixel
    in python list

    """
    global enclosing_pixels_pointcloud
    enclosing_pixels_pointcloud=[]


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
    return enclosing_pixels_pointcloud


def find_center(points):


    return np.mean(points, axis=0)

def check_surroundings(contour):

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
            else:
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


def check_flatness(segment):
    """
    check flatness of the input segment and return the segmented plane

    Input:

    segment: point cloud segment in .pcd

    Output:

    plane_percentage: the percentage of the input pointcloud that can fit inside a plane
    mode: the plane equation in a python list [a,b,c,d]
    cloud_plane: The fitted plane in .pcd

    """
    global seg_pcd
    seg = segment.make_segmenter_normals(ksearch=50)
    seg.set_optimize_coefficients(True)
    seg.set_model_type(pcl.SACMODEL_NORMAL_PLANE)
    seg.set_normal_distance_weight(0.1)
    seg.set_method_type(pcl.SAC_RANSAC)
    seg.set_max_iterations(100)
    seg.set_distance_threshold(0.03)
    indices, model = seg.segment()
    cloud_plane = segment.extract(indices, negative=False)
    seg_pcd=segment.extract(indices,negative=True)
    plane_percentage=1.0*cloud_plane.size/segment.size
    return (plane_percentage, model, cloud_plane)


#Begin Checking
print " "
print "################################################################### RUNNING PLANE CHECKING #######################################################################################"
print " "
pointcloud=np.load("pc.npy").astype(np.float32)
try:
    img = cv2.imread("bin.png")
except:
    print "UV map or RGB info not provided, cannot show contours of segments on the origianl image"
filecount=0
seg_pcd=None
Plane_info=[]

for filename in glob.iglob('segments/*.npy'):
     print filename
     mask=np.load('%s' % filename)
     segment_npy= pointcloud[mask.nonzero()]
     segment_npy=segment_npy.astype(np.float32)
     seg_pcd=pcl.PointCloud(segment_npy)
     while seg_pcd.size>1000:
         deductions = {'Orientation': 0, 'top_Coverage': 0, 'Side_coverage':0, 'side_height':0, 'size':0}
         global plane_equation
         try:
             plane_percentage,plane_equation,plane_pcd=check_flatness(seg_pcd)
             if plane_pcd.size == 0:
                 break
         except:
             break

         if plane_pcd.size>500:

             plane = np.asarray(plane_pcd)
             image,plane_map = pcd2image(plane)


             #calculate the orientation of the plane using plane equation

             angle=math.acos(plane_equation[2])
             angle=angle*180.0/math.pi
             if abs(angle)>90:
                 angle=180-abs(angle)

                 #For orientation greater than 20 degree, take 0.5 point per 10 degree on top of that

             gray = cv2.cvtColor(image, cv2.COLOR_BGR2GRAY)
             gray = cv2.GaussianBlur(gray, (7, 7), 0)

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
                 cnt=max(cnts,key=cv2.contourArea)
                 cv2.drawContours(image,cnt, -1, (0, 255, 0), 2)

                 #check if the size of the plane can fit in the suction nozzle
                 size_good,ratio, fitness=check_size(cnt)

                 #find the pixels for the plane edge in the original pointcloud
                 edgepoints=np.reshape(cnt,(cnt.size/2,2))
                 edge_Depths,pixel_in_pointcloud=mapPlane_pixel2Pointcloud_pixel(edgepoints)

                 #Check around the edge of the plane and detect if it is likely covered around the edge
                 edge_gap, edge_covered, edge_missing, edge_close, gap_Depth, converage_height=check_surroundings(pixel_in_pointcloud)

                 #check if the top of the plane is likely covered
                 #Use those edge pixels to find all the enclosing pixels inside the contour
                 enclosing_pixels_pointcloud=find_enclosingPixels(pixel_in_pointcloud)

                 center=find_center(plane)
                 percentage,average_distance=check_top(enclosing_pixels_pointcloud,plane_equation)

                 Collision=True
                 if percentage<0.02 and ratio>1:
                     Collision=False




                 #Here are the priliminary scorings for each features

                 if ratio<1:
                     deductions['size']=(1/ratio)**2*20-20
                 if fitness<0.8:
                     deductions['size']+=(0.8-fitness)*20

                 deductions['Side_coverage']=max(0,edge_close+edge_covered*converage_height*400-edge_gap*gap_Depth*100)


                 if percentage>0.02 and average_distance>0:
                     deductions['top_Coverage']=average_distance*1000*percentage*100*0.5

                 if angle>10:
                     deductions['Orientation']=((angle-10)/10)**2*0.5

                 Finalscore=Score-sum(deductions.values())

                 Plane_info.append({'segment': filename,'center':center.tolist(),'orientation':plane_equation, 'score':Finalscore, 'flatness':plane_percentage, 'tilting_angle': angle, 'size': ratio, 'convexity':fitness,'close_edge': edge_close, 'gap': edge_gap,'blockage': edge_covered, 'edge_inconclusive':edge_missing, 'gap_Depth':gap_Depth, 'blockage_height':converage_height,'top_coverage':percentage, 'cover_height': average_distance })

                 # printing information to the terminal

                #  print('Plane angle is {} degrees, taking {} points off.'.format(angle,deductions['Orientation']))
                #  print " "
                #  if Collision is True:
                #      print "No valid placement for the suction cup found"
                #  else:
                #      print "Valid placement for the suction cup found"
                #  print " "
                #  print('{} percent of the original can be approximated as a plane.'.format(plane_percentage))
                #  print " "
                #  print('The smaller dimension is {} required, its shape can be approximated as a rectangle or ellipse by {} %, taking {} off.'.format(ratio,fitness*100,deductions['size']))
                #  print " "
                #  print('The side of the plane is clean, {} percent of surroundings are confirmed gaps, with an average dip of {} cm.  {} percent of surroundings are confirmed blockage, with an average height of {} cm. {} percent of surroundings have insufficient readings. {} percent of the surroundings have readings that are close to the edge. Taking {} points off'.format(edge_gap*100,gap_Depth*100,edge_covered*100,converage_height*100,edge_missing*100,edge_close*100, deductions['Side_coverage']))
                #  print " "
                #  print('The top of the plane should be clean, {} percent of measurements is above the 1 cm thereshold, with an average displacement of {} cm. Taking {} points off.'.format(percentage*100,average_distance*100,deductions['top_Coverage']))

                #  print " "
                #  print ("The final score for the plane is {}" .format(Finalscore))
                #  print " "
                #  print " "
                #  print "                                                               Press 'Space' for the next plane                                                                              "
                #  print " "


                # #  try:

                #      #if uv map and color image exists, map depth pixels to RGB pixels for drawing segments on the original image
                #      ctr = np.array(pixel_in_pointcloud).reshape((-1,1,2)).astype(np.int32)
                #      img2=img.copy()

                #      M = cv2.moments(ctr)
                #      cX = int(M["m10"] / M["m00"])
                #      cY = int(M["m01"] / M["m00"])
                #      cv2.drawContours(img2,[ctr],0,(0,255,0),2)
                #      cv2.putText(img2, str(Finalscore)[:4], (cX-10 , cY ),cv2.FONT_HERSHEY_SIMPLEX, 0.8, (0, 50, 255), 3)

                #      cv2.imshow("image",img2)
                #      k = cv2.waitKey(0)
                #      if k == 27:

                #          cv2.destroyAllWindows()


                #      # Otherwise, just show images of the segments

                #  except:
                #      # Otherwise, just show images of the segments
                #      cv2.imshow("segments",image)
                #      k = cv2.waitKey(0)
                #      if k == 27:         # wait for ESC key to exit

                #          cv2.destroyAllWindows()

             else:
                 print " no contour founded,continue to the next segment"
                 continue




#Load all plane information into a python list where each of the sublist is a python dictionary


data_file="planes.txt"
with open(data_file, 'wb') as dump:
    dump.write(json.dumps(Plane_info))

    print"All plane information has been succesfully recorded!"
