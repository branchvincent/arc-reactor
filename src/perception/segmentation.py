from status import CameraStatus
import matplotlib.pyplot as plt
import numpy as np

class Segmentation:

    def __init__(self):
        self.backgroundImage = None

    def setBG(self, pointCloud):
        self.backgroundImage = pointCloud

    def bgsubtract(self, pointcloud):
        bgsubpc = np.zeros((pointcloud.shape))
        for y in range(pointcloud.shape[0]):
            for x in range(pointcloud.shape[1]):
                if pointcloud[y][x][2] == 0:
                    continue
                if abs(pointcloud[y][x][0] - self.backgroundImage[y][x][0]) < 0.001 and abs(pointcloud[y][x][1] - self.backgroundImage[y][x][1]) < 0.001 and abs(pointcloud[y][x][2] - self.backgroundImage[y][x][2]) < 0.3:
                    #was a pixel in the background don't use this pixel
                    pass
                else:
                    bgsubpc[y][x] = pointcloud[y][x]
        
        return bgsubpc

def testSegmentation():
    seg = Segmentation()
    cam_status = CameraStatus()
    list_of_serial_nums = cam_status.depthCamera.get_online_cams()
    cam_status.poll()
    points = cam_status.cameraPointClouds[list_of_serial_nums[0]]
    color = cam_status.cameraColorImages[list_of_serial_nums[0]]
    seg.setBG(points)

    plt.imshow(np.reshape(color,(480,640,3)))
    plt.show()
    cam_status.poll()
    color = cam_status.cameraColorImages[list_of_serial_nums[0]]
    points = cam_status.cameraPointClouds[list_of_serial_nums[0]]
    outimage = seg.bgsubtract(points)
    finalimage = np.zeros((outimage.shape))
    for y in range(outimage.shape[0]):
        for x in range(outimage.shape[1]):
            if outimage[y][x][2] != 0:
                finalimage[y][x] = color[y][x]
    
    plt.imshow(np.reshape(finalimage,(480,640,3)))
    plt.show()

if __name__ == '__main__':
    testSegmentation()