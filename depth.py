from PIL import Image
import numpy as np
import cv2

def readData():
            name = "new_dataset_p/1.pcd"
            f = open(name,"r")
            lines = f.readlines()
            w = int(lines[6].split(" ")[1])
            h = int(lines[7].split(" ")[1])
            points = lines[11:]

            depth = (np.array([float(x.split(' ')[2]) for x in points])).reshape((h,w))
            depth[depth!=depth]= np.amax(depth) # REMOVE NANS


            depth = (depth *255. / np.max(depth)).astype(np.uint8) #NORMALIZE
            depth_image = Image.fromarray(depth)
            
            depth_cv = np.array(depth_image)

            cv2.namedWindow('depth_image',cv2.WINDOW_NORMAL) 
            cv2.imshow('depth_image',depth_cv)

            cv2.waitKey(0)


if __name__ == "__main__":
    readData()