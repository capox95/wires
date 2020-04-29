'''
Calculate depth images from pointcloud and provide segmented binary images of the wires 
'''

from PIL import Image
import numpy as np
import cv2
import sys

def readData(file_path):
    with open(file_path+"test.txt", "r") as f:
        for line in f:

            name = line.strip().split('.')

            string = file_path + name[0] + "p." + name[1]
            print(string)

            f = open(string,"r")
            lines = f.readlines()
            w = int(lines[6].split(" ")[1])
            h = int(lines[7].split(" ")[1])
            points = lines[11:]

            depth = (np.array([float(x.split(' ')[2]) for x in points])).reshape((h,w))
            depth[depth!=depth]= np.amax(depth) # REMOVE NANS


            depth = (depth *255. / np.max(depth)).astype(np.uint8) #NORMALIZE
            depth_image = Image.fromarray(depth)

            depth_cv = np.array(depth_image)
            ret,th1 = cv2.threshold(depth_cv,127,255,cv2.THRESH_BINARY_INV)
            kernel = np.ones((5,5),np.uint8)
            dilation = cv2.dilate(th1,kernel,iterations = 1)

            filename = file_path+name[0]+".jpg"
            cv2.imwrite(filename, dilation) 


if __name__ == "__main__":
    if len(sys.argv) < 2:
	print("filepath expected!")

    file_path = sys.argv[1]
    readData(file_path)
