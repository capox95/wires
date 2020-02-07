from PIL import Image
import numpy as np
import cv2

file_path = "dataset_1/"
file_processed_dataset = "dataset_prova/"

def readData():

    with open(file_path+"/test.txt", "r") as f:
        for line in f:

            name = line.strip().split('.')

            f = open(file_path+line.strip(),"r")
            lines = f.readlines()
            w = int(lines[6].split(" ")[1])
            h = int(lines[7].split(" ")[1])
            points = lines[11:]

            # array of 'gray' columns
            gray = np.array([int(x.split(' ')[5]) for x in points]).reshape((h,w)).astype(np.uint8)
            gray_image = Image.fromarray(gray)
           
            save_gray = np.array(gray_image)
            filename_gray = file_processed_dataset+name[0]+"_gray.jpg"
            cv2.imwrite(filename_gray, save_gray)


if __name__ == "__main__":
    readData()