import numpy as np
import cv2 as cv
import glob
import time



def captureVideo(): 
    number = 0
    # define a video capture object 
    vid = cv.VideoCapture(0) 

    if not vid.isOpened():
        print("Cannot open camera")
        exit()

    while(True): 
        while (number <= 10):
            # Capture the video frame by frame 
            ret, frame = vid.read() 

            # if frame is read correctly ret is True
            if not ret:
                print("Can't receive frame (stream end?). Exiting ...")
                break

            # getApril(frame)
            # Display the resulting frame 
            cv.imshow('frame', frame)
            time.sleep(2)
            filename = "calibrateMe" + str(number) + ".jpg"
            cv.imwrite(filename, frame)
            number += 1
        
        # the 'q' button is set as the 
        # quitting button you may use any 
        # desired button of your choice 
        if cv.waitKey(1) & 0xFF == ord('q'): 
            break

    # After the loop release the cap object 
    vid.release() 
    # Destroy all the windows 
    cv.destroyAllWindows() 

captureVideo()



# termination criteria
criteria = (cv.TERM_CRITERIA_EPS + cv.TERM_CRITERIA_MAX_ITER, 30, 0.001)
 
# prepare object points, like (0,0,0), (1,0,0), (2,0,0) ....,(6,5,0)
objp = np.zeros((6*9,3), np.float32)
objp[:,:2] = np.mgrid[0:9,0:6].T.reshape(-1,2)
 
# Arrays to store object points and image points from all the images.
objpoints = [] # 3d point in real world space
imgpoints = [] # 2d points in image plane.
 
images = glob.glob('*.jpg')
##20 images from a couple differnt positions
 
for fname in images:
    img = cv.imread(fname)
    gray = cv.cvtColor(img, cv.COLOR_BGR2GRAY)
 
    # Find the chess board corners
    ret, corners = cv.findChessboardCorners(gray, (9,6), None)
 
    # If found, add object points, image points (after refining them)
    if ret == True:
        objpoints.append(objp)
 
        corners2 = cv.cornerSubPix(gray,corners, (11,11), (-1,-1), criteria)
        imgpoints.append(corners2)
 
        # Draw and display the corners
        cv.drawChessboardCorners(img, (9,6), corners2, ret)
        cv.imshow('img', img)
        cv.waitKey(500)


        ##scale matrux by how big the squares actually are
    else:
        print("Calibration Failed")

        
ret, mtx, dist, rvecs, tvecs = cv.calibrateCamera(objpoints, imgpoints, gray.shape[::-1], None, None)
print("Matrix: ",mtx)
print("Dist", dist)
print("Rvecs", rvecs)
print("Tvecs", tvecs)
 
cv.destroyAllWindows()
