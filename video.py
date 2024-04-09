# import the opencv library 
import cv2 
import apriltag
import numpy
import matplotlib.pyplot as plt 
import math

global_params = None 
pixels = []
clicked = []
rotate = []
scale = 0

# function to display the coordinates of 
# of the points clicked on the image  
def click_event(event, x, y, flags, param): 

    global global_params
  
    # checking for left mouse clicks 
    if event == cv2.EVENT_LBUTTONDOWN: 
  
        # displaying the coordinates 
        # on the Shell 
        print(x, ' ', y) 
        clicked.append([x,y])

    if event == cv2.EVENT_RBUTTONDOWN:
        pixels.append([x,y])



# def getApril(img):

#     image = img
#     gray = cv2.cvtColor(image, cv2.COLOR_BGR2GRAY)

#     options = apriltag.DetectorOptions(families="tag36h11")
#     detector = apriltag.Detector(options)
#     results = detector.detect(gray)




#     # loop over the AprilTag detection results
#     for r in results:
#         # extract the bounding box (x, y)-coordinates for the AprilTag
#         # and convert each of the (x, y)-coordinate pairs to a centroid
#         (ptA, ptB, ptC, ptD) = r.corners
#         tag = r.tag_id

#         ptB = (int(ptB[0]), int(ptB[1]))
#         ptC = (int(ptC[0]), int(ptC[1]))
#         ptD = (int(ptD[0]), int(ptD[1]))
#         ptA = (int(ptA[0]), int(ptA[1]))
        
#         print("ptA: ", str(ptA))
#         print("ptB: ", str(ptB))
#         print("ptC: ", str(ptC))
#         print("ptD: ", str(ptD))


#         centerx = (int(ptA[0]) + int(ptB[0]) + int(ptC[0]) + int(ptD[0])) / 4
#         centery = (int(ptA[1]) + int(ptB[1]) + int(ptC[1]) + int(ptD[1])) / 4

#         centroid = (centerx, centery)
#         print(str(centerx), "," , str(centery))

#         #TODO: Some conditional based on tag ID
#         #send to pick and place
#         #TODO: convert camera to world space!
#         # These are point in camera space
  
# def captureVideo(): 
#     # define a video capture object 
#     vid = cv2.VideoCapture(2) 

#     if not vid.isOpened():
#         print("Cannot open camera")
#         exit()

#     while(True): 
        
#         # Capture the video frame by frame 
#         ret, frame = vid.read() 

#         # if frame is read correctly ret is True
#         if not ret:
#             print("Can't receive frame (stream end?). Exiting ...")
#             break

#         getApril(frame)
#         # Display the resulting frame 

#         cv2.imshow('frame', img)
        
#         # the 'q' button is set as the 
#         # quitting button you may use any 
#         # desired button of your choice 
#         if cv2.waitKey(1) & 0xFF == ord('q'): 
#             break

#     # After the loop release the cap object 
#     vid.release() 
#     # Destroy all the windows 
#     cv2.destroyAllWindows() 


def pixelToBase(roate, scale, Ox, Oy, x, y):
    scaledx, scaledy = numpy.array([x * scale, y * scale])
    rotated = numpy.dot(rotate, numpy.array([scaledx,scaledy]))
    origin = numpy.array([Ox,Oy])
    return numpy.subtract(rotated, origin)


def pixelToReal(x, y, scale, rotate):
    scaledx, scaledy = numpy.array([x * scale, y * scale])

    rotated = numpy.dot(rotate, numpy.array([scaledx,scaledy]))

    return rotated

    # return numpy.add(scaledxy, rotated)
    

def getAngle(pointA, pointB, pointC):
    pointZ = (pointB[0], pointA[1])
    deltaB = abs(pointB[1] - pointZ[1])
    deltaA = abs(pointA[0] - pointZ[0])
    angle = math.atan2(deltaB,deltaA)
    return angle



def getRotationMatrix(pointA, pointB, pointC):
    theta = getAngle(pointA, pointB, pointC)

    rotate = numpy.array([
        [math.cos(theta), -math.sin(theta)],
        [math.sin(theta), math.cos(theta)]
    ])
    print("this is rotate: ", rotate)
    return rotate


def getScale(pointA, pointB, pointC):
    width = (pointB[0] - pointC[0], pointB[1] - pointC[1])
    height = (pointA[0] - pointB[0], pointA[1] - pointB[1])

    # 5.5 inches is 0.1397 meters
    #width
    scaleW = 0.1397 / width[0]

    #11 inches is 0.104775 meters
    #hieght
    #4.125
    scaleH = 0.104775 / height[1]

    scale = (abs(scaleH) + abs(scaleW)) / 2.0

    return scale

    
#Click points from left to right
def getImg():

    global global_params

    vid = cv2.VideoCapture(0) 

    if not vid.isOpened():
        print("Cannot open camera")
        exit()

    ret, frame = vid.read() 
    

    mtx = numpy.array([[1.56035688e+03, 0.00000000e+00, 7.44074967e+02],
            [0.00000000e+00, 1.55382936e+03, 6.04020129e+02],
            [0.00000000e+00, 0.00000000e+00, 1.00000000e+00]])
    dist =  numpy.array([[ 7.27892841e-01, -8.78018308e+00, 3.35198169e-02, 3.91299576e-02, 4.48245023e+01]])

    dst = cv2.undistort(frame, mtx, dist, None, None)
    cv2.namedWindow("dst")

    scale = 0
    rotate = []
    
    while True:
        cv2.imshow('dst', dst)

        cv2.setMouseCallback('dst', click_event)

        for c in clicked:
            cv2.circle(dst, (c[0],c[1]), 3, (255,0,0), 5)


        if cv2.waitKey(1) & 0xFF == ord('q'): 
            break


        if len(clicked) == 3 and scale == 0:
            # So height O width
            #click in order of A,B,C
            scale = getScale(clicked[0], clicked[1], clicked[2])
            rotate = getRotationMatrix(clicked[0], clicked[1], clicked[2])

        if len(clicked) == 3 and len(pixels) == 1:
            coords = pixelToReal(pixels[0][0], pixels[0][1], scale, rotate)
            print(coords[0], coords[1])
            break
            


        

    vid.release() 
    # Destroy all the windows 
    cv2.destroyAllWindows() 

getImg()



