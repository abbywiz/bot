import time
import cv2 
import apriltag
import math
import numpy
import json
from interbotix_xs_modules.arm import InterbotixManipulatorXS
#Globals
rotate = []
scale = 0
origin = []

#Colors
color_pink= [[0,150,100],[30,255,255]]
color_purple= [[145,80,80],[165,255,255]]
color_green = [[70,100,100],[80,255,255]]

ran = False

box0 = None #testing box

box1 = None #april tag id = 1 
box2 = None #april tag id = 2 #Green
box3 = None #april tag id = 3

bxOffset = 0.01
byOffset = 0.05
fxOffset = 0
fyOffset = 0.04

class Coord:
    def __init__(self,x,y):
        self.x=x
        self.y=y
    def __str__(self):
        return f"(X: {self.x}, Y: {self.y})"

def pixelToBase(x, y):
    x = x - origin[0]
    y = y - origin[1]
    rotated = rotate.T @ (numpy.array([x,y]).T)
    newxy = rotated * scale
    return  Coord(newxy[0], newxy[1])

# Accepts real world coordinates for where to put the gripper
# In respect to the base frame
def pickAndPlace(x,y,z,pitch,finalx, finaly):
    # Initialize the arm module al,ong with the pointcloud and armtag modules
    bot = InterbotixManipulatorXS("rx200", moving_time=1.5, accel_time=0.75)

    # Initial Wake Up Position
    bot.arm.set_ee_pose_components(x = 0.3, z = 0.3)
    bot.gripper.open()
    
    # Move gripper to point to the ground
    bot.arm.set_single_joint_position("wrist_angle", numpy.pi/2.0)

    # Move gripper to given x and y position
    bot.arm.set_ee_pose_components(x=x, y=y, z=0.2,pitch =1.2)

    # Make Sure the gripper is pointing down
    bot.arm.set_single_joint_position("wrist_angle", numpy.pi/2.0)

    #Move Down to the block
    bot.arm.set_ee_cartesian_trajectory(z=-0.15)

    #Pick Up the block
    bot.gripper.open()
    bot.gripper.close()


    bot.arm.set_ee_pose_components(x=x, y=y, z=0.2,pitch=0.5)
    time.sleep(0.5)

    # Final Destination (Cup drop)
    bot.arm.set_ee_pose_components(x=finalx, y=finaly, z=0.3)
    bot.gripper.open()
    
    # Final Pre Sleep Position
    bot.arm.set_ee_pose_components(x=0.3, z=0.2)
    bot.arm.go_to_sleep_pose()
def pickBox0ToHardCoded():
    if box0 is None:
        print("Box 0 NOT FOUND")
    else:
        print("box1:", box1)
        if isinstance(box0, Coord):
            pickAndPlace(x=box0.x,y=-(box0.y),z=0.05,pitch=0.0, finalx=0.0 , finaly=0.2)
            
def pickBox0ToBox1(b):
    # if box0 is None:
    #     print("Box 0 NOT FOUND")
    if box1 is None:
        print("Box 1 NOT FOUND")
    else:
        #print("box0:", box0)
        print("box1:", box1)
        pickAndPlace(x=(b.x)+0.01,y=-(b.y)+0.06,z=0.05,pitch=0.0, finalx=box1.x, finaly=-(box1.y)+0.04)
        #pickAndPlace(x=b.x+0.01,y=-(b.y)+0.02,z=0.05,pitch=0.0, finalx=box1.x, finaly=-(box1.y))


def pickGreenToBox(b):
    if box2 is None:
        print("Box Green NOT FOUND")
    else:
        print("box2:", box1)
        pickAndPlace(x=(b.x)+bxOffset,y=-(b.y)+byOffset,z=0.05,pitch=0.0, finalx=box2.x+fxOffset, finaly=-(box2.y)+fyOffset)

def pickRedToBox(b):
    if box1 is None:
        print("Box Red NOT FOUND")
    else:
        print("box1:", box1)
        pickAndPlace(x=(b.x)+bxOffset,y=-(b.y)+byOffset,z=0.05,pitch=0.0, finalx=box2.x+fxOffset, finaly=-(box2.y)+fyOffset)

def pickBlueToBox(b):
    if box3 is None:
        print("Box Blue NOT FOUND")
    else:
        print("box3:", box3)
        pickAndPlace(x=(b.x)+bxOffset,y=-(b.y)+byOffset,z=0.05,pitch=0.0, finalx=box3.x+fxOffset, finaly=-(box3.y)+fyOffset)


def getApril(img):
    global box0,box1,box2,box3
    image = img
    gray = cv2.cvtColor(image, cv2.COLOR_BGR2GRAY)
    # define the AprilTags detector options and then detect the AprilTags
    options = apriltag.DetectorOptions(families="tag36h11")
    detector = apriltag.Detector(options)
    results = detector.detect(gray)
    # loop over the AprilTag detection results
    for r in results:
        # extract the bounding box (x, y)-coordinates for the AprilTag
        # and convert each of the (x, y)-coordinate pairs to integers
        (ptA, ptB, ptC, ptD) = r.corners
        centerx = (int(ptA[0]) + int(ptB[0]) + int(ptC[0]) + int(ptD[0])) / 4
        centery = (int(ptA[1]) + int(ptB[1]) + int(ptC[1]) + int(ptD[1])) / 4
        #points in camera space
        #transform to coords in base link frame of reference
        block =  pixelToBase(centerx,centery)

        #store base link coords in correct variable based on april tag id
        print("ID IS: ",r.tag_id)
        if r.tag_id == 0:
            print("Found Test Tag!")
            box0 = block
        elif r.tag_id == 1:
            print("Found Box 1 Tag!")
            box1 = block
        elif r.tag_id == 2:
            print("Found Box 2 GREEN Tag!")
            box2 = block
        elif r.tag_id == 3:
            print("Found Box 3 Tag!")
            box3 = block
        else:
            print("Found Tag: ", r.tag_id, "?")

def findColor(color, image):

    _,frame = vid.read()
    hsv = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)
    lower_bound = numpy.array(color[0])
    upper_bound = numpy.array(color[1])

    mask = cv2.inRange(hsv, numpy.array(lower_bound), numpy.array(upper_bound))
    #result = cv2.bitwise_and(frame,frame,mask=mask)
    #cv2.imshow('result',result)
    contours, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
    
    # if len(contours) == 0:
    #     print("No color in range found! Range: ", str(lower_bound), str(upper_bound))

    boxes = []
    for c in contours:
        x,y,w,h = cv2.boundingRect(c)
        image = cv2.rectangle(image, (x,y), (x+w, y+h), (0,0,255), 2)
        cv2.imshow("frame", image)
        corners = numpy.array([(x,y), (x+w,y), (x+w, y+h), (x, y+h)],dtype=numpy.int32)
        centerx = (int(corners[0][0]) + int(corners[1][0]) + int(corners[2][0]) + int(corners[3][0])) / 4
        centery = (int(corners[0][1]) + int(corners[1][1]) + int(corners[2][1]) + int(corners[3][1])) / 4
        center= pixelToBase(centerx,centery)
        print("center:",center)
        #boxes.append([x_center,y_center])
        return center


def captureVideo(): 
    # define a video capture object
    vid = cv2.VideoCapture(2) 

    if not vid.isOpened():
        print("Cannot open camera")
        exit()

    mtx = numpy.array([[1.56035688e+03, 0.00000000e+00, 7.44074967e+02],
                        [0.00000000e+00, 1.55382936e+03, 6.04020129e+02],
                        [0.00000000e+00, 0.00000000e+00, 1.00000000e+00]])
    dist =  numpy.array([[ 7.27892841e-01, -8.78018308e+00, 3.35198169e-02, 3.91299576e-02, 4.48245023e+01]]) 

    while(True): 
      
        # Capture the video frame by frame 
        ret, frame = vid.read() 
        #frame = cv2.imread('pic.jpg', 1) 


        # if frame is read correctly ret is True
        if not ret:
            print("Can't receive frame (stream end?). Exiting ...")
            break

        frame = cv2.undistort(frame, mtx, dist, None, None)
        cv2.namedWindow("frame")
        
        # Find the positions of the end cups
        getApril(frame)
        
        #Pick Up Green Box and Put It In Green Cup
        boxes = findColor(color_green,frame)
        pickGreenToBox(b)

        # boxes = findColor(color_blue,frame)
        # pickBlueToBox(b)


        # boxes = findColor(color_red,frame)
        # pickRedToBox(b)


        # #Pick Up Box 0
        # # pickBox0ToHardCoded()
        # pickBox0ToBox1(boxes)
        # boxes = findColor(color_green,dst)
        # pickBox0ToBox1(boxes)
        # boxes = findColor(color_blue,dst)
        # pickBox0ToBox1(boxes)
 
        # Display the resulting frame 
        cv2.imshow('frame', frame)
        
        # 'q' to quit
        if (cv2.waitKey(1) & 0xFF == ord('q')) or ran == True: 
            break
    # After the loop release the cap object 
    vid.release() 
    # Destroy all the windows
    cv2.destroyAllWindows() 

if __name__=="__main__": 
    with open('config.json') as openfile:
        json_object = json.load(openfile)
    origin = numpy.array(json_object["origin"])
    scale = (json_object["scale"])
    rotate = numpy.array(json_object["rotate"])
    if origin is None or scale is None or rotate is None:
        print("Error with camera config")
    if ran == False:
        ran = True
        captureVideo()

# cap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tureVideo()
#    ''' i = 0
#     while i<3:
#         pickAndPlace(0.0,0.2,0.04,0.5)
#         i += 1'''
    