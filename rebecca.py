import time
import cv2 
import apriltag
import math
import numpy
import json
# from interbotix_xs_modules.arm import InterbotixManipulatorXS


#Globals
rotate = []
scale = 0
origin = []

#Colors
color_red = [[0,100,100], [10,255,255]]
color_green = [[0,100,100], [10,255,255]]
color_gray = [[0,100,100], [10,255,255]]

ran = False

box0 = None #testing box
box1 = None #april tag id = 1
box2 = None #april tag id = 2
box3 = None #april tag id = 3


#findColors
# def findColor(color):
#     #return centerx and centery

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
    # Initialize the arm module along with the pointcloud and armtag modules
    bot = InterbotixManipulatorXS("rx200", moving_time=1.5, accel_time=0.75)


    # Initial Wake Up Position
    bot.arm.set_ee_pose_components(x = 0.3, z = 0.2)

    bot.gripper.open()
    
    #time.sleep(0.5)

    # pick up all the objects and drop them in a virtual basket in front of the robot
    bot.arm.set_ee_pose_components(x=x, y=y, z=z, pitch=pitch)
    bot.gripper.close()
    bot.arm.set_ee_pose_components(x=x, y=y, z=z, pitch=pitch)

    time.sleep(0.5)

    # Final Destination (Cup drop)
    bot.arm.set_ee_pose_components(x=0.2, y=-0.2, z=0.1)
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
            pickAndPlace(x=box0.x,y=-(box0.y),z=0.05,pitch=0.5, finalx=0.0 , finaly=0.2)


def pickBox0ToBox1():
    if box0 is None:
        print("Box 0 NOT FOUND")
    if box1 is None:
        print("Box 0 NOT FOUND")
    else:
        print("box0:", box0)
        print("box1:", box1)
        pickAndPlace(x=box0.x,y=-(box0.y),z=0.05,pitch=0.5, finalx=box1.x, finaly=-(box1.y))


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
        if r.tag_id == 0:
            print("Found Test Tag!")
            box0 = block
        elif r.tag_id == 1:
            print("Found Box 1 Tag!")
            box1 = block
        elif r.tag_id == 2:
            print("Found Box 2 Tag!")
            box2 = block
        elif r.tag_id == 3:
            print("Found Box 3 Tag!")
            box3 = block
        else:
            print("Found Tag: ", r.tag_id, "?")


def captureVideo(): 
    # define a video capture object 
    vid = cv2.VideoCapture(0) 

    if not vid.isOpened():
        print("Cannot open camera")
        exit()

    while(True): 
        
        # Capture the video frame by frame 
        ret, frame = vid.read() 

        # if frame is read correctly ret is True
        if not ret:
            print("Can't receive frame (stream end?). Exiting ...")
            break

        # Find the positions of the end cups
        getApril(frame)

        #Pick Up Box 0
        pickBox0ToHardCoded()

 

        #given pink
        ##findcolor nad pick and place
        # block = findColor(color_red)
        # pickAndPlace(block[0],-(block[1]),0.025,0.5, box1[0], box1[1])
        # pickAndPlace(block[0],-(block[1]),0.025,0.5, .02, .05)


        # Display the resulting frame 
        cv2.imshow('frame', frame)

        
        # the 'q' button is set as the 
        # quitting button you may use any 
        # desired button of your choice 
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
    
# pickAndPlace(0.0,0.2,0.04,0.5)


# pickAndPlace(block[0],-(block[1]),0.025,0.5)
