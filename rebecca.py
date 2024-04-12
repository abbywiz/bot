import time
import cv2 
import apriltag
import math
import numpy
import json
from interbotix_xs_modules.arm import InterbotixManipulatorXS

rotate = []
scale = 0
origin = []
ran = False
box1 = [] #april tag id = 1
box2 = [] #april tag id = 2
box3 = [] #april tag id = 3
color_red = [[0,100,100], [10,255,255]]
color_green = [[0,100,100], [10,255,255]]
color_gray = [[0,100,100], [10,255,255]]


#findColors
# def findColor(color):
#     #return centerx and centery

def pixelToBase(x, y):
    x = x - origin[0]
    y = y - origin[1]
    rotated = rotate.T @ (numpy.array([[x,y]]).T)
    return  rotated * scale

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
    bot.arm.set_ee_pose_components(x=x, y=y, z=0.075, pitch=pitch)
    bot.arm.set_ee_pose_components(x=x, y=y, z=0.075, pitch=pitch)
    bot.gripper.close()
    bot.arm.set_ee_pose_components(x=x, y=y, z=0.075, pitch=pitch)

    time.sleep(0.5)

    # Final Destination (Cup drop)
    bot.arm.set_ee_pose_components(x=finalx, y=finaly, z=0.2)
    bot.gripper.open()
    


    # Final Pre Sleep Position
    bot.arm.set_ee_pose_components(x=0.3, z=0.2)
    bot.arm.go_to_sleep_pose()

def getApril(img):

    image = img
    gray = cv2.cvtColor(image, cv2.COLOR_BGR2GRAY)

    # define the AprilTags detector options and then detect the AprilTags
    # in the input image
    #TODO: make sure this family is right
    options = apriltag.DetectorOptions(families="tag36h11")
    detector = apriltag.Detector(options)
    results = detector.detect(gray)

    # loop over the AprilTag detection results
    # if results.size() > 0:
    boxes = []
    # for i in range(1,4):
    for r in results:
        print("Found April Tag!")

        # print(r.tag_id)
        # if r.tag_id == i:
        # if results[0] is not None:
        # extract the bounding box (x, y)-coordinates for the AprilTag
        # and convert each of the (x, y)-coordinate pairs to integers
        (ptA, ptB, ptC, ptD) = r.corners

        centerx = (int(ptA[0]) + int(ptB[0]) + int(ptC[0]) + int(ptD[0])) / 4
        centery = (int(ptA[1]) + int(ptB[1]) + int(ptC[1]) + int(ptD[1])) / 4
        #points in camera space

        #transform to coords in base link frame of reference
        block = pixelToBase(centerx,centery)
        boxes.append(block)
    # if len(boxes) == 3:
    if len(boxes) > 0:
        box1 = boxes[0]
    #     box2 = boxes[1]
    #     box3 = boxes[2]




def captureVideo(): 
    # define a video capture object 
    vid = cv2.VideoCapture(2) 

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


        # if len(box1) != 0 and len(box2) != 0 and len(box3) != 0:
        
        getApril(frame)
        if len(box1) > 0:
            pickAndPlace(box1[0],-(box1[1]),0.025,0.5, .02, .05)

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


# cap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tureVideo()
#    ''' i = 0
#     while i<3:
#         pickAndPlace(0.0,0.2,0.04,0.5)
#         i += 1'''
    
# pickAndPlace(0.0,0.2,0.04,0.5)


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



# pickAndPlace(block[0],-(block[1]),0.025,0.5)
