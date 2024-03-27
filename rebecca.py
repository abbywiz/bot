import time
import cv2 
import apriltag
from interbotix_xs_modules.arm import InterbotixManipulatorXS


# To get started, open a terminal and type 'roslaunch interbotix_xsarm_perception xsarm_perception.launch robot_model:=wx200'
# Then change to this directory and type 'python pick_place.py'


# Accepts real world coordinates for where to put the gripper
def pickAndPlace(x,y,z):
    # Initialize the arm module along with the pointcloud and armtag modules
    bot = InterbotixManipulatorXS("rx200", moving_time=1.5, accel_time=0.75)

    # set initial arm and gripper pose
    bot.arm.set_ee_pose_components(x=0.3, z=0.2)
    bot.gripper.open()

    # get the ArmTag pose
    bot.arm.set_ee_pose_components(y=-0.3, z=0.2)
    time.sleep(0.5)


    ## TODO: find the transform from the robots base frame to the camera frame

    ## TODO: Get all april tag coordinates and put them in cluster[position]


    # pick up all the objects and drop them in a virtual basket in front of the robot
    bot.arm.set_ee_pose_components(x=x, y=y, z=z+0.05, pitch=0.5)
    bot.arm.set_ee_pose_components(x=x, y=y, z=z, pitch=0.5)
    bot.gripper.close()
    bot.arm.set_ee_pose_components(x=x, y=y, z=z+0.05, pitch=0.5)
    bot.arm.set_ee_pose_components(x=0.3, z=0.2)
    bot.gripper.open()
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
    #TODO: conditional statement "if april tag 1 ... put it here"
    if results.size() > 0:
        if results[0] == 0:
            # extract the bounding box (x, y)-coordinates for the AprilTag
            # and convert each of the (x, y)-coordinate pairs to integers
            (ptA, ptB, ptC, ptD) = r.corners
            ptB = (int(ptB[0]), int(ptB[1]))
            ptC = (int(ptC[0]), int(ptC[1]))
            ptD = (int(ptD[0]), int(ptD[1]))
            ptA = (int(ptA[0]), int(ptA[1]))

            # cv2.solvePnP() idk what this is 
            # cv2.Rodrigues()
            
            pickAndPlace(x,y,z)

            # print(r.tag_id)
            # return[ptB,ptC,ptD,ptA]

            # print("PtB: ", ptB)
            # print("PtC: ", ptC)
            # print("PtD: ", ptD)
            # print("PtA: ", ptA)
            # These are point in camera space

 
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

        getApril(frame)
        time.sleep(5)
        # Display the resulting frame 
        cv2.imshow('frame', frame)

        
        # the 'q' button is set as the 
        # quitting button you may use any 
        # desired button of your choice 
        if cv2.waitKey(1) & 0xFF == ord('q'): 
            break

    # After the loop release the cap object 
    vid.release() 
    # Destroy all the windows
    cv2.destroyAllWindows() 


captureVideo()