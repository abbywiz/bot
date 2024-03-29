import time
import cv2 
import apriltag
from interbotix_xs_modules.arm import InterbotixManipulatorXS


# Accepts real world coordinates for where to put the gripper
# We think it's in respect to the base frame
def pickAndPlace(x,y,z,pitch):
    # Initialize the arm module along with the pointcloud and armtag modules
    bot = InterbotixManipulatorXS("rx200", moving_time=1.5, accel_time=0.75)

    i= 0
    bot.arm.set_ee_pose_components(x = 0.3, z = 0.2)
    while i<3:

        # set initial arm and gripper pose
        
        bot.gripper.open()

        # get the ArmTag pose
        #bot.arm.set_ee_pose_components(y=-0.3, z=0.2) #home base?
        #time.sleep(0.5)

        ## TODO: Call set_ee_pose_matrix with T from base link to gripper
        # Transformation Matrix representing the transform from the /<robot_name>/base_link frame to the /<robot_name>/ee_gripper_link frame

        #Do we need base to gripper matrix here?

        # pick up all the objects and drop them in a virtual basket in front of the robot
        bot.arm.set_ee_pose_components(x=x, y=y, z=z+0.05, pitch=pitch)
        bot.arm.set_ee_pose_components(x=x, y=y, z=z, pitch=pitch)
        bot.gripper.close()

        bot.arm.set_ee_pose_components(x=x, y=y, z=z+0.05, pitch=pitch)

        bot.arm.set_ee_pose_components(x=0.0,y = -0.2 ,z=0.2)
        bot.gripper.open()
        
        i+=1

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
    #TODO: conditional statement "if april tag 1 ... put it here"
    if results.size() > 0:

        # print(r.tag_id)
        if results[0] == 0:
            # extract the bounding box (x, y)-coordinates for the AprilTag
            # and convert each of the (x, y)-coordinate pairs to integers
            (ptA, ptB, ptC, ptD) = r.corners
            ptB = (int(ptB[0]), int(ptB[1]))
            ptC = (int(ptC[0]), int(ptC[1]))
            ptD = (int(ptD[0]), int(ptD[1]))
            ptA = (int(ptA[0]), int(ptA[1]))
            #points in camera space

            #TODO: Find centroid of these four points
            #Use transformation matrix here
            #TODO: find transform from camera to gripper
            pickAndPlace(x,y,z)


            # cv2.solvePnP() idk what this is 
            # cv2.Rodrigues()


 
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


# cap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tureVideo()
#    ''' i = 0
#     while i<3:
#         pickAndPlace(0.0,0.2,0.04,0.5)
#         i += 1'''
    
pickAndPlace(0.0,0.2,0.04,0.5)