# import the opencv library 
import cv2 
import apriltag

def getApril(img):

    image = img
    gray = cv2.cvtColor(image, cv2.COLOR_BGR2GRAY)

    options = apriltag.DetectorOptions(families="tag36h11")
    detector = apriltag.Detector(options)
    results = detector.detect(gray)


    # loop over the AprilTag detection results
    for r in results:
        # extract the bounding box (x, y)-coordinates for the AprilTag
        # and convert each of the (x, y)-coordinate pairs to a centroid
        (ptA, ptB, ptC, ptD) = r.corners
        tag = r.tag_id

        ptB = (int(ptB[0]), int(ptB[1]))
        ptC = (int(ptC[0]), int(ptC[1]))
        ptD = (int(ptD[0]), int(ptD[1]))
        ptA = (int(ptA[0]), int(ptA[1]))
        
        print("ptA: ", str(ptA))
        print("ptB: ", str(ptB))
        print("ptC: ", str(ptC))
        print("ptD: ", str(ptD))


        centerx = (int(ptA[0]) + int(ptB[0]) + int(ptC[0]) + int(ptD[0])) / 4
        centery = (int(ptA[1]) + int(ptB[1]) + int(ptC[1]) + int(ptD[1])) / 4

        centroid = (centerx, centery)
        # print(str(centerx), "," , str(centery))

        #TODO: Some conditional based on tag ID
        #send to pick and place
        #TODO: convert camera to world space!
        # These are point in camera space
  
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

        getApril(frame)
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




