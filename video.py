# import the opencv library 
import cv2 
import apriltag
import numpy
import math
import json

pixels = []
clicked = []
rotate = []
scale = 0

def saveConfig(origin, scale, rotate):
    config = {
    "origin": origin.tolist(),
    "scale": scale,
    "rotate": rotate.tolist()
    }
    json_object = json.dumps(config, indent=4)
    with open("config.json", "w") as outfile:
        outfile.write(json_object)

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
        print("RIGHT: ", x, ' ', y) 
        pixels.append([x,y])



def pixelToBase(rotate, scale, origin, x, y):
    scaled = numpy.array([x * scale, y * scale])
    rotated = numpy.dot(rotate, scaled)
    return numpy.subtract(rotated, origin)


def pixelToReal(x, y, scale, rotate):
    scaledx, scaledy = numpy.array([x * scale, y * scale])
    rotated = numpy.dot(rotate, numpy.array([scaledx,scaledy]))
    return rotated
    

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
    # frame = cv2.imread('botty.jpg', 1) 

    mtx = numpy.array([[1.56035688e+03, 0.00000000e+00, 7.44074967e+02],
            [0.00000000e+00, 1.55382936e+03, 6.04020129e+02],
            [0.00000000e+00, 0.00000000e+00, 1.00000000e+00]])
    dist =  numpy.array([[ 7.27892841e-01, -8.78018308e+00, 3.35198169e-02, 3.91299576e-02, 4.48245023e+01]])

    dst = cv2.undistort(frame, mtx, dist, None, None)
    cv2.namedWindow("dst")

    scale = 0
    rotate = []
    foundbase = False
    origin = []
     
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


            origin =  pixelToReal(clicked[0][1], clicked[2][0], scale, rotate)
            origin = numpy.array([origin[0] * scale, origin[1]*scale])


            #circle our origin
            cv2.circle(dst, (clicked[0][1], clicked[2][0]), 3, (0,255,0), 5)


            #Save image and config
            filename = "dots" + ".jpg"
            cv2.imwrite(filename, dst)
            saveConfig(origin, scale, rotate)
            break

        # if len(clicked) == 3 and len(pixels) == 1 and foundbase == False:
        #     origin = pixelToReal(pixels[0][0], pixels[0][1], scale, rotate)
        #     cv2.circle(dst, (pixels[0][0], pixels[0][1]), 3, (0,255,0), 5)
        #     print(origin[0], origin[1])
        #     filename = "dots" + ".jpg"
        #     cv2.imwrite(filename, dst)
        #     saveConfig(origin, scale, rotate)
        #     break

        

    vid.release() 
    # Destroy all the windows 
    cv2.destroyAllWindows() 

getImg()



