# import the opencv library 
import cv2 
import apriltag
import numpy
import math
import json
import colorsys

pixels = []
clicked = []
rotate = []
scale = 0
deviation = .0019


def clamp(n, min, max): 
    if n < min: 
        return min
    elif n > max: 
        return max
    else: 
        return n 

def rgb_to_hsv(color): 
  
    # R, G, B values are divided by 255 
    # to change the range from 0..255 to 0..1: 
    r = color[0]
    g = color[1]
    b = color[2] 

    h,s,v = colorsys.rgb_to_hsv(r, g, b)
    return [h, s, v]
    

# returns pixel values of box

def findColor(color, image):

    hsv = cv2.cvtColor(image, cv2.COLOR_BGR2HSV)

    # lower_bound = numpy.array([max(0, color[0]-deviation), color[1], color[2]])
    # upper_bound = numpy.array([min(179, color[0] + deviation), color[1], color[2]])
    # for c in color:
    #     lower_bound.append(clamp((c-deviation), 0, 255))
    #     upper_bound.append(clamp((c+deviation), 0, 255))
        # lower_bound.append(c)
        # upper_bound.append(c)
    # lower_bound = rgb_to_hsv(lower_bound)
    # upper_bound = rgb_to_hsv(upper_bound)



    lower_bound = [max(0.0, color[0] - deviation * 100.0), max(0.0, color[1] - deviation * 100.0), max(0.0, color[2] - deviation * 360.0)]
    upper_bound = [min(179.0, color[0] + deviation * 100.0), min(255.0, color[1] + deviation * 100.0), min(255.0, color[2] + deviation * 360.0)]


    print("Color Range HSV: ", str(lower_bound), str(upper_bound))

    mask = cv2.inRange(hsv, numpy.array(lower_bound), numpy.array(upper_bound))
    contours, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
    
    if len(contours) == 0:
        print("No color in range found! Range: ", str(lower_bound), str(upper_bound))

    boxes = []
    for c in contours:
        x,y,w,h = cv2.boundingRect(c)
        image = cv2.rectangle(image, (x,y), (x+w, y+h), (0,0,255), 2)
        cv2.imshow("dst", image)
        corners = numpy.array([(x,y), (x+w,y), (x+w, y+h), (x, y+h)],dtype=numpy.int32)
        centerx = (int(corners[0][0]) + int(corners[1][0]) + int(corners[2][0]) + int(corners[3][0])) / 4
        centery = (int(corners[0][1]) + int(corners[1][1]) + int(corners[2][1]) + int(corners[3][1])) / 4
        boxes.append([centerx,centery])
    return boxes
    


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

        clicked.append([x,y])

    if event == cv2.EVENT_RBUTTONDOWN:
        pixels.append([x,y])



def pixelToBase(rotate, scale, origin, x, y):
    x = x - origin[0]
    y = y - origin[1]
    print("Transformed x and y: ", x, y)

    rotated = rotate.T @ (numpy.array([[x,y]]).T)
    print("Rotated: ", rotated)
    print("Rotation Matrix: ", rotate)
    return  rotated * scale
    

def getAngle(pointA, pointB, pointC):
    pointZ = (pointB[0], pointA[1])
    deltaB = (pointB[1] - pointZ[1])
    deltaA = (pointA[0] - pointZ[0])
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

    # 5.78 inches is 0.146812
    #width
    scaleW = 0.146812 / width[0]

    # is 0.104775 meters
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

    # ret, frame = vid.read() 
    frame = cv2.imread('pic.jpg', 1) 

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

        color = (0,.2418,.9569)
        findColor(color, dst)
        filename = "dots" + ".jpg"
        cv2.imwrite(filename, dst)


        # if len(clicked) == 3 and scale == 0:
        #     # So height O width
        #     #click in order of A,B,C
        #     #cliked[0] is A
        #     #clicked[1] is B
        #     #clicked[2] is C
        #     print(clicked[0][0])
        #     A = numpy.array(
        #         [
        #             [clicked[0][0]], 
        #             [clicked[0][1]], 
        #             [0]
        #         ]
        #         )

        #     B = numpy.array(
        #         [
        #             [clicked[1][0]], 
        #             [clicked[1][1]], 
        #             [0]
        #         ]
        #     )    
        #     C = numpy.array(
        #         [
        #             [clicked[2][0]], 
        #             [clicked[2][1]], 
        #             [0]
        #         ]
        #     )   



        #     #Get scale and rotation matrix
        #     scale = getScale(clicked[0], clicked[1], clicked[2])
        #     rotate = getRotationMatrix(clicked[0], clicked[1], clicked[2])

        #     #Calculate the oritin
        #     Px = B - A
        #     NormalPx = Px / numpy.linalg.norm(Px)
        #     Pz = numpy.array([
        #                     [0],
        #                     [0],
        #                     [1]])

        #     Px3D = numpy.array([Px[0], Px[1], [0]])

        #     Py = numpy.cross(Px3D.reshape(-1), Pz.reshape(-1))
        #     NormalPy = Py / numpy.linalg.norm(Py)
        #     bc = numpy.linalg.norm(B - C)

        #     pybc =  NormalPy * numpy.linalg.norm(B - C)


        #     originRinI = NormalPy * numpy.linalg.norm(B - C) + A.reshape(-1)
        #     origin = originRinI[:-1]


        #     #circle our origin
        #     cv2.circle(dst, (int(origin[0]), int(origin[1])), 3, (0,255,0), 5)
        #     #Save image and config
        #     filename = "dots" + ".jpg"
        #     cv2.imwrite(filename, dst)
        #     saveConfig(origin, scale, rotate)


        # # Run this to click a point
        # if len(clicked) == 3 and len(pixels) == 1 and foundbase == False:
        #     #Pink Block color is 77,82,211
        #     upper = (80,85,214)
        #     lower = (74,79,208)
        #     colorRange = [lower, upper]
        #     findColor(colorRange, dst)
        #     # print("Clicked here", pixels[0][0], pixels[0][1])
        #     # cv2.circle(dst, (pixels[0][0], pixels[0][1],), 3, (0,255,0), 5)
        #     # points = pixelToBase(rotate, scale, origin, pixels[0][0], pixels[0][1])
        #     # print("WRT the base", points[0], points[1])

        #     filename = "dots" + ".jpg"
        #     cv2.imwrite(filename, dst)
        #     break

        

    vid.release() 
    # Destroy all the windows 
    cv2.destroyAllWindows() 

getImg()





