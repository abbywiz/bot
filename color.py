import cv2

def on_mouse_click(event, x, y, flags, param):
    if event == cv2.EVENT_LBUTTONDOWN:
        bgr_color = img[y, x]  # OpenCV uses BGR color format
        rgb_color = tuple(reversed(bgr_color))  # Convert to RGB
        print("Color at ({},{}): {}".format(x, y, rgb_color))

# Read an image
img = cv2.imread("pic.jpg")

# Display the image
cv2.imshow("Image", img)

# Set mouse callback function
cv2.setMouseCallback("Image", on_mouse_click)

# Wait for a key press and then close the window
cv2.waitKey(0)
cv2.destroyAllWindows()
