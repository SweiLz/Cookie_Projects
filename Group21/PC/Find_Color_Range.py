import cv2
import numpy as np

def nothing(x):
    pass

cv2.namedWindow('Color Range',cv2.WINDOW_NORMAL)
cv2.createTrackbar('H_Min','Color Range',0,255,nothing)
cv2.createTrackbar('S_Min','Color Range',81,255,nothing)
cv2.createTrackbar('V_Min','Color Range',0,255,nothing)
cv2.createTrackbar('H_Max','Color Range',255,255,nothing)
cv2.createTrackbar('S_Max','Color Range',104,255,nothing)
cv2.createTrackbar('V_Max','Color Range',247,255,nothing)

# img = cv2.imread(img_names[2])
# img = cv2.resize(img,None,fx=0.3, fy=0.3, interpolation = cv2.INTER_CUBIC)
# img_hsv = cv2.cvtColor(img, cv2.COLOR_BGR2HSV)
# img_hsv = cv2.GaussianBlur(img_hsv,(15,15),0)
cap = cv2.VideoCapture(1)
cap.set(3,1280)
cap.set(4,720)
count=0
while True:
    ret, frame = cap.read()
    height = np.size(frame, 0)
    width = np.size(frame, 1)
    # print(height,width)
    img_hsv = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)
    # img_hsv = cv2.GaussianBlur(img_hsv, (5, 5), 0)

    H_Min = cv2.getTrackbarPos('H_Min', 'Color Range')
    S_Min = cv2.getTrackbarPos('S_Min', 'Color Range')
    V_Min = cv2.getTrackbarPos('V_Min', 'Color Range')
    H_Max = cv2.getTrackbarPos('H_Max', 'Color Range')
    S_Max = cv2.getTrackbarPos('S_Max', 'Color Range')
    V_Max = cv2.getTrackbarPos('V_Max', 'Color Range')
    str_command = '('+'('+str(H_Min)+','+str(S_Min)+','+str(V_Min)+')'+','+'('+str(H_Max)+','+str(S_Max)+','+str(V_Max)+')'+')'
    print(str_command)
    mask = cv2.inRange(img_hsv, (H_Min, S_Min, V_Min), (H_Max, S_Max, V_Max))
    # mask = cv2.erode(mask, None, iterations=2)
    # mask = cv2.dilate(mask, None, iterations=3)
    # mask = cv2.erode(mask, cv2.getStructuringElement(cv2.MORPH_ELLIPSE, (5, 5)), iterations=2)
    # mask = cv2.dilate(mask, cv2.getStructuringElement(cv2.MORPH_ELLIPSE, (5, 5)), iterations=2)
    # mask = cv2.dilate(mask, cv2.getStructuringElement(cv2.MORPH_ELLIPSE, (5, 5)), iterations=2)
    # mask = cv2.erode(mask, cv2.getStructuringElement(cv2.MORPH_ELLIPSE, (5, 5)), iterations=2)
    kernel = np.ones((5, 5), np.uint8)
    opening = cv2.morphologyEx(mask, cv2.MORPH_OPEN, kernel)
    closing = cv2.morphologyEx(opening, cv2.MORPH_CLOSE, kernel)
    contours, hierarchy = cv2.findContours(closing.copy(), cv2.RETR_TREE, cv2.CHAIN_APPROX_NONE)[1:]
    for cnt in contours:
        cv2.drawContours(frame, cnt, -1, (0, 0, 255), 2)
    cv2.namedWindow('Output Image', cv2.WINDOW_NORMAL)
    cv2.imshow('Mask',mask)
    cv2.imshow('Output Image', frame)


    key = cv2.waitKey(1) & 0xFF
    if key == 27:
        break
    if  key == ord('s'):
        name = "frame%d.jpg" % count
        cv2.imwrite(name, frame)  # save frame as JPEG file
        print(name)
        count += 1

cv2.destroyAllWindows()