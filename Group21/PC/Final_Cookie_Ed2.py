'''

# Member of Group
# 1.Woravee Chanlongrat        57340500065 FIBO sec B
# 2.Pitchaporn Rewatbowornwong 57340500051 FIBO sec B
# 3.Wuttipat Chokanantasab     57340500067 FIBO sec B

from shapedetector import ShapeDetector
from findrealdistance import Mapping
import imutils
import cv2
import math
import serial
import numpy as np

def convert_Pulse(milimeter):
    return  (768*milimeter)/(2*math.pi*6.175)

current_position = [1103,283]
goal = [(807,502),(807,405),(807,290),(807,163)]
Zshape = [0,0,0,0]
Openshape = [int(22*20000/180),int(30*20000/180),int(40*20000/180),int(30*20000/180)]
Shapei = []#rec tri sq cir

Color_text = (0,0,0)
px1 = 390
px2 = 889
py1 = 96
py2 = 593
Af_pts1 = [[px1,py1],[px2,py1],[px1,py2],[px2,py2]]
pts1 = np.float32(Af_pts1)
pts2 = np.float32([[0,0],[900,0],[0,900],[900,900]])

red = ((0,45,138),(211,150,206))#((0,50,145),(255,125,210))
yellow = ((20,45,211),(55,147,255)) #((0,108,239),(34,128,255))
green = ((0,0,0),(255,255,180)) #((84,72,84),(119,153,178))
blue = ((67,152,80),(189,221,255))
purple =((77,89,145),(210,171,255))
pink = ((114,48,216),(139,89,249))
object_color = (red,yellow,green,blue,purple,pink)

super_angle = []
tri_twice = []
counter = [0,0,0,0,0,0,0]#all tri sq pen cir rec etc
image = cv2.imread('frame0.JPG')
M = cv2.getPerspectiveTransform(pts1,pts2)
image = cv2.warpPerspective(image,M,(900,900))
# image = cv2.resize(image, None, fx=0.3, fy=0.3, interpolation=cv2.INTER_CUBIC)
cX = []
cY = []
data_color = []
color_sequence = ['red', 'yellow', 'green', 'blue', 'purple','pink']
index= []
shape = []
count_color = 0
ii = None
i=0
obj_num = 0
img_hsv = cv2.cvtColor(image.copy(), cv2.COLOR_BGR2HSV)
for color in object_color:
    mask = cv2.inRange(img_hsv, color[0], color[1])
    # mask = cv2.erode(mask, None, iterations=2)
    # mask = cv2.dilate(mask, None, iterations=3)
    mask = cv2.erode(mask, cv2.getStructuringElement(cv2.MORPH_ELLIPSE, (5,5)), iterations=4)
    mask = cv2.dilate(mask, cv2.getStructuringElement(cv2.MORPH_ELLIPSE, (5,5)), iterations=4)

    mask = cv2.dilate(mask, cv2.getStructuringElement(cv2.MORPH_ELLIPSE, (5,5)), iterations=4)
    mask = cv2.erode(mask, cv2.getStructuringElement(cv2.MORPH_ELLIPSE, (5,5)), iterations=4)


    # blurred = cv2.GaussianBlur(mask, (5,5), 0)
    # edges = cv2.Canny(mask,700,1000)
    __,contours, hierarchy = cv2.findContours(mask.copy(), cv2.RETR_TREE,cv2.CHAIN_APPROX_SIMPLE)
    i=i+1
    # cv2.namedWindow('Output Mark', cv2.WINDOW_AUTOSIZE)
    cv2.imshow('Output Mark'+str(i),mask)
    sd = ShapeDetector()
    obj_num = len(contours)

    angle = []

    for c in contours:
        area = cv2.contourArea(c)
        print(area)
        if area>3000:
            data_color.append(color_sequence[count_color])
            counter[0]+= 1

            M = cv2.moments(c)
            # cX.append(int((M["m10"] / M["m00"])))
            # cY.append(int((M["m01"] / M["m00"])))
            if(M['m00'])!=0:
                cX.append(int((M["m10"] / M["m00"])))
                cY.append(int((M["m01"] / M["m00"])))
            else:
                cX.append(int((M["m10"] / 1)))
                cY.append(int((M["m01"] / 1)))
            shape.append(sd.detect(c))
            epsilon = 0.04 * cv2.arcLength(contours[0], True)
            approx = cv2.approxPolyDP(contours[0], epsilon, True)

            # Draw approx
            for i in approx:
                cv2.circle(image, (i[0, 0], i[0, 1]), 3, (0,255,255), -1)
            # cv2.drawContours(image, contours, -1, (0, 255, 0), 3)

            if shape[-1] == "triangle" :
                counter[1]+=1
                # cv2.drawContours(image, [c], -1, (51,153,255), -1)
                p1 = [approx[0][0][0], approx[0][0][1]]
                p2 = [approx[1][0][0], approx[1][0][1]]
                p3 = [approx[2][0][0], approx[2][0][1]]
                d1 = math.pow((p1[0] - p2[0]), 2) + math.pow((p1[1] - p2[1]), 2)
                d2 = math.pow((p2[0] - p3[0]), 2) + math.pow((p2[1] - p3[1]), 2)
                d3 = math.pow((p1[0] - p3[0]), 2) + math.pow((p1[1] - p3[1]), 2)
                a = None
                if (d1 - d2 > 200 and d1 - d3 > 200):
                    a = (-math.atan2((p1[1] - p2[1]), (p1[0] - p2[0])) * 180 / math.pi)
                    P = [p3]
                    if(p2[0]<=p1[0]):
                        P += [p2,p1]
                    else:
                        P += [p1, p2]
                elif (d2 - d1 > 200 and d2 - d3 > 200):
                    a = (-math.atan2((p2[1] - p3[1]), (p2[0] - p3[0])) * 180 / math.pi)
                    P = [p1]
                    if (p2[0] <= p3[0]):
                        P += [p2, p3]
                    else:
                        P += [p3, p2]
                elif (d3 - d1 > 200 and d3 - d2 > 200):
                    a = (-math.atan2((p1[1] - p3[1]), (p1[0] - p3[0])) * 180 / math.pi)
                    P = [p2]
                    if (p3[0] <= p1[0]):
                        P += [p3, p1]
                    else:
                        P += [p1, p3]
                else: a = 9999
                if a<0:
                    a+=180
                if (P[0][0]<=P[1][0] and P[0][0]<=P[2][0]):
                    tri_twice.append(0)
                elif (P[0][0] <= P[1][0] and P[0][0] <= P[2][0]):
                    tri_twice.append(1)
                elif (P[0][1]>=P[1][1]):
                    if (P[1][1]<=P[2][1]):
                        tri_twice.append(0)
                    else: tri_twice.append(1)
                else:
                    if (P[1][1] <= P[2][1]):
                        tri_twice.append(1)
                    else: tri_twice.append(0)
                if tri_twice[-1] == 0:
                    angle.append(a)
                else:
                    angle.append(-a)

            elif shape[-1] == "square" :
                # cv2.drawContours(image, [c], -1, (128,128,128), -1)
                p1 = [approx[0][0][0], approx[0][0][1]]
                p2 = [approx[1][0][0], approx[1][0][1]]
                p3 = [approx[2][0][0], approx[2][0][1]]
                d1 = math.pow((p1[0] - p2[0]), 2) + math.pow((p1[1] - p2[1]), 2)
                d2 = math.pow((p2[0] - p3[0]), 2) + math.pow((p2[1] - p3[1]), 2)
                # print([d1,d2])
                # if abs(d1 - d2) > 6000:
                #     print('c')
                #     if d1 > d2:
                #         a = (-math.atan2((p1[1] - p2[1]), (p1[0] - p2[0])) * 180 / math.pi)
                #     else:
                #         a = (-math.atan2((p2[1] - p3[1]), (p2[0] - p3[0])) * 180 / math.pi)
                #     if (a == -180):
                #         a = 0
                #     shape[-1] = "rectangle"
                #     counter[5] += 1
                # else:
                a = (-math.atan2((p1[1] - p2[1]), (p1[0] - p2[0])) * 180 / math.pi)
                if a >= 90:
                    a -= 90
                counter[2] += 1
                angle.append(a)
            elif shape[-1] == "pentagon":
                shape[-1] = "circle"
                counter[4] += 1
                angle.append(0)
                # cv2.drawContours(image, [c], -1, (153,153,255), -1)
            elif shape[-1] == "circle":
                counter[4]+=1
                angle.append(0)
                # cv2.drawContours(image, [c], -1, (76,153, 0), -1)
            elif shape[-1] == "rectangle":
                # cv2.drawContours(image, [c], -1, (255, 125, 0), -1)
                p1 = [approx[0][0][0], approx[0][0][1]]
                p2 = [approx[1][0][0], approx[1][0][1]]
                p3 = [approx[2][0][0], approx[2][0][1]]
                d1 = math.pow((p1[0] - p2[0]), 2) + math.pow((p1[1] - p2[1]), 2)
                d2 = math.pow((p2[0] - p3[0]), 2) + math.pow((p2[1] - p3[1]), 2)
                a = None
                if abs(d1 - d2) < 6000:
                    a = (-math.atan2((p1[1] - p2[1]), (p1[0] - p2[0])) * 180 / math.pi)
                    if a >= 90:
                        a -= 90
                    shape[-1] = "square"
                    counter[2] += 1
                else:
                    if d1 > d2:
                        a = (-math.atan2((p1[1] - p2[1]), (p1[0] - p2[0])) * 180 / math.pi)
                    else:
                        a = (-math.atan2((p2[1] - p3[1]), (p2[0] - p3[0])) * 180 / math.pi)
                    if (a == -180 or a==-0):
                        a = 0
                counter[5] += 1
                angle.append(a)
            else :
                counter[6]+=1
                # cv2.drawContours(image, [c], -1, (255,102,178), -1)
                angle.append(None)
            c = c.astype("float")
            c = c.astype("int")
            cv2.putText(image, shape[-1], (cX[-1]+10, cY[-1]), cv2.FONT_HERSHEY_SIMPLEX,0.5, Color_text, 1)
            cv2.circle(image, (cX[-1], cY[-1]), 7, (255, 255, 255), -1)
            center = "("+str(cX[-1])+","+str(cY[-1])+")"
            cv2.putText(image,center,(cX[-1] - 20, cY[-1] - 20), cv2.FONT_HERSHEY_SIMPLEX, 0.5, Color_text, 1)
            super_angle += angle
    count_color+=1
index = [None] * len(super_angle)
ii = 1
sequence = []
print(data_color)
if counter[5] > 0:
    rec = [index for index, value in enumerate(shape) if value == "rectangle"]
    for cl in color_sequence:
        for i in rec:
            if data_color[i] == cl:
                sequence.append(i)
                Shapei.append(0)
                index[i] = ii
                ii += 1
    del rec
if counter[1] > 0:
    tri = [index for index, value in enumerate(shape) if value == "triangle"]
    for cl in color_sequence:
        for i in tri:
            if data_color[i] == cl:
                sequence.append(i)
                Shapei.append(1)
                index[i] = ii
                ii += 1
    del tri
if counter[2] > 0:
    sq = [index for index, value in enumerate(shape) if value == "square"]
    for cl in color_sequence:
        for i in sq:
            if data_color[i] == cl:
                sequence.append(i)
                Shapei.append(2)
                index[i] = ii
                ii += 1
    del sq
if counter[4] > 0:
    cir = [index for index, value in enumerate(shape) if value == "circle"]
    for cl in color_sequence:
        for i in cir:
            if data_color[i] == cl:
                Shapei.append(3)
                sequence.append(i)
                index[i] = ii
                ii += 1
    del cir
for i in range(len(super_angle)):
    cv2.putText(image, str(super_angle[i]), (cX[i], cY[i] + 20), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 0, 0), 1)
    cv2.putText(image, str(index[i]), (cX[i] - 20, cY[i] + 20), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (150, 0, 0), 1)
    #print(len(super_angle))
print(sequence)
print(index)
y = 20
total = "Total of  Shape : " + str(counter[0])
cv2.putText(image,total,(30,30), cv2.FONT_HERSHEY_SIMPLEX,0.5, (0,0, 0), 1)
triangle = "Triangle  Shape : "+str(counter[1])
cv2.putText(image,triangle,(30,30+y), cv2.FONT_HERSHEY_SIMPLEX,0.5, (0,0, 0), 1)
rectangle = "Rectangle Shape : "+str(counter[5])
cv2.putText(image,rectangle,(30,30+(2*y)), cv2.FONT_HERSHEY_SIMPLEX,0.5, (0,0, 0), 1)
square  =  "Square Shape : "+ str(counter[2])
cv2.putText(image,square,(30,30+(3*y)), cv2.FONT_HERSHEY_SIMPLEX,0.5, (0,0, 0), 1)
pentagon = "Pentagon  Shape : "+str(counter[3])
cv2.putText(image,pentagon,(30,30+(4*y)), cv2.FONT_HERSHEY_SIMPLEX,0.5, (0,0, 0), 1)
circle = "Circle    Shape : "+str(counter[4])
cv2.putText(image,circle,(30,30+(5*y)), cv2.FONT_HERSHEY_SIMPLEX,0.5, (0,0, 0), 1)
undefine = "Unidentified Shape : "+str(counter[6])
cv2.putText(image,undefine,(30,30+(6*y)), cv2.FONT_HERSHEY_SIMPLEX,0.5, (0,0, 0), 1)


cv2.namedWindow('Image', cv2.WINDOW_FREERATIO)
cv2.imshow("Image",image)
cv2.waitKey(0)
#
# ser = serial.Serial('COM2', 9600, timeout=0)
# sent_data = True
# j = 0
# half_round = 0
# D = Mapping()
# if ser.is_open:
#       print('serial is open')
#       while True:
#             if ser.inWaiting() > 0:
#                 x = ser.read(1)
#                 if x == str.encode('f'):
#                     print('finished!')
#                     sent_data = True
#             if(j<obj_num):
#                 if sent_data:
#                     # update parameters here x y z s1 s2
#                     if half_round==0:
#                         current_position = [cX[j]+px1, cY[j]+py1]
#                         target = M.to3D(current_position)
#                         str_command = '<'
#                         str_command += str(convert_Pulse(target[1])) + ','
#                         str_command += str(convert_Pulse(-target[0])) + ','
#                         str_command += str(convert_Pulse(Zshape[Shapei[j]])) + ','
#                         str_command += str(Openshape[Shapei[j]]) + ','
#                         str_command += str(int(angle[j]*20000/180)) + '>'
#                         ser.write(str.encode(str_command))
#                         sent_data = False
#                         half_round = 1
#                     else:
#                         go = D.findDistance(current_position, [goal[Shapei[j]][0],goal[Shapei[j]][1]])
#                         current_position = [goal[Shapei[j]][0],goal[Shapei[j]][1]]
#                         target = M.to3D(current_position)
#                         str_command = '<'
#                         str_command += str(convert_Pulse(target[1])) + ','
#                         str_command += str(convert_Pulse(-target[0])) + ','
#                         str_command += str(convert_Pulse(Zshape[Shapei[j]])) + ','
#                         str_command += str(int(90*20000/180)) + ','
#                         str_command += str(0) + '>'
#                         ser.write(str.encode(str_command))
#                         sent_data = False
#                         half_round = 0
#                         j += 1
#             else: break
#

cv2.destroyAllWindows()
'''
'''

# Member of Group
# 1.Woravee Chanlongrat        57340500065 FIBO sec B
# 2.Pitchaporn Rewatbowornwong 57340500051 FIBO sec B
# 3.Wuttipat Chokanantasab     57340500067 FIBO sec B

from shapedetector import ShapeDetector
import imutils
import cv2
import math
import serial
import numpy as np

cameramat = [[ 463.9221989 ,    0.        ,  321.32220552],
             [   0.        ,  447.72313044,  251.97148995],
             [   0.        ,    0.        ,    1.        ]]
cm = np.linalg.pinv(cameramat)

def Mapto3D(p):
    p+=[1]
    return np.dot(cm, p)*Z

def convert_Pulse(milimeter):
    return  (768*milimeter)/(2*math.pi*6.175)

setzero = [1799,-75]#[1164,-143]
goal = [(807,502),(807,405),(807,290),(807,163)]
Zshape = [0,0,0,0]
Openshape = [220,300,400,300]
Shapei = []#rec tri sq cir

Color_text = (0,0,0)
px1 = 390
px2 = 889
py1 = 96
py2 = 593
Af_pts1 = [[px1,py1],[px2,py1],[px1,py2],[px2,py2]]
pts1 = np.float32(Af_pts1)
pts2 = np.float32([[0,0],[900,0],[0,900],[900,900]])
red = ((0,0,103),(231,255,144))#((0,50,145),(255,125,210))
yellow = ((0,127,144),(85,210,164)) #((0,108,239),(34,128,255))
green = ((63,190,0),(128,255,123))#((84,72,84),(119,153,178))
blue = ((0,237,32),(253,255,162))#((67,152,80),(189,221,255))
pink = ((113,6,154),(136,255,174))
purple =((97,160,142),(107,239,176))#((115,47,178),(255,134,221))
# red = ((0,45,138),(211,150,206))#((0,50,145),(255,125,210))
# yellow = ((20,45,211),(55,147,255)) #((0,108,239),(34,128,255))
# green = ((0,0,0),(255,255,180)) #((84,72,84),(119,153,178))
# blue = ((67,152,80),(189,221,255))
# purple =((77,89,145),(210,171,255))
# pink = ((114,48,216),(139,89,249))
object_color = (red,yellow,green,blue,purple,pink)

super_angle = []
tri_twice = []
counter = [0,0,0,0,0,0,0]#all tri sq pen cir rec etc
image = cv2.imread('frame0.JPG')
M = cv2.getPerspectiveTransform(pts1,pts2)
image = cv2.warpPerspective(image,M,(900,900))
# image = cv2.resize(image, None, fx=0.3, fy=0.3, interpolation=cv2.INTER_CUBIC)
cX = []
cY = []
data_color = []
color_sequence = ['red', 'yellow', 'green', 'blue', 'purple','pink']
index= []
shape = []
count_color = 0
ii = None
i=0
obj_num = 0
img_hsv = cv2.cvtColor(image.copy(), cv2.COLOR_BGR2HSV)
for color in object_color:
    mask = cv2.inRange(img_hsv, color[0], color[1])
    mask = cv2.erode(mask, None, iterations=2)
    mask = cv2.dilate(mask, None, iterations=3)
    # mask = cv2.erode(mask, cv2.getStructuringElement(cv2.MORPH_ELLIPSE, (5,5)), iterations=4)
    # mask = cv2.dilate(mask, cv2.getStructuringElement(cv2.MORPH_ELLIPSE, (5,5)), iterations=4)
    #
    # mask = cv2.dilate(mask, cv2.getStructuringElement(cv2.MORPH_ELLIPSE, (5,5)), iterations=4)
    # mask = cv2.erode(mask, cv2.getStructuringElement(cv2.MORPH_ELLIPSE, (5,5)), iterations=4)


    # blurred = cv2.GaussianBlur(mask, (5,5), 0)
    # edges = cv2.Canny(mask,700,1000)
    __,contours, hierarchy = cv2.findContours(mask.copy(), cv2.RETR_TREE,cv2.CHAIN_APPROX_SIMPLE)
    i=i+1
    # cv2.namedWindow('Output Mark', cv2.WINDOW_AUTOSIZE)
    cv2.imshow('Output Mark'+str(i),mask)
    sd = ShapeDetector()
    obj_num = len(contours)

    angle = []

    for c in contours:
        area = cv2.contourArea(c)
        print(area)
        if area>3000:
            data_color.append(color_sequence[count_color])
            counter[0]+= 1

            M = cv2.moments(c)
            # cX.append(int((M["m10"] / M["m00"])))
            # cY.append(int((M["m01"] / M["m00"])))
            if(M['m00'])!=0:
                cX.append(int((M["m10"] / M["m00"])))
                cY.append(int((M["m01"] / M["m00"])))
            else:
                cX.append(int((M["m10"] / 1)))
                cY.append(int((M["m01"] / 1)))
            shape.append(sd.detect(c))
            epsilon = 0.04 * cv2.arcLength(contours[0], True)
            approx = cv2.approxPolyDP(contours[0], epsilon, True)

            # Draw approx
            for i in approx:
                cv2.circle(image, (i[0, 0], i[0, 1]), 3, (0,255,255), -1)
            # cv2.drawContours(image, contours, -1, (0, 255, 0), 3)

            if shape[-1] == "triangle" :
                counter[1]+=1
                # cv2.drawContours(image, [c], -1, (51,153,255), -1)
                p1 = [approx[0][0][0], approx[0][0][1]]
                p2 = [approx[1][0][0], approx[1][0][1]]
                p3 = [approx[2][0][0], approx[2][0][1]]
                d1 = math.pow((p1[0] - p2[0]), 2) + math.pow((p1[1] - p2[1]), 2)
                d2 = math.pow((p2[0] - p3[0]), 2) + math.pow((p2[1] - p3[1]), 2)
                d3 = math.pow((p1[0] - p3[0]), 2) + math.pow((p1[1] - p3[1]), 2)
                a = None
                if (d1 - d2 > 200 and d1 - d3 > 200):
                    a = (-math.atan2((p1[1] - p2[1]), (p1[0] - p2[0])) * 180 / math.pi)
                    P = [p3]
                    if(p2[0]<=p1[0]):
                        P += [p2,p1]
                    else:
                        P += [p1, p2]
                elif (d2 - d1 > 200 and d2 - d3 > 200):
                    a = (-math.atan2((p2[1] - p3[1]), (p2[0] - p3[0])) * 180 / math.pi)
                    P = [p1]
                    if (p2[0] <= p3[0]):
                        P += [p2, p3]
                    else:
                        P += [p3, p2]
                elif (d3 - d1 > 200 and d3 - d2 > 200):
                    a = (-math.atan2((p1[1] - p3[1]), (p1[0] - p3[0])) * 180 / math.pi)
                    P = [p2]
                    if (p3[0] <= p1[0]):
                        P += [p3, p1]
                    else:
                        P += [p1, p3]
                else: a = 9999
                if a<0:
                    a+=180
                if (P[0][0]<=P[1][0] and P[0][0]<=P[2][0]):
                    tri_twice.append(0)
                elif (P[0][0] <= P[1][0] and P[0][0] <= P[2][0]):
                    tri_twice.append(1)
                elif (P[0][1]>=P[1][1]):
                    if (P[1][1]<=P[2][1]):
                        tri_twice.append(0)
                    else: tri_twice.append(1)
                else:
                    if (P[1][1] <= P[2][1]):
                        tri_twice.append(1)
                    else: tri_twice.append(0)
                angle.append(a)

            elif shape[-1] == "square" :
                # cv2.drawContours(image, [c], -1, (128,128,128), -1)
                p1 = [approx[0][0][0], approx[0][0][1]]
                p2 = [approx[1][0][0], approx[1][0][1]]
                p3 = [approx[2][0][0], approx[2][0][1]]
                d1 = math.pow((p1[0] - p2[0]), 2) + math.pow((p1[1] - p2[1]), 2)
                d2 = math.pow((p2[0] - p3[0]), 2) + math.pow((p2[1] - p3[1]), 2)
                # print([d1,d2])
                # if abs(d1 - d2) > 6000:
                #     print('c')
                #     if d1 > d2:
                #         a = (-math.atan2((p1[1] - p2[1]), (p1[0] - p2[0])) * 180 / math.pi)
                #     else:
                #         a = (-math.atan2((p2[1] - p3[1]), (p2[0] - p3[0])) * 180 / math.pi)
                #     if (a == -180):
                #         a = 0
                #     shape[-1] = "rectangle"
                #     counter[5] += 1
                # else:
                a = (-math.atan2((p1[1] - p2[1]), (p1[0] - p2[0])) * 180 / math.pi)
                if a >= 90:
                    a -= 90
                counter[2] += 1
                angle.append(a)
            elif shape[-1] == "pentagon":
                shape[-1] = "circle"
                counter[4] += 1
                angle.append(0)
                # cv2.drawContours(image, [c], -1, (153,153,255), -1)
            elif shape[-1] == "circle":
                counter[4]+=1
                angle.append(0)
                # cv2.drawContours(image, [c], -1, (76,153, 0), -1)
            elif shape[-1] == "rectangle":
                # cv2.drawContours(image, [c], -1, (255, 125, 0), -1)
                p1 = [approx[0][0][0], approx[0][0][1]]
                p2 = [approx[1][0][0], approx[1][0][1]]
                p3 = [approx[2][0][0], approx[2][0][1]]
                d1 = math.pow((p1[0] - p2[0]), 2) + math.pow((p1[1] - p2[1]), 2)
                d2 = math.pow((p2[0] - p3[0]), 2) + math.pow((p2[1] - p3[1]), 2)
                a = None
                if abs(d1 - d2) < 6000:
                    a = (-math.atan2((p1[1] - p2[1]), (p1[0] - p2[0])) * 180 / math.pi)
                    if a >= 90:
                        a -= 90
                    shape[-1] = "square"
                    counter[2] += 1
                else:
                    if d1 > d2:
                        a = (-math.atan2((p1[1] - p2[1]), (p1[0] - p2[0])) * 180 / math.pi)
                    else:
                        a = (-math.atan2((p2[1] - p3[1]), (p2[0] - p3[0])) * 180 / math.pi)
                    if (a == -180 or a==-0):
                        a = 0
                counter[5] += 1
                angle.append(a)
            else :
                counter[6]+=1
                # cv2.drawContours(image, [c], -1, (255,102,178), -1)
                angle.append(None)
            c = c.astype("float")
            c = c.astype("int")
            cv2.putText(image, shape[-1], (cX[-1]+10, cY[-1]), cv2.FONT_HERSHEY_SIMPLEX,0.5, Color_text, 1)
            cv2.circle(image, (cX[-1], cY[-1]), 7, (255, 255, 255), -1)
            center = "("+str(cX[-1])+","+str(cY[-1])+")"
            cv2.putText(image,center,(cX[-1] - 20, cY[-1] - 20), cv2.FONT_HERSHEY_SIMPLEX, 0.5, Color_text, 1)
            super_angle += angle
    count_color+=1
index = [None] * len(super_angle)
ii = 1
sequence = []
print(data_color)
if counter[5] > 0:
    rec = [index for index, value in enumerate(shape) if value == "rectangle"]
    for cl in color_sequence:
        for i in rec:
            if data_color[i] == cl:
                sequence.append(i)
                Shapei.append(0)
                index[i] = ii
                ii += 1
    del rec
if counter[1] > 0:
    tri = [index for index, value in enumerate(shape) if value == "triangle"]
    for cl in color_sequence:
        for i in tri:
            if data_color[i] == cl:
                sequence.append(i)
                Shapei.append(1)
                index[i] = ii
                ii += 1
    del tri
if counter[2] > 0:
    sq = [index for index, value in enumerate(shape) if value == "square"]
    for cl in color_sequence:
        for i in sq:
            if data_color[i] == cl:
                sequence.append(i)
                Shapei.append(2)
                index[i] = ii
                ii += 1
    del sq
if counter[4] > 0:
    cir = [index for index, value in enumerate(shape) if value == "circle"]
    for cl in color_sequence:
        for i in cir:
            if data_color[i] == cl:
                Shapei.append(3)
                sequence.append(i)
                index[i] = ii
                ii += 1
    del cir
# for i in range(len(super_angle)):
#     cv2.putText(image, str(super_angle[i]), (cX[i], cY[i] + 20), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 0, 0), 1)
#     cv2.putText(image, str(index[i]), (cX[i] - 20, cY[i] + 20), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (150, 0, 0), 1)
    #print(len(super_angle))
print(sequence)
print(index)
y = 20
total = "Total of  Shape : " + str(counter[0])
cv2.putText(image,total,(30,30), cv2.FONT_HERSHEY_SIMPLEX,0.5, (0,0, 0), 1)
triangle = "Triangle  Shape : "+str(counter[1])
cv2.putText(image,triangle,(30,30+y), cv2.FONT_HERSHEY_SIMPLEX,0.5, (0,0, 0), 1)
rectangle = "Rectangle Shape : "+str(counter[5])
cv2.putText(image,rectangle,(30,30+(2*y)), cv2.FONT_HERSHEY_SIMPLEX,0.5, (0,0, 0), 1)
square  =  "Square Shape : "+ str(counter[2])
cv2.putText(image,square,(30,30+(3*y)), cv2.FONT_HERSHEY_SIMPLEX,0.5, (0,0, 0), 1)
pentagon = "Pentagon  Shape : "+str(counter[3])
cv2.putText(image,pentagon,(30,30+(4*y)), cv2.FONT_HERSHEY_SIMPLEX,0.5, (0,0, 0), 1)
circle = "Circle    Shape : "+str(counter[4])
cv2.putText(image,circle,(30,30+(5*y)), cv2.FONT_HERSHEY_SIMPLEX,0.5, (0,0, 0), 1)
undefine = "Unidentified Shape : "+str(counter[6])
cv2.putText(image,undefine,(30,30+(6*y)), cv2.FONT_HERSHEY_SIMPLEX,0.5, (0,0, 0), 1)


cv2.namedWindow('Image', cv2.WINDOW_FREERATIO)
cv2.imshow("Image",image)
cv2.waitKey(0)
#
# ser = serial.Serial('COM2', 9600, timeout=0)
# sent_data = True
# j = 0
# half_round = 0
# if ser.is_open:
#       print('serial is open')
#       while True:
#             if ser.inWaiting() > 0:
#                 x = ser.read(1)
#                 if x == str.encode('f'):
#                     print('finished!')
#                     sent_data = True
#             if(j<obj_num):
#                 if sent_data:
#                     # update parameters here x y z s1 s2
#                     if half_round==0:
#                         current_position = [cX[j]+px1+setzero[0], cY[j]+py1+setzero[1]]
#                         target = Mapto3D(current_position)
#                         str_command = '<'
#                         str_command += str(int(target[1]*10)) + ','
#                         str_command += str(int(target[0]*10)) + ','
#                         str_command += str(int(Zshape[Shapei[j]])) + ','
#                         str_command += str(Openshape[Shapei[j]]) + ','
#                         str_command += str(int(angle[j]*10) + '>'
#                         ser.write(str.encode(str_command))
#                         sent_data = False
#                         half_round = 1
#                     else:
#                         go = D.findDistance(current_position, [goal[Shapei[j]][0],goal[Shapei[j]][1]])
#                         current_position = [goal[Shapei[j]][0],goal[Shapei[j]][1]]
#                         target = Mapto3D(current_position)
#                         str_command = '<'
#                         str_command += str(int(target[1]*10)) + ','
#                         str_command += str(int(target[0]*10)) + ','
#                         str_command += str(int(Zshape[Shapei[j]])) + ','
#                         if Shapei[j]==1:
#                           t = tri_twice.pop(0)
#                           if t == 0:
#                               str_command += '1800' + ','
#                           else:
#                               str_command += '0' + ','
#                         else:
#                           str_command += '0' + ','
#                         str_command += '0' + '>'
#                         ser.write(str.encode(str_command))
#                         sent_data = False
#                         half_round = 0
#                         j += 1
#             else: break
#

cv2.destroyAllWindows()

'''

# Member of Group
# 1.Woravee Chanlongrat        57340500065 FIBO sec B
# 2.Pitchaporn Rewatbowornwong 57340500051 FIBO sec B
# 3.Wuttipat Chokanantasab     57340500067 FIBO sec B

from shapedetector import ShapeDetector
import imutils
import cv2
import math
import serial
import numpy as np
from time import sleep
cap = cv2.VideoCapture(1)
cap.set(3,1280)
cap.set(4,720)
count=0
px1 = 355
px2 = 675#855
py1 = 97
py2 = 597
goal = [(770,499+8),(770,406+8),(770,293+8),(770,166+8)]
while True:
    ret, frame = cap.read()
    cv2.line(frame, (px1,py1), (px2,py1), (0, 255, 0), 3)
    cv2.line(frame, (px1,py1), (px1,py2), (0, 255, 0), 3)
    cv2.line(frame, (px2,py1), (px2,py2), (0, 255, 0), 3)
    cv2.line(frame, (px1,py2), (px2,py2), (0, 255, 0), 3)
    cv2.circle(frame, goal[0], 4, (0, 0, 255), -1)
    cv2.circle(frame, goal[1], 4, (0, 0, 255), -1)
    cv2.circle(frame, goal[2], 4, (0, 0, 255), -1)
    cv2.circle(frame, goal[3], 4, (0, 0, 255), -1)
    cv2.imshow('Output Image', frame)
    key = cv2.waitKey(1) & 0xFF
    if key == 27:
        break
    if key == ord('s'):
        name = "frame%d.jpg" % count
        cv2.imwrite(name, frame)  # save frame as JPEG file
        print(name)
        image = frame
        count += 1
        break

setzero = [1182,-162]
Z = 625.0
cameramat = [[  1.07747337e+03,   0.00000000e+00,   6.34076240e+02],
             [  0.00000000e+00,   1.07739008e+03,   3.57274392e+02],
             [  0.00000000e+00,   0.00000000e+00,   1.00000000e+00]]
cm = np.linalg.pinv(cameramat)
def Mapto3D(p):
    r = np.array([p[0], p[1], 1])
    o = np.array([setzero[0], setzero[1], 1])
    pr1 = np.dot(cm, r) * Z
    pr0 = np.dot(cm, o)*Z
    return [pr1[0]-pr0[0],pr1[1]-pr0[1]]

#def convert_Pulse(milimeter):
#    return  (768*milimeter)/(2*math.pi*6.175)

Zshape = [720,720,760,720]
Openshape = [140,200,400,150]
#rec tri sq cir
Color_text = (0,0,0)

Af_pts1 = [[px1,py1],[px2,py1],[px1,py2],[px2,py2]]
pts1 = np.float32(Af_pts1)
pts2 = np.float32([[0,0],[320,0],[0,500],[320,500]])
red = ((0,47,183),(255,115,239))#((0,0,164),(196,105,231))
yellow = ((0,0,0),(42,255,255))#((0,51,218),(66,129,253))
green = ((0,0,31),(255,255,178))#((82,77,124),(147,204,188))#((26,84,61),(98,234,233))
blue = ((101,101,143),(108,255,255))#((0,190,12),(255,255,251))
pink = ((108,0,215),(255,98,255))#((114,31,215),(152,255,255))
purple =((107,94,0),(255,255,255))#((105,61,190),(131,222,255))
object_color = (red,yellow,green,blue,purple,pink)
super_angle = []
tri_twice = []
P = None
counter = [0,0,0,0,0,0,0]#all tri sq pen cir rec etc
# height = np.size(image, 0)
# width = np.size(image, 1)
# print(height,width)
M = cv2.getPerspectiveTransform(pts1,pts2)
image = cv2.warpPerspective(image,M,(320,500))
# height = np.size(image2, 0)
# width = np.size(image2, 1)
# print(height,width)
# cv2.imshow("Out",image2)
# cv2.waitKey(0)
# image = cv2.resize(image, None, fx=0.3, fy=0.3, interpolation=cv2.INTER_CUBIC)
cX = []
cY = []
angles = []
data_color = []
color_sequence = ['red', 'yellow', 'green', 'blue', 'purple','pink']
index= []
shape = []
count_color = 0
ii = None
i=0
obj_num = 0
img_hsv = cv2.cvtColor(image.copy(), cv2.COLOR_BGR2HSV)
# img_hsv = cv2.GaussianBlur(img_hsv, (5, 5), 0)
for color in object_color:
    mask = cv2.inRange(img_hsv, color[0], color[1])
    kernel = np.ones((5, 5), np.uint8)
    opening = cv2.morphologyEx(mask, cv2.MORPH_OPEN, kernel)
    __,contours, hierarchy = cv2.findContours(mask.copy(), cv2.RETR_TREE,cv2.CHAIN_APPROX_SIMPLE)
    i=i+1
    # cv2.namedWindow('Output Mark', cv2.WINDOW_AUTOSIZE)
    cv2.imshow('Output Mark'+str(color),mask)
    sd = ShapeDetector()

    for c in contours:
        area = cv2.contourArea(c)
        if area>1700:
            data_color.append(color_sequence[count_color])
            obj_num += 1
            counter[0]+= 1
            M = cv2.moments(c)
            # cX.append(int((M["m10"] / M["m00"])))
            # cY.append(int((M["m01"] / M["m00"])))
            if(M['m00'])!=0:
                cX.append(int((M["m10"] / M["m00"])))
                cY.append(int((M["m01"] / M["m00"])))
            else:
                cX.append(int((M["m10"] / 1)))
                cY.append(int((M["m01"] / 1)))
            shape.append(sd.detect(c))
            epsilon = 0.04 * cv2.arcLength(c, True)
            approx = cv2.approxPolyDP(c, epsilon, True)

            #
            # if shape == 'triangle': #and data_color == 'red':
            #     print(approx)
            # Draw approx
            for i in approx:
                cv2.circle(image, (i[0, 0], i[0, 1]), 3, (0,255,255), -1)
            # cv2.drawContours(image, contours, -1, (0, 255, 0), 3)
            if shape[-1] == "triangle" :
                counter[1]+=1
                # cv2.drawContours(image, [c], -1, (51,153,255), -1)
                p1 = [approx[0][0][0], approx[0][0][1]]
                p2 = [approx[1][0][0], approx[1][0][1]]
                p3 = [approx[2][0][0], approx[2][0][1]]
                d1 = math.pow((p1[0] - p2[0]), 2) + math.pow((p1[1] - p2[1]), 2)
                d2 = math.pow((p2[0] - p3[0]), 2) + math.pow((p2[1] - p3[1]), 2)
                d3 = math.pow((p1[0] - p3[0]), 2) + math.pow((p1[1] - p3[1]), 2)
                a = None
                if (d1 - d2 > 200 and d1 - d3 > 200):
                    a = (-math.atan2((p1[1] - p2[1]), (p1[0] - p2[0])) * 180 / math.pi)
                    P = [p3]
                    if(p2[0]<=p1[0]):
                        P += [p2,p1]
                    else:
                        P += [p1, p2]
                elif (d2 - d1 > 200 and d2 - d3 > 200):
                    a = (-math.atan2((p2[1] - p3[1]), (p2[0] - p3[0])) * 180 / math.pi)
                    P = [p1]
                    if (p2[0] <= p3[0]):
                        P += [p2, p3]
                    else:
                        P += [p3, p2]
                elif (d3 - d1 > 200 and d3 - d2 > 200):
                    a = (-math.atan2((p1[1] - p3[1]), (p1[0] - p3[0])) * 180 / math.pi)
                    P = [p2]
                    if (p3[0] <= p1[0]):
                        P += [p3, p1]
                    else:
                        P += [p1, p3]
                else:
                    a = 9999
                    P = [[0,0],[0,0],[0,0]]
                if a<0:
                    a+=180
                if (P[0][0]<=P[1][0] and P[0][0]<=P[2][0]):
                    tri_twice.append(0)
                elif (P[0][0] <= P[1][0] and P[0][0] <= P[2][0]):
                    tri_twice.append(1)
                elif (P[0][1]>=P[1][1]):
                    if (P[1][1]<=P[2][1]):
                        tri_twice.append(0)
                    else: tri_twice.append(1)
                else:
                    if (P[1][1] <= P[2][1]):
                        tri_twice.append(1)
                    else: tri_twice.append(0)
                print("--t "+str(tri_twice[-1]))
                cv2.putText(image, str(a), (cX[obj_num-1], cY[obj_num-1] + 20), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 0, 0),1)

                angles.append(180-a)
            elif shape[-1] == "square" :
                # cv2.drawContours(image, [c], -1, (128,128,128), -1)
                p1 = [approx[0][0][0], approx[0][0][1]]
                p2 = [approx[1][0][0], approx[1][0][1]]
                p3 = [approx[2][0][0], approx[2][0][1]]
                d1 = math.pow((p1[0] - p2[0]), 2) + math.pow((p1[1] - p2[1]), 2)
                d2 = math.pow((p2[0] - p3[0]), 2) + math.pow((p2[1] - p3[1]), 2)
                print([d1,d2])
                if abs(d1 - d2) > 6000:
                    print('c')
                    if d1 > d2:
                        a = (-math.atan2((p1[1] - p2[1]), (p1[0] - p2[0])) * 180 / math.pi)
                    else:
                        a = (-math.atan2((p2[1] - p3[1]), (p2[0] - p3[0])) * 180 / math.pi)
                    if (a == -180):
                        a = 0
                    shape[-1] = "rectangle"
                    counter[5] += 1
                else:
                    a = (-math.atan2((p1[1] - p2[1]), (p1[0] - p2[0])) * 180 / math.pi)
                if a >= 90:
                    a -= 90
                counter[2] += 1
                cv2.putText(image, str(a), (cX[obj_num - 1], cY[obj_num - 1] + 20), cv2.FONT_HERSHEY_SIMPLEX, 0.5,
                            (0, 0, 0), 1)

                angles.append(180-a)
            elif shape[-1] == "pentagon":
                shape[-1] = "circle"
                counter[4] += 1
                cv2.putText(image, str(0), (cX[obj_num - 1], cY[obj_num - 1] + 20), cv2.FONT_HERSHEY_SIMPLEX, 0.5,
                            (0, 0, 0), 1)
                angles.append(180)
                # cv2.drawContours(image, [c], -1, (153,153,255), -1)
            elif shape[-1] == "circle":
                counter[4]+=1
                cv2.putText(image, str(0), (cX[obj_num - 1], cY[obj_num - 1] + 20), cv2.FONT_HERSHEY_SIMPLEX, 0.5,
                            (0, 0, 0), 1)
                angles.append(180)
                # cv2.drawContours(image, [c], -1, (76,153, 0), -1)
            elif shape[-1] == "rectangle":
                # cv2.drawContours(image, [c], -1, (255, 125, 0), -1)
                p1 = [approx[0][0][0], approx[0][0][1]]
                p2 = [approx[1][0][0], approx[1][0][1]]
                p3 = [approx[2][0][0], approx[2][0][1]]
                d1 = math.pow((p1[0] - p2[0]), 2) + math.pow((p1[1] - p2[1]), 2)
                d2 = math.pow((p2[0] - p3[0]), 2) + math.pow((p2[1] - p3[1]), 2)
                a = None
                if abs(d1 - d2) < 6000:
                    a = (-math.atan2((p1[1] - p2[1]), (p1[0] - p2[0])) * 180 / math.pi)
                    if a >= 90:
                        a -= 90
                    shape[-1] = "square"
                    counter[2] += 1
                else:
                    if d1 > d2:
                        a = (-math.atan2((p1[1] - p2[1]), (p1[0] - p2[0])) * 180 / math.pi)
                    else:
                        a = (-math.atan2((p2[1] - p3[1]), (p2[0] - p3[0])) * 180 / math.pi)
                    if (a == -180 or a==-0):
                        a = 0
                counter[5] += 1
                cv2.putText(image, str(a), (cX[obj_num - 1], cY[obj_num - 1] + 20), cv2.FONT_HERSHEY_SIMPLEX, 0.5,
                            (0, 0, 0), 1)

                angles.append(180-a)
            else :
                counter[6]+=1
                # cv2.drawContours(image, [c], -1, (255,102,178), -1)
                angles.append(None)
            c = c.astype("float")
            c = c.astype("int")
            cv2.putText(image, shape[-1], (cX[-1]+10, cY[-1]), cv2.FONT_HERSHEY_SIMPLEX,0.5, Color_text, 1)
            cv2.circle(image, (cX[-1], cY[-1]), 7, (255, 255, 255), -1)
            center = "("+str(cX[-1])+","+str(cY[-1])+")"
            cv2.putText(image,center,(cX[-1] - 20, cY[-1] - 20), cv2.FONT_HERSHEY_SIMPLEX, 0.5, Color_text, 1)
            super_angle += angles
    count_color+=1
index = [None] * obj_num
ii = 1
Shapei = [None] * obj_num
sequence = []
print(data_color)
tritw = [None]*obj_num
if counter[5] > 0:
    rec = [index for index, value in enumerate(shape) if value == "rectangle"]
    for cl in color_sequence:
        for i in rec:
            if data_color[i] == cl:
                sequence.append(i)
                Shapei[i] = 0
                index[i] = ii
                ii += 1
    del rec
if counter[1] > 0:
    tri = [index for index, value in enumerate(shape) if value == "triangle"]
    for cl in color_sequence:
        for i in tri:
            if data_color[i] == cl:
                sequence.append(i)
                Shapei[i] = 1
                index[i] = ii
                tritw[i] = tri_twice.pop(0)
                ii += 1
    del tri
if counter[2] > 0:
    sq = [index for index, value in enumerate(shape) if value == "square"]
    for cl in color_sequence:
        for i in sq:
            if data_color[i] == cl:
                sequence.append(i)
                Shapei[i] = 2
                index[i] = ii
                ii += 1
    del sq
if counter[4] > 0:
    cir = [index for index, value in enumerate(shape) if value == "circle"]
    for cl in color_sequence:
        for i in cir:
            if data_color[i] == cl:
                Shapei[i] = 3
                sequence.append(i)
                index[i] = ii
                ii += 1
    del cir
for i in range(len(angles)):
    #cv2.putText(image, str((super_angle[i])), (cX[i], cY[i] + 20), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 0, 0), 1)
    cv2.putText(image, str(index[i]), (cX[i] - 20, cY[i] + 20), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (150, 0, 0), 1)
    #print(len(super_angle))
print(sequence)
print(index)
print(angles)
print(Shapei)
y = 20
total = "Total of  Shape : " + str(counter[0])
cv2.putText(image,total,(30,30), cv2.FONT_HERSHEY_SIMPLEX,0.5, (0,0, 0), 1)
triangle = "Triangle  Shape : "+str(counter[1])
cv2.putText(image,triangle,(30,30+y), cv2.FONT_HERSHEY_SIMPLEX,0.5, (0,0, 0), 1)
rectangle = "Rectangle Shape : "+str(counter[5])
cv2.putText(image,rectangle,(30,30+(2*y)), cv2.FONT_HERSHEY_SIMPLEX,0.5, (0,0, 0), 1)
square  =  "Square Shape : "+ str(counter[2])
cv2.putText(image,square,(30,30+(3*y)), cv2.FONT_HERSHEY_SIMPLEX,0.5, (0,0, 0), 1)
pentagon = "Pentagon  Shape : "+str(counter[3])
cv2.putText(image,pentagon,(30,30+(4*y)), cv2.FONT_HERSHEY_SIMPLEX,0.5, (0,0, 0), 1)
circle = "Circle    Shape : "+str(counter[4])
cv2.putText(image,circle,(30,30+(5*y)), cv2.FONT_HERSHEY_SIMPLEX,0.5, (0,0, 0), 1)
undefine = "Unidentified Shape : "+str(counter[6])
cv2.putText(image,undefine,(30,30+(6*y)), cv2.FONT_HERSHEY_SIMPLEX,0.5, (0,0, 0), 1)
cv2.namedWindow('Image', cv2.WINDOW_FREERATIO)
cv2.imshow("Image",image)
cv2.waitKey(0)

x = 'c'
ser = serial.Serial('COM5', 9600, timeout=0)
ser.setDTR(False)
ser.setRTS(False)
sent_data = False
j = 0
buff = 0
step = 1
shape_count = [0,0,0,0] #rec tri sq cir
half_round = 0
positionstr = None
target = None
firsttime = 1
setz0 = 0
#cX = [514-px1]
#cY = [213-py1]
#angle = [90]
#Shapei = [3]
#obj_num =  1
if ser.is_open:
      print('serial is open')
      while True:
            if ser.inWaiting() > 0:
                x = ser.read(1)
                if x == str.encode('f'):
                    print('finished!')
                    sent_data = True
            if(j<obj_num):
                if sent_data:
                    # update parameters here x y z s1 s2
                    # if firsttime==1:
                    #     print('set 0')
                    #     sleep(2)  # Time in seconds.
                    #     #ser.write(str.encode('@'))
                    #     #sleep(2)
                    #     cv2.waitKey(0)
                    #     firsttime = 0
                    if setz0 == 1:
                        print('set 0')
                        setz0 = 0
                        ser.write(str.encode('@'))
                        sent_data  = False
                    elif half_round==0:
                        # if step == 1:  # go
                        #     current_position = [0 + px1 + setzero[0], 0 + py1 + setzero[1]]
                        #     target = Mapto3D(current_position)
                        #     positionstr = '<' + str(int(target[1]*10)) + ',' + str(int(target[0]*10))
                        #     str_command = positionstr
                        #     str_command += ',0,0,900>'
                        #     #str_command = '<0,0,0,0,1700,0>'
                        #     #step+=1
                        # elif step==2:#rotate
                        #     current_position = [cX[j] + px1 + setzero[0], cY[j] + py1 + setzero[1]]
                        #     target = Mapto3D(current_position)
                        #     positionstr = '<' + str(int(target[1] * 10)) + ',' + str(int(target[0] * 10))
                        #     str_command = positionstr
                        #     str_command += ',0,0,900>'
                        #         # str_command = '<0,0,0,0,1700,0>'
                        #     step += 1
                        if step==1:#go
                            print('go')
                            current_position = [cX[sequence[j]] + px1 + setzero[0], cY[sequence[j]] + py1 + setzero[1]]
                            target = Mapto3D(current_position)
                            print(target)
                            positionstr = '<' + str(int(target[1]*10)) + ',' + str(int(target[0]*10))
                            str_command = positionstr
                            str_command += ',0,1000,1800>'
                            #str_command = '<0,0,0,0,0,0>'
                            step+=1
                        elif step==2:#rotate
                            print('rotate')
                            print(angles[sequence[j]]*10)
                            str_command = positionstr +',0,1000,' + str(int(angles[sequence[j]]*10)) + '>'
                            step+=1
                        elif step==3:#z
                            print('z')
                            str_command = positionstr + ',' + str(int(Zshape[Shapei[sequence[j]]])) + ',1000,'
                            str_command += str(int(angles[sequence[j]] * 10)) + '>'
                            step+=1
                        elif step==4:#close jaw
                            print('close')
                            str_command = positionstr + ','
                            str_command += str(int(Zshape[Shapei[sequence[j]]])) + ','
                            str_command += str(Openshape[Shapei[sequence[j]]]) + ','
                            str_command += str(int(angles[sequence[j]] * 10)) + '>'
                            step+=1
                        elif step==5:#-z
                            print('-z')
                            str_command = positionstr  + ',0,'
                            str_command += str(Openshape[Shapei[sequence[j]]]) + ','
                            str_command += str(int(angles[sequence[j]] * 10)) + '>'
                            step+=1
                        elif step==6:#-rotate
                            print('-rotate')
                            if Shapei[sequence[j]]==1:
                                t = tritw[sequence[j]]
                                #shape_count[1]+=1
                                if t==0:
                                    buff = 1800
                                    print('non-inverse')
                                else:
                                    buff = 0
                                    print('inverse')
                            else:
                                buff = 1800
                            str_command = positionstr + ',0,'
                            str_command += str(Openshape[Shapei[sequence[j]]]) + ','
                            str_command += str(buff) + '>'
                            half_round = 1
                            step = 1
                        ser.write(str.encode(str_command))
                        print(step-1)
                        print(str_command)
                        cv2.waitKey(0)
                        sent_data = False
                    else:
                        if step==1:#go
                            print(j)
                            print('go2')
                            current_position = [goal[Shapei[sequence[j]]][0]+setzero[0],goal[Shapei[sequence[j]]][1]+setzero[1]]
                            target = Mapto3D(current_position)
                            positionstr = '<' + str(int(target[1] * 10)) + ',' + str(int(target[0] * 10))
                            str_command = positionstr  + ',0,'
                            str_command += str(Openshape[Shapei[sequence[j]]]) + ','
                            str_command += str(buff) + '>'
                            step += 1
                        elif step==2:#z
                            print('z2')
                            str_command = positionstr  + ','
                            #print(Shapei)
                            #print(j)
                            str_command += str((Zshape[Shapei[sequence[j]]])-200*shape_count[Shapei[sequence[j]]]) + ','
                            str_command += str(Openshape[Shapei[sequence[j]]]) + ','
                            str_command += str(buff) + '>'
                            step+=1
                        elif step==3:#open jaw
                            print('open2')
                            str_command = positionstr + ','
                            str_command += str((Zshape[Shapei[sequence[j]]]) - 220 * shape_count[Shapei[sequence[j]]]) + ',1000,'
                            str_command += str(buff) + '>'
                            step += 1
                        elif step==4:#-z
                            print('-z2')
                            str_command = positionstr + ',0,1000,'
                            str_command += str(buff) + '>'
                            step += 1
                        elif step==5:#rotate to 0
                            print('r to 0')
                            str_command = positionstr + ',0,1000,1800>'
                            half_round = 0
                            step = 1
                            shape_count[Shapei[sequence[j]]] += 1
                            j += 1
                            if j % 2 == 0:
                                setz0 = 1

                        print(step-1)
                        print(str_command)
                        ser.write(str.encode(str_command))
                        cv2.waitKey(0)
                        sent_data = False


            else:
                ser.write(str.encode('@'))
                break
else: print('cannot open port')
print('end')

cv2.destroyAllWindows()