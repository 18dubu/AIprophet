import cv2
import numpy as np

def nothing(x):
    pass

cap = cv2.VideoCapture('../data/hexbug-training_video.mp4')
fout = open('centroid_data','w')
fout.write('[')

centroid = []
ctr = 0

hmin = (238-30)/2
hmax = (238+30)/2

smin = 255/2
smax = 255

vmin = int(0.20 * 255)
vmax = int(1.0 * 255)

# define range of color in HSV
lower = np.array([hmin,smin,vmin])
upper = np.array([hmax,smax,vmax])

while True:

    try:
        _, frame = cap.read()
        hsv = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)
        ctr += 1
        if ctr % 100 == 0:
            print 'frame', ctr
            s = ''
            for c in centroid:
                s += str(c) + ',\n'
            fout.write(s)
            del centroid[:]
    except:
        break

##    # create trackbars for color change
##    cv2.createTrackbar('H','mask',0,180,nothing)
##    cv2.createTrackbar('S','mask',0,255,nothing)
##    cv2.createTrackbar('V','mask',0,255,nothing)
##
##    # get current positions of four trackbars
##    h = cv2.getTrackbarPos('H','mask')
##    s = cv2.getTrackbarPos('S','mask')
##    v = cv2.getTrackbarPos('V','mask')
##    hsv_list.append((h,s,v))
##
##    hradius = 36
##    sradius = 51
##    vradius = 51
##
##    hmin = max(  0,h-hradius)
##    hmax = min(180,h+hradius)
##    smin = max(  0,s-sradius)
##    smax = min(255,s+sradius)
##    vmin = max(  0,v-vradius)
##    vmax = min(255,v+vradius)

    # Threshold the HSV image to get only selected colors
    mask = cv2.inRange(hsv, lower, upper)
#    cv2.imshow('mask',mask)

    ret,thresh = cv2.threshold(mask,127,255,0)
    contours,hierarchy = cv2.findContours(thresh, 1, 2)
    if contours:
        cnt = contours[0]
        M = cv2.moments(cnt)
        if M['m00'] != 0:
            cx = int(M['m10']/M['m00'])
            cy = int(M['m01']/M['m00'])
            centroid.append([cx,cy])
        else:
            centroid.append([-1,-1])

##    k = cv2.waitKey(5) & 0xFF
##    if k == 27:
##        break

s = ''
for c in centroid:
    if c is centroid[-2]:
        s += str(c) + ']\n'
        break
    else:
        s += str(c) + ',\n'
fout.write(s)

print 'all done!'

fout.close()
cap.release()
cv2.destroyAllWindows()
