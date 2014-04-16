#!/usr/bin/env python
# /* -*-  indent-tabs-mode:t; tab-width: 8; c-basic-offset: 8  -*- */
# /*
# Copyright (c) 2014, Daniel M. Lofaro <dan (at) danLofaro (dot) com>
# All rights reserved.
#
# Redistribution and use in source and binary forms, with or without
# modification, are permitted provided that the following conditions are met:
#     * Redistributions of source code must retain the above copyright
#       notice, this list of conditions and the following disclaimer.
#     * Redistributions in binary form must reproduce the above copyright
#       notice, this list of conditions and the following disclaimer in the
#       documentation and/or other materials provided with the distribution.
#     * Neither the name of the author nor the names of its contributors may
#       be used to endorse or promote products derived from this software
#       without specific prior written permission.
# 
# THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS" AND
# ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED
# WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
# DISCLAIMED. IN NO EVENT SHALL THE AUTHOR BE LIABLE FOR ANY DIRECT, INDIRECT,
# INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT
# LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR
# PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF
# LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE
# OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF
# ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
# */
import diff_drive
import ach
import sys
import time
from ctypes import *
import socket
import cv2.cv as cv
import cv2
import numpy as np
from math import sqrt

dd = diff_drive
ref = dd.H_REF()
tim = dd.H_TIME()

ROBOT_DIFF_DRIVE_CHAN   = 'robot-diff-drive'
ROBOT_CHAN_VIEW_R   = 'robot-vid-chan-r'
ROBOT_CHAN_VIEW_L   = 'robot-vid-chan-l'
ROBOT_TIME_CHAN  = 'robot-time'
# CV setup 
cv.NamedWindow("wctrl_L", cv.CV_WINDOW_AUTOSIZE)
cv.NamedWindow("wctrl_R", cv.CV_WINDOW_AUTOSIZE)
#capture = cv.CaptureFromCAM(0)
#capture = cv2.VideoCapture(0)

# added
##sock.connect((MCAST_GRP, MCAST_PORT))
newx = 320
newy = 240

nx = 320
ny = 240

r = ach.Channel(ROBOT_DIFF_DRIVE_CHAN)
r.flush()
vl = ach.Channel(ROBOT_CHAN_VIEW_L)
vl.flush()
vr = ach.Channel(ROBOT_CHAN_VIEW_R)
vr.flush()
t = ach.Channel(ROBOT_TIME_CHAN)
t.flush()

i=0


print '======================================'
print '============= Robot-View ============='
print '========== Daniel M. Lofaro =========='
print '========= dan@danLofaro.com =========='
print '======================================'
while True:
    # Get Frame
    imgL = np.zeros((newx,newy,3), np.uint8)
    imgR = np.zeros((newx,newy,3), np.uint8)
    c_image = imgL.copy()
    c_image = imgR.copy()
    vidL = cv2.resize(c_image,(newx,newy))
    vidR = cv2.resize(c_image,(newx,newy))
    [status, framesize] = vl.get(vidL, wait=False, last=True)
    if status == ach.ACH_OK or status == ach.ACH_MISSED_FRAME or status == ach.ACH_STALE_FRAMES:
        vid2 = cv2.resize(vidL,(nx,ny))
        imgL = cv2.cvtColor(vid2,cv2.COLOR_BGR2RGB)
        cv2.imshow("wctrl_L", imgL)
        cv2.waitKey(10)
    else:
        raise ach.AchException( v.result_string(status) )
    [status, framesize] = vr.get(vidR, wait=False, last=True)
    if status == ach.ACH_OK or status == ach.ACH_MISSED_FRAME or status == ach.ACH_STALE_FRAMES:
        vid2 = cv2.resize(vidR,(nx,ny))
        imgR = cv2.cvtColor(vid2,cv2.COLOR_BGR2RGB)
        cv2.imshow("wctrl_R", imgR)
        cv2.waitKey(10)
    else:
        raise ach.AchException( v.result_string(status) )


    [status, framesize] = t.get(tim, wait=False, last=True)
    if status == ach.ACH_OK or status == ach.ACH_MISSED_FRAME or status == ach.ACH_STALE_FRAMES:
        pass
        #print 'Sim Time = ', tim.sim[0]
    else:
        raise ach.AchException( v.result_string(status) )

#-----------------------------------------------------
#-----------------------------------------------------
#-----------------------------------------------------
    # Def:
    # ref.ref[0] = Right Wheel Velos
    # ref.ref[1] = Left Wheel Velos
    # tim.sim[0] = Sim Time
    # imgL       = cv image in BGR format (Left Camera)
    # imgR       = cv image in BGR format (Right Camera)
    
    img1 = cv2.cvtColor(imgL,cv2.COLOR_BGR2RGB)
    img2 = cv2.cvtColor(imgR,cv2.COLOR_BGR2RGB)
    lower_blue = np.array([0,100,0],dtype=np.uint8)
    upper_blue = np.array([0,255,0],dtype=np.uint8)
    mask1 = cv2.inRange(img1,lower_blue,upper_blue) 
    mask2 = cv2.inRange(img2,lower_blue,upper_blue)
    Lcontours,hierarchyL = cv2.findContours(mask1, 1, 2)
    Rcontours,hierarchyR = cv2.findContours(mask2, 1, 2)

    if (Lcontours != [] and Rcontours != []):
	cntL = Lcontours[0]
	cntR = Rcontours[0]
	ML = cv2.moments(cntL)
	MR = cv2.moments(cntR)
	if int(ML['m00']) == 0:
	    ML['m00'] = 1
	if int(MR['m00']) == 0:
	    MR['m00'] = 1	
	cxL = int(ML['m10']/ML['m00'])
	cxR = int(MR['m10']/MR['m00'])
	baseline = .4
	foc = .085
	pwidth = .00028
	dep = (baseline*foc)/(pwidth*np.sqrt((cxL-cxR)*(cxL-cxR)))

        if dep >= 4.5:
	    if i <= 4.5:
		i = i+.1
	elif dep >= 4 and dep < 4.5:
	    if i <= 4.5:
		i = i+ .05
	else:
	    i = i-.05
	leftw = i
	rightw = i
        if (cxL>=160): 
            alpha = cxL-160;
            if alpha >=  60:
                ref.ref[0] = rightw-.4; #right wheel
                ref.ref[1] = leftw; #left wheel
                r.put(ref);
            else:    
	        ref.ref[0] = rightw-.2; #right wheel
	        ref.ref[1] = leftw; #left wheel
	        r.put(ref);
        else:
	    alpha = 160 - cxL
	    if alpha >=  60:
	        ref.ref[0] = rightw; #right wheel
	        ref.ref[1] = leftw-.4; #left wheel
 	        r.put(ref);
	    else:    
 	        ref.ref[0] = rightw; #right wheel
	        ref.ref[1] = leftw -.2; #left wheel
 	        r.put(ref);

    	print "right wheel:",ref.ref[0],"left wheel:",ref.ref[1]
    	print "Distance to object:",dep
    print ''

    # Sleeps
    time.sleep(0.1)   
#-----------------------------------------------------
#-----------------------------------------------------
#-----------------------------------------------------
