import apriltag
import cv2
from pseyepy import Camera
import numpy as np
from playsound import playsound

#Valid tags
'''
  tag36h10
  tag36h11
  tag25h9
  tag16h5
  tagCircle21h7
  tagCircle49h12
  tagStandard41h12
  tagStandard52h13
  tagCustom48h12
'''


aprltagTYP = "tag25h9"
fps = 30 #this is obvious
iro = False #Boolean True is Yes color
gain = 50 #max of 63 higher values increases gain at the cost of noise
exp = 63 #max of 63 higher values increases brightness at the cost of motion blur
#################################################
fov = 44.5 #field of view of both cameras
diff = 8.5725 #the distance between the two cameras from distance
#################################################
#c1 is Left Aoi
c1 = Camera(1, fps=fps, resolution=Camera.RES_LARGE, colour=iro, gain=gain, exposure=exp)
#c2 is Right Akai
c2 = Camera(0, fps=fps, resolution=Camera.RES_LARGE, colour=iro, gain=gain, exposure=exp)

def drwCord(clone, detections):
    global iro #is the camera colored?
    clone = clone.copy() 
    if not iro:
        clone = cv2.cvtColor(clone, cv2.COLOR_GRAY2BGR)
    for r in detections:
            (ptA, ptB, ptC, ptD) = r['lb-rb-rt-lt']
            ptB = (int(ptB[0]), int(ptB[1])) #sets the corner values of the tag6
            ptC = (int(ptC[0]), int(ptC[1]))
            ptD = (int(ptD[0]), int(ptD[1]))
            ptA = (int(ptA[0]), int(ptA[1]))
            # draw the bounding box of the apriltag detection
            cv2.line(clone, ptA, ptB, (255, 255, 0), 2) #shita            creates the line color from ptn to ptn by cordination)
            cv2.line(clone, ptB, ptC, (0, 255, 255), 2) #migi
            cv2.line(clone, ptC, ptD, (255, 0, 255), 2) #ue
            cv2.line(clone, ptD, ptA, (0, 0, 255), 2) #hidari 
            # draw the center (x, y)-coordinates of the AprilTag
            (cX, cY) = (int(r['center'][0]), int(r['center'][1]))
            cv2.circle(clone, (cX, cY), 5, (0, 0, 255), -1) #center of the tag
            # draw the tag family on the image
            tagFamily =  "tagnooya> " + str(aprltagTYP) + " " + str(r['id']) 
            cv2.putText(clone, tagFamily, (ptA[0], ptA[1] - 15),
                cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 255, 0), 2)
    return clone.copy()

def aprldec(img):
    global aprltagTYP, iro
    if iro: #is the color?
        img = cv2.cvtColor(img, cv2.COLOR_BGR2GRAY)
    at_detector = apriltag.apriltag(
            family = aprltagTYP,
            maxhamming = 1,
            threads = 16, 
            decimate = 1.0,
            blur = 0.8,
            refine_edges = False,
            debug = 0
        )
    detections = at_detector.detect(img)
         
    return detections #returns the detected values in a array

def dispcalculation(Lcam, Rcam, Ltrk, Rtrk):
    global fov, diff, iro
    if iro:
        high, wid, dept = Lcam.shape
    else:
        high, wid = Lcam.shape
    #init half of the equation
    foc = (wid * 0.5) / np.tan(fov * 0.5 * np.pi/180) 
    #check if theres even a tracked image
    if not Ltrk or not Rtrk:
        return 0 #returns 0 bc theres nothing there
    x_L = Ltrk[0]['center']
    x_R = Rtrk[0]['center']
    disp = x_L-x_R #get the cordnates between the iamges by pixel
    zDTH = abs((diff*foc)/disp) #finish the equation
    return zDTH[0]/30.48



def indivUndistortion(file):    
    loadedFile = np.load(file)
    objPtns = loadedFile['objPtns'] #gets the objptns from saved params
    imgPtns = loadedFile['imgPtns'] #gets the imgptns from saved params
    ret, mtx, dist, rvecs, tvecs = cv2.calibrateCamera(objPtns, imgPtns, (640,480), None, None)
    newcameramtx, roi = cv2.getOptimalNewCameraMatrix(mtx, dist, (640,480), 1, (640,480))
    return newcameramtx, dist, roi
    
def stereoUndistortion(file, LnewCamMtx, RnewCamMtx, Ldist, Rdist):
    loadedFile = np.load(file) #load the file given from the params
    objPtns = loadedFile['objPtns']
    LimgPtns = loadedFile['LimgPtns']
    RimgPtns = loadedFile['RimgPtns']
    #calibrate the stereo
    retStereo, newCameraMatrixL, distL, newCameraMatrixR, distR, rot, trans, essentialMatrix, fundamentalMatrix = cv2.stereoCalibrate(
    objPtns, #always the same for both cameras
    LimgPtns, #gets the image point from the Left cam in StereoParams
    RimgPtns, #gets the iamge point from the right cam in StereoParams
    LnewCamMtx, #the left cam undistorted matrix
    Ldist, #Left distortion value given from undistortionstep2 function
    RnewCamMtx, #the right cam undistorted matrix
    Rdist, #right distortion value given from undistortionstep2 function
    (640, 480), #height and width from both camera
    criteria = (cv2.TERM_CRITERIA_EPS + cv2.TERM_CRITERIA_MAX_ITER, 100, 0.001), #criteria given DO NOT CHANGE
    flags = cv2.CALIB_FIX_INTRINSIC #flags DO NOT CHANGE
    )
    ####################################################
    rectify = 1 #rectify value
    #initiate rectification
    rectL, rectR, projMatrixL, projMatrixR, Q, roi_l, roi_r = cv2.stereoRectify(
    newCameraMatrixL, #after stereo calibrate get the newNEWcamera matrix from the Left
    distL, #the NEW distortion value given in stereocalibration
    newCameraMatrixR, #same thing as Left but for the right
    distR, #same for the left but for the right now
    (640,480), #height and width
    rot, #rotation rectification (if one of the camera is rotated along the horizon)
    trans, #Translation vector idk wtf this is i'm trans 知らないけどｗｗｗ
    rectify, #rectification value
    (0,0) #wtf is this
    )
    #make undistorted values maps
    stereoMapL = cv2.initUndistortRectifyMap(newCameraMatrixL, 
                                             distL, 
                                             rectL, 
                                             projMatrixL, 
                                             (640,480), 
                                             cv2.CV_16SC2) 
    stereoMapR = cv2.initUndistortRectifyMap(newCameraMatrixR, 
                                             distR, 
                                             rectR, 
                                             projMatrixR, 
                                             (640,480), 
                                             cv2.CV_16SC2)

    return stereoMapL, stereoMapR, roi_l, roi_r

#done with the functions
#mains

#preq



LnewCamMtx, Ldist, L_Roi1 = indivUndistortion('LeyeParams.npz') #get undistorted values for LEFT
RnewCamMtx, Rdist, R_Roi1 = indivUndistortion('ReyeParams.npz') #get undistorted values for RIGHT

#get new camera matrix
stereoMpL, stereoMpR, Lroi, Rroi = stereoUndistortion('StereoParams.npz',
                                          LnewCamMtx, 
                                          RnewCamMtx,
                                          Ldist,
                                          Rdist)

#this is for cropping the cameras so it fits

#x1,y1,w1,h1 = L_Roi1
#x2,y2,w2,h2 = R_Roi1

#x1,y1,w1,h1 = R_Roi1
#x2,y2,w2,h2 = L_Roi1

x1,y1,w1,h1 = Lroi
x2,y2,w2,h2 = Rroi

#x1,y1,w1,h1 = Rroi
#x2,y2,w2,h2 = Lroi

while True:
    v1, timestamp1 = c1.read() #L cam
    v2, timestamp2 = c2.read() #R cam
    #undistort it
    v1 = cv2.remap(v1, stereoMpL[0], stereoMpL[1], cv2.INTER_LANCZOS4, cv2.BORDER_CONSTANT, 0)
    v2 = cv2.remap(v2, stereoMpR[0], stereoMpR[1], cv2.INTER_LANCZOS4, cv2.BORDER_CONSTANT, 0)
    #crop it
    v1 = v1[y1:y1+h1, x1:x1+w1]
    v2 = v2[y2:y2+h2, x2:x2+w2]
    #get detection values
    v1Cent = aprldec(v1) #detect the tag
    v2Cent = aprldec(v2)
    #get the distance in ft
    zDistance = dispcalculation(v1, v2, v1Cent, v2Cent)
    zDistanceInch = round((zDistance - int(zDistance))*12, 1) #rounds to the nearest tenth after inch calculation
    #draw the tracked apriltag
    v1 = drwCord(v1, v1Cent) 
    v2 = drwCord(v2, v2Cent)
    #put test on the Left image
    cv2.putText(v1, str(int(zDistance)) + "' " + str(zDistanceInch) + '"', 
                (50,80), 
                cv2.FONT_HERSHEY_SIMPLEX, 
                2, 
                (220,255,0),
                3)
    
    cv2.imshow("Left-Hidari", v1)
    cv2.imshow("Right-Migi", v2)

    if cv2.waitKey(1) == ord('q'):
            break
    
