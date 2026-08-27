import math
import robomaster
import cv2
import threading
import os
import time
import sys
import json
import tempfile
from robomaster import robot
from robomaster import config
config.LOCAL_IP_STR = "192.168.10.2"

resultPath = "./targetLocation.json"

poseRecord = [[0, 0, 0, 0]]
errorAccumulation = 0
arucoParams = cv2.aruco.DetectorParameters()
drone = robot.Drone()
drone.initialize()
flight = drone.flight
camera = drone.camera
targetId = 3
tagSizeCm = 7.5
focalLength = 890
droneHeight = 0
markerFound = False
lastTargetLoc = (-1, -1, -1, -1, -1)
flightLock = threading.Lock()

# Number of consecutive detections before we trust the sighting
confirmFrames = 3

# Approximate cruise speed in cm/s, used to estimate how far an aborted move got.
# Measure this: time a straight 200cm hop and divide.
cruiseSpeedCmS = 50

# The downward camera output is mirrored, so frames are flipped before detection.
# flipCode 1 = horizontal. Set to None to disable if you switch cameras.
flipCode = 1

# Sign conventions mapping image axes to drone axes for the DOWNWARD camera.
# Verify empirically: place the tag clearly ahead of the drone and check that
# fDist comes out positive, then place it clearly left and check lDist positive.
imageYToForward = -1
imageXToLeft = 1

arucoDict = cv2.aruco.getPredefinedDictionary(cv2.aruco.DICT_4X4_100)

# Used for the wrapper methods for drone movement
movementDict = {
    "f": flight.forward,
    "b": flight.backward,
    "l": flight.left,
    "r": flight.right,
    "u": flight.up,
    "d": flight.down
}

detector = cv2.aruco.ArucoDetector(arucoDict, arucoParams)

def waitForComplete(action, timeout = 15):
    global markerFound
    startTime = time.time()
    while (time.time() - startTime) < timeout:
        if action.is_completed:
            return True
        if markerFound:
            try:
                flight.rc(0, 0, 0, 0)
            except Exception:
                pass
            return False
        time.sleep(0.05)
    return False

def subDistanceHandler():
    global droneHeight
    while True:
        try:
            droneHeight = drone.sensor.get_distance()
        except Exception:
            pass
        time.sleep(0.2)

def publishResult(x, y, radius):
    payload = {"x": x, "y": y, "radius": radius}
    tmpFd, tmpPath = tempfile.mkstemp(dir=os.path.dirname(resultPath))
    try:
        with os.fdopen(tmpFd, "w") as f:
            json.dump(payload, f)
        os.replace(tmpPath, resultPath)
    except Exception as e:
        print("Failed to write result file: " + str(e))
        if os.path.exists(tmpPath):
            os.remove(tmpPath)

# Self-explanatory, computes a rotational change in pose
def rotUpdate(rotAddition, prevRot):
    return (prevRot + rotAddition) % 360

# Handles updating the pose. Uses dead reckoning, so not accurate. Error accumulation used to calculate variation
def poseUpdate(fDist, lDist, vDist, rotation):
    global poseRecord
    global errorAccumulation
    global droneHeight
    prevPose = poseRecord[-1]
    newPose = prevPose.copy()
    heading = math.radians(prevPose[3])
    newPose[0] = prevPose[0] + (fDist * math.cos(heading)) + (lDist * math.cos(heading + (math.pi / 2)))
    newPose[1] = prevPose[1] + (fDist * math.sin(heading)) + (lDist * math.sin(heading + (math.pi / 2)))
    newPose[2] = droneHeight
    newPose[3] = rotUpdate(rotation, prevPose[3])
    poseRecord.append(newPose)

# Handles the drone movement methods and pose update calls.
# On an aborted move the drone has still travelled some distance, so the pose is
# updated with an estimate rather than being left stale.
def moveDrone(dist, dirKey):
    if dist < 20 or markerFound:
        return
    with flightLock:
        startTime = time.time()
        currentAction = movementDict[dirKey](distance=dist)
        completed = waitForComplete(currentAction)
        elapsed = time.time() - startTime
    if completed:
        travelled = dist
    else:
        travelled = min(dist, elapsed * cruiseSpeedCmS)
        print("Move aborted, estimated " + str(round(travelled)) + "cm of " + str(dist) + "cm")
    fDist = (int(dirKey == "f") - int(dirKey == "b")) * travelled
    lDist = (int(dirKey == "l") - int(dirKey == "r")) * travelled
    vDist = (int(dirKey == "u") - int(dirKey == "d")) * travelled
    poseUpdate(fDist, lDist, vDist, 0)

# Wrapper function for drone rotation
def rotateDrone(degrees):
    if markerFound:
        return
    with flightLock:
        action = flight.rotate(angle=degrees)
        completed = waitForComplete(action, timeout = 15)
    if completed:
        poseUpdate(0, 0, 0, degrees)

# When killswitch activated, stop all movement and land.
# Deliberately does NOT take flightLock: the lock is usually held by an in-progress
# move, and waiting for it would delay the landing by up to the move timeout.
def killswitch():
    global flight
    global drone
    global markerFound
    while(True):
        if(input() == "#"):
            markerFound = True
            try:
                flight.rc(0, 0, 0, 0)
            except Exception:
                pass
            try:
                flight.land()
            except Exception:
                pass
            time.sleep(3)
            os._exit(1)

# Prints current pose
def printPose():
    global poseRecord
    print("X: " + str(poseRecord[-1][0]) + " Y: " + str(poseRecord[-1][1]) + " Z: " + str(poseRecord[-1][2]) + " ROT: " + str(poseRecord[-1][3]))

# Searches the space below the camera for an AruCo marker with the correct tag, and
# returns its center position on the camera if found. Coordinates are returned in
# true (un-mirrored) camera space so downstream geometry does not need to compensate.
def cameraCheck(timeout = 0.25):

    # print("cameraCheck")

    global detector
    global targetId
    startTime = time.time()
    while (time.time() - startTime) < timeout:
        try:
            img = camera.read_cv2_image(strategy = "newest", timeout = 1)
        except Exception:
            img = None

        if img is not None:
            if flipCode is not None:
                workingImage = cv2.flip(img, flipCode)
            else:
                workingImage = img
            greyscaleImage = cv2.cvtColor(workingImage, cv2.COLOR_BGR2GRAY)
            corners, ids, rejected = detector.detectMarkers(greyscaleImage)
            if ids is not None and len(ids) > 0:
                # print('tag found')
                # print(ids)
                idsFlattened = ids.flatten()
                frameHeight, frameWidth = img.shape[:2]
                targetFound = False
                targetX = -1
                targetY = -1
                pixelWidth = -1
                for i, tagCorners in enumerate(corners):
                    cornerPoints = tagCorners[0]
                    centerX = int(sum(cornerPoint[0] for cornerPoint in cornerPoints) / 4)
                    centerY = int(sum(cornerPoint[1] for cornerPoint in cornerPoints) / 4)
                    if idsFlattened[i] == targetId:
                        targetFound = True
                        targetX = centerX
                        targetY = centerY
                        pixelWidth = math.sqrt((cornerPoints[0][0] - cornerPoints[1][0])**2 + (cornerPoints[0][1] - cornerPoints[1][1])**2)
                if targetFound:
                    # Undo the horizontal mirror so targetX is in true camera coords
                    if flipCode == 1 or flipCode == -1:
                        targetX = frameWidth - 1 - targetX
                    if flipCode == 0 or flipCode == -1:
                        targetY = frameHeight - 1 - targetY
                    return targetX, targetY, pixelWidth, frameWidth, frameHeight
        time.sleep(0.02)
    return -1, -1, -1, -1, -1

# Observation only. This thread never moves the drone and never takes flightLock,
# so it cannot deadlock against an in-progress move. Confirmation is done by
# requiring several consecutive detections rather than by flying a check manoeuvre.
def camThreadManager():
    global markerFound
    global lastTargetLoc
    consecutiveHits = 0
    try:
        while not markerFound:
            currentResult = cameraCheck()
            if currentResult[0] != -1:
                consecutiveHits = consecutiveHits + 1
                lastTargetLoc = currentResult
                print("HIT " + str(consecutiveHits) + " at " + str(currentResult[0]) + ", " + str(currentResult[1]) + " width " + str(round(currentResult[2])) + "px")
                if consecutiveHits >= confirmFrames:
                    markerFound = True
                    print("MARKER CONFIRMED")
                    break
            else:
                consecutiveHits = 0
            time.sleep(0.05)
    except Exception:
        import traceback
        traceback.print_exc()
        print("Cam thread died")

# General search logic, just moving around the space it exists in until the camera thread detects the marker
def searchSpace():
    global markerFound
    global lastTargetLoc
    rowOffset = 100
    for i in range(6):
        if markerFound:
            break

        n_forward = 6
        for j in range(n_forward if (i==0) else n_forward//2):
            if markerFound:
                break
            # moveDrone(60, "f")
            moveDrone(60, "f" if (i % 2 == 0) else "b")
            time.sleep(1)
        if markerFound:
            break
        # turnDir = "l" if (i % 2 == 0) else "r"
        # rotateDrone(90 if turnDir == "l" else -90)
        turnDir = "r"
        if markerFound:
            break
        moveDrone(rowOffset, turnDir)
        # moveDrone(rowOffset, "f")
        if markerFound:
            break
        # rotateDrone(90 if turnDir == "l" else -90)
    return lastTargetLoc

def errorThreadHandler():
    global errorAccumulation
    while(True):
        errorAccumulation = errorAccumulation + 1
        time.sleep(1)

# Uses image data to calculate the relative possible location of the target cube.
# Geometry assumes a DOWNWARD-facing camera: distanceCm is the vertical distance to
# the tag, and the tag's offset from image centre gives its lateral offset from the
# point directly beneath the drone.
def determineTagCoords():
    global poseRecord
    global tagSizeCm
    global focalLength
    global errorAccumulation
    moveDrone(130, "u")
    time.sleep(0.5)
    targetLoc = searchSpace()
    if(targetLoc and targetLoc[0] != -1):
        targetX, targetY, pixelWidth, frameWidth, frameHeight = targetLoc
        print("TARGET FOUND")
        print(str(targetX) + ", " + str(targetY))
        distanceCm = (tagSizeCm * focalLength) / pixelWidth
        xCam = (targetX - (frameWidth / 2)) * distanceCm / focalLength
        yCam = (targetY - (frameHeight / 2)) * distanceCm / focalLength
        fDist = imageYToForward * yCam
        lDist = imageXToLeft * xCam
        print("Height to tag: " + str(round(distanceCm)) + "cm, offset f=" + str(round(fDist)) + " l=" + str(round(lDist)))
        prevPose = poseRecord[-1]
        heading = math.radians(prevPose[3])
        tagX = prevPose[0] + (fDist * math.cos(heading)) + (lDist * math.cos(heading + (math.pi / 2)))
        tagY = prevPose[1] + (fDist * math.sin(heading)) + (lDist * math.sin(heading + (math.pi / 2)))
        tagCoords = [tagX, tagY]
        time.sleep(0.5)
    else:
        print("NO TARGET, falling back to last pose")
        tagCoords = poseRecord[-1][:2]
    with flightLock:
        moveDrone(60, "f")
        flight.land().wait_for_completed(timeout = 15)
    result = [tagCoords[0], tagCoords[1], (15 + (errorAccumulation * 0.875))]
    publishResult(result[0], result[1], result[2])
    return result

camera.start_video_stream(display=False)
time.sleep(2)

try:
    waitForComplete(flight.takeoff())
    distThread = threading.Thread(target = subDistanceHandler, daemon = True)
    errorThread = threading.Thread(target = errorThreadHandler, daemon = True)
    killswitchThread = threading.Thread(target = killswitch, daemon = True)
    camThread = threading.Thread(target = camThreadManager, daemon = True)
    distThread.start()
    killswitchThread.start()
    camThread.start()
    errorThread.start()
    print(determineTagCoords())

    # while True:
    #     img = camera.read_cv2_image()
    #     cv2.imshow("Drone", img)
    #     cv2.waitKey(1)
finally:
    cv2.destroyAllWindows()
    # drone.close()