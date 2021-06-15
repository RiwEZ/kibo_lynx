import numpy as np
import cv2 as cv

def main():
    img = cv.imread("test.png")
    
    dictionary = cv.aruco.getPredefinedDictionary(cv.aruco.DICT_5X5_250)
    corners, markerids, rejected = cv.aruco.detectMarkers(img, dictionary)

    imgCopy = img
    cv.aruco.drawDetectedMarkers(imgCopy, corners, markerids)
    
    cameraMat = np.array([[567.229305, 0.0, 659.077221]
                        , [0.0, 574.192915, 517.007571]
                        , [0.0, 0.0, 1.0]])
    distCoeffs = np.array([-0.216247, 0.03875, -0.010157, 0.001969, 0.0])

    rvecs, tvecs, _objPoints = cv.aruco.estimatePoseSingleMarkers(corners, 0.05, cameraMat, distCoeffs)

    for i in range(markerids.size):
        cv.aruco.drawAxis(imgCopy, cameraMat, distCoeffs, rvecs[i], tvecs[i], 0.05)      

    # x, z, y
    camCord = np.array([11.302, 5.274, -9.785 ])

    arPoint = camCord - tvecs[0]   
    print(arPoint)

    cv.imshow("test", imgCopy)
    cv.waitKey(0)

if __name__ == "__main__":
    main()
