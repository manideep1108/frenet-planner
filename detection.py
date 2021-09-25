import cv2 
import numpy as np 
from queue import PriorityQueue
import math
import os 
from spline import * 

# Change the path of Image according to your OS if not working 
map_path = os.path.join(os.getcwd(),"Map.png")
print(map_path)
MAP = cv2.imread(map_path)
if MAP is None:
    print("Unable to Read the Image!")
    exit()

MAPGRAY = cv2.cvtColor(MAP,cv2.COLOR_BGR2GRAY)

#-------------------------------isValid Function-------------------------------------------------------#

def isValid(i: int = None, j: int = None, MAP: np.ndarray = None) -> bool:
    "returns True if the given (i,j) is valid for the given MAP shape"
    if i > 0 and j > 0 and i < MAP.shape[0]-1 and j < MAP.shape[1]-1:
        return True
    return False

#---------------------------------ego Function-------------------------------------------------------------------#

def ego(MAP: np.ndarray = None) -> tuple:
    "returns a 3 membered tuple consisting of x,y,r of ego, when MAP Image is passed into it"
    lower = np.array([55, 160, 25]).astype("uint8")
    upper = np.array([90, 195, 50]).astype("uint8")
    mask = cv2.inRange(MAP,lower,upper)

    contours, hier = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)

    if contours:
        c = contours[0]
        (x,y),r = cv2.minEnclosingCircle(c)
        r = int(r)
        x = int(x)
        y = int(y)
        return x,y,r
    
    return None

#---------------------------------waypoint Function------------------------------------------------------------#

def waypoint(MAP: np.ndarray = None) -> PriorityQueue:
    "returns a Pq based on increasing x coordinates of waypoints given in the MAP"

    lower = np.array([15,15,120]).astype("uint8")
    upper = np.array([50,40,250]).astype("uint8")
    mask = cv2.inRange(MAP, lower, upper)

    contours, hier = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)

    way = PriorityQueue()

    for contour in contours:
        (x,y),r = cv2.minEnclosingCircle(contour)
        x = int(x)
        y = int(y)
        way.put((x,y))

    return way 

#-------------------------------globalWayPoints Function--------------------------------------------------------#

def globalWayPoints(MAP: np.ndarray = None) -> list:
    'returns list of tuples (x,y,r) with increasing x coordinates of Global Path'

    # Basically this function includes the initial postition of vehicle in the path along with the wayPoints
    path = []
    path.append((ego(MAP)[0], ego(MAP)[1]))
    
    ways = waypoint(MAP)
    while not ways.empty():
        x,y = ways.get()
        path.append((x,y))

    return path

#--------------------------------obstacle Function---------------------------------------------------------#

def obstacle(MAP: np.ndarray = None, FrenetX: list = None, FrenetY: list = None, R: int = None) -> bool:
    "Returns False only if every coordinate in Frenet list is free of obstacle in a distance range of R"

    for i,j in zip(FrenetX, FrenetY):
        if not isValid(i,j,MAP):
            return True
        if np.array_equal(MAP[int(i),int(j)], np.array([255,255,255])):
            return True
        
        
        for theta in range(0, 360, 15):

            x = i + R * math.cos(theta * math.pi / 180)
            y = j + R * math.sin(theta * math.pi / 180)

            if not isValid(x,y,MAP):
                return True

            if np.array_equal(MAP[int(x),int(y)], np.array([255,255,255])):
                return True

    return False

#-----------------------------------globalPath Function-------------------------------------------------------------#

def globalPath(MAP: np.ndarray = None, k: int = None) -> list:
    'returns a list of tuple (x,y) with increasing x coordinates with a differnce of k from start point ot end point'
    "uses cubic spline interpolation to do that"

    Path = globalWayPoints(MAP)

    X = []
    Y = []
 
    for path in Path:
        X.append(path[0])
        Y.append(path[1])

    xs = np.arange(Path[0][0],Path[len(Path)-1][0],k)

    X = np.array(X)
    Y = np.array(Y)

    Spline = cubicSplineSelf(X,Y,xs)

    GP = []
    for i in range(xs.shape[0]):
        GP.append((int(Spline[i]), xs[i]))

    return GP


#-----------------------------------main Function-------------------------------------------------------------#

def main(MAP: np.ndarray = None, gap: int = 5):
    "Demonstration how Cubic Spline Interpolation is working on our MAP to generate the global path"

    GPATH = globalPath(MAP, gap)
    for i in range(len(GPATH)):
        MAP[GPATH[i][0]-1: GPATH[i][0]+2, GPATH[i][1]-1: GPATH[i][1]+2] = [255,100,100]

    cv2.namedWindow("Global Path")
    cv2.imshow("Global Path", MAP)
    k = cv2.waitKey(0)
    if k == 27:
        cv2.destroyAllWindows()


if __name__ == '__main__':
    main(MAP)