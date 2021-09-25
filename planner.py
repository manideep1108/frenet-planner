from polynomial import * 
from detection import * 

#---------------------------------------- GLOBAL VARIABLES ----------------------------------------------------------#

# Limits for x in Lateral 
UPPER_LAT_X = 200
STEP_LAT_X = 20
# Limits for x in Long
STEP_LONG_X = 50
LOWER_LONG_X = 200
UPPER_LONG_X = 600
# Limits for trajectory time
STEP_TIME = 1
UPPER_TIME = 7
LOWER_TIME = 5

# Limits for v in Long
UPPER_LONG_V = 100
LOWER_LONG_V = 90
STEP_LONG_V = 10
# Limits for a in Long
UPPER_LONG_A = 25
LOWER_LONG_A = 20
STEP_LONG_A = 5
# Target velocity in Long 
TARGET_V = 95

# Ego Vehicle Radius 
R = ego(MAP)[2]  

# Weight constants for making the Cost function
WEIGHT_JERK = 0.2
WEIGHT_TIME = 1
WEIGHT_DISTANCE = 3

# Weight for Lateral and Longitudinal Paths for thier Weighted sum
WEIGHT_LATERAL = 2
WEIGHT_LONGITUDINAL = 1

#------------------------------------------- FRENET CLASS ---------------------------------------------------------#

" This Frenet Class implies to a Single Frenet Trajectory with x,v,a and jerk at each time step "
" In both frenet and Cartesian Frame, Along with the cost of the trajectory "
class Frenet:
    def __init__(self):
        self.time = []

        # Cost of a particular Path 
        self.cost = -1

        # X, Y coordinates at each time steps
        self.x_cartesian = []
        self.y_cartesian = []

        # Lateral motion planning at each time step 
        self.x_lateral = []
        self.v_lateral = []
        self.a_lateral = []
        self.j_lateral = []

        # Longitudinal motion planning at each time step 
        self.x_long = []
        self.v_long = []
        self.a_long = []
        self.j_long = []



#-----------------------------------------FrenetPaths function-------------------------------------------------------#

def FrenetPaths(lateral_x, laternal_v, lateral_a, longi_x, longi_v, longi_a):
    "Generate a List of frenet Object with required start position, velocity, acceleration in Lateral and Longitudinal direction" 

    # List of Frenet Paths
    FrenetPaths = []

    "Sampling Process"

    # Sampling of TIME 
    for TIME in np.arange(LOWER_TIME, UPPER_TIME, STEP_TIME):
        # Sampling based on final distance in Lateral direction 
        for LAT_X in np.arange(-UPPER_LAT_X, UPPER_LAT_X, STEP_LAT_X):
            # Sampling based on Final Distance in Longitudinal Direction
            for LONG_X in np.arange(LOWER_LONG_X,UPPER_LONG_X,STEP_LONG_X):
                # Sampling based on Final Velocity in Longitudinal Direction
                for LONG_V in np.arange(LOWER_LONG_V, UPPER_LONG_V, STEP_LONG_V):
                    # # Sampling based on Final Acceleration in Longitudinal Direction
                    for LONG_A in np.arange(LOWER_LONG_A, UPPER_LONG_A, STEP_LONG_A):

                        # A Frenet object "frenet" created:
                        frenet = Frenet()
                        # Trajectory time steps
                        frenet.time = [t for t in np.arange(0, TIME, STEP_TIME)]


                        # Generation of Lateral Movement
                        # Making state polynomial with the help of polynomial class
                        "Final velocity and acceleration is kept 0 for Optimisation Purposes (target manifold)"
                        state_lateral = Polynomial(lateral_x, laternal_v, lateral_a, LAT_X, 0, 0,  TIME) 
                        # Storing position, velocity, acceleration and jerk
                        frenet.x_lateral = [state_lateral.value(t) for t in frenet.time]
                        frenet.v_lateral = [state_lateral.derivative_1(t) for t in frenet.time]
                        frenet.a_lateral = [state_lateral.derivative_2(t) for t in frenet.time]
                        frenet.v_lateral = [state_lateral.derivative_3(t) for t in frenet.time]


                        # Generation of Longitudinal Movement
                        # Making state polynomial with the help of polynomial class
                        state_long = Polynomial(longi_x, longi_v, longi_a, LONG_X, LONG_V, LONG_A, TIME)
                        # Storing position, velocity, acceleration and jerk
                        frenet.x_long = [state_long.value(t) for t in frenet.time]
                        frenet.v_long = [state_long.derivative_1(t) for t in frenet.time]
                        frenet.a_long = [state_long.derivative_2(t) for t in frenet.time]
                        frenet.j_long = [state_long.derivative_3(t) for t in frenet.time]


                        # Time Integral of square of Jerk
                        Integral_J_Lateral = 0
                        Integral_J_Long = 0

                        for j in frenet.j_lateral:
                            Integral_J_Lateral += j * j * STEP_TIME

                        for j in frenet.j_long:
                            Integral_J_Long += j * j * STEP_TIME

                        # Distance offset square in Lateral Direction
                        Distance_Lateral = (LAT_X - lateral_x) * (LAT_X - lateral_x)

                        # Velocity Keeping Term
                        Vel_Keeping_Lat = (TARGET_V - LONG_V) * (TARGET_V - LONG_V)

                        # Finally storing Cost functions for Lateral and Longitudinal Trajectory respectively
                        frenet.cost_lat = WEIGHT_JERK * Integral_J_Lateral + WEIGHT_TIME * TIME + WEIGHT_DISTANCE * Distance_Lateral
                        frenet.cost_long = WEIGHT_JERK * Integral_J_Long + WEIGHT_TIME * TIME + WEIGHT_DISTANCE * Vel_Keeping_Lat
                        # Combining Lateral and Longitudinal Trajectories 
                        frenet.cost = WEIGHT_LATERAL * frenet.cost_lat + WEIGHT_LONGITUDINAL  * frenet.cost_long

                        # Finally adding "frenet" Frenet into the list
                        FrenetPaths.append(frenet)

    return FrenetPaths


#-----------------------frenet2cartesian Function---------------------------------------------#

def frenet2cartesian(P1: tuple = None, P2: tuple = None) -> list:
    "Generate A list of all Frenet Object b/w P1 and P2 as (x, y) in Cartesian Coordinates"

    # List of all frenets between points P1 and P2
    CartesianFreNetPaths = []

    x1, y1 = P1
    x2, y2 = P2

    ctheta = (x2 - x1) / math.sqrt( (x2 - x1)**2 + (y2 - y1)**2 )
    stheta = (y2 - y1) / math.sqrt( (x2 - x1)**2 + (y2 - y1)**2 )

    FRENETPATHS = FrenetPaths(0,0,0,0,0,0)

    for FREENET in FRENETPATHS:


        for i in range(len(FREENET.time)):

            x = FREENET.x_long[i] * ctheta - FREENET.x_lateral[i] * stheta
            y = FREENET.x_long[i] * stheta + FREENET.x_lateral[i] * ctheta 

            FREENET.x_cartesian.append(x1 + int(x))
            FREENET.y_cartesian.append(y1 + int(y))

        CartesianFreNetPaths.append(FREENET)

    return CartesianFreNetPaths


#---------------------------validFrenet Function-------------------------------------------#


def validFrenet(Frenets: list = None, MAP: np.ndarray = None) -> list:
    " Returns a list of Frenet objects which are valid (are obstacle free) "

    Valid = []

    for frenet in Frenets: 

        if not obstacle(MAP, frenet.x_cartesian, frenet.y_cartesian, R):
            Valid.append(frenet)

    return Valid

#--------------------------------bestFrenet Function-------------------------------------------#

def bestFrenet(Frenets: list = None) -> Frenet:
    "Returns a frenet object which have the lowest cost_final when provided with a list of Frenet objects"

    best = Frenets[0]

    for FRENET in Frenets:  

        if FRENET.cost < best.cost:
            best = FRENET

    return best

#---------------------------------finalFrenet Function----------------------------------------------#

def finalFrenet(P1: tuple = None, P2: tuple = None, MAP: np.ndarray = None) -> Frenet:
    "Utility function which can be called directly to return a best Frenet"

    PATHS = frenet2cartesian(P1, P2)
    PATHS = validFrenet(PATHS, MAP)

    PATH = None

    if len(PATHS) != 0:
        PATH = bestFrenet(PATHS)
        return PATH


#--------------------------------main Function--------------------------------------------------#

def main():
    "Demonstrate Frenet Paths in the first Iteration"

    GPATH = globalPath(MAP, 10)
    for i in range(len(GPATH)):
        MAP[GPATH[i][0]-1: GPATH[i][0]+2, GPATH[i][1]-1: GPATH[i][1]+2] = [0,100,100]

    x0, y0 = GPATH[0]
    x1, y1 = GPATH[1]

    Frenet_paths = FrenetPaths(0,0,0,0,0,0)
    PATHS = frenet2cartesian((x0,y0),(x1,y1))

    pyplot.title("Frenet Planning")
    pyplot.xlabel("Distance moved across the Path")
    pyplot.ylabel("Velocity along the Path")

    for Frenet in Frenet_paths:
        pyplot.plot(Frenet.x_long, Frenet.v_long)



    for PATH in PATHS:
        i0,j0 = GPATH[0]
        for (i,j) in zip(PATH.x_cartesian, PATH.y_cartesian):
            cv2.line(MAP, (j0,i0), (j,i), 255, 1)
            i0 = i
            j0 = j

    pyplot.show()

    cv2.namedWindow("Frenet Path")
    cv2.imshow("Frenet Path", MAP)
    k = cv2.waitKey(0)



if __name__ == "__main__":
    main()