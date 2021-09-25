import math 
from matplotlib import pyplot
import numpy as np

# Spline Interpolation 
# Connecting points 
# Fits between any two points are being called a Spline
# Will use a Cubic Spline   

def knots(x_crdnts: np.ndarray = None, y_crdnts: np.ndarray = None) -> np.ndarray:
    "Generate knots (k) for the given x_crdnts and y_crdnts assuming "
    n = (x_crdnts).shape[0] - 2
    knots = np.zeros(n)

    a = np.zeros((n,n+1))
    a[0,0] = 2 * (x_crdnts[0]-x_crdnts[2])
    a[0,1] = x_crdnts[1] - x_crdnts[2]
    a[-1,-3] = x_crdnts[-3] - x_crdnts[-2]
    a[-1,-2] = 2 * (x_crdnts[-3] - x_crdnts[-1])

    for i in range(1,n-1,1):
        
        a[i][i-1] = x_crdnts[i] - x_crdnts[i+1]
        a[i][i] = 2 * (x_crdnts[i] - x_crdnts[i+2])
        a[i][i+1] = x_crdnts[i+1] - x_crdnts[i+2]


    for i in range(0,n,1):
        temp = (y_crdnts[i] - y_crdnts[i+1])/(x_crdnts[i] - x_crdnts[i+1]) - (y_crdnts[i+1] - y_crdnts[i+2])/(x_crdnts[i+1] - x_crdnts[i+2])
        a[i][-1] = 6 * temp


    # Applying Gauss Elimination
    for i in range(n):        
        for j in range(i+1, n):
            ratio = a[j][i]/a[i][i]
            
            for k in range(n+1):
                a[j][k] = a[j][k] - ratio * a[i][k]


    # Back Substitution
    knots[n-1] = a[n-1][n]/a[n-1][n-1]

    for i in range(n-2,-1,-1):
        knots[i] = a[i][n]
        
        for j in range(i+1,n):
            knots[i] = knots[i] - a[i][j] * knots[j]
        
        knots[i] = knots[i]/a[i][i]

    finalknots = np.zeros(n+2)
    for i in range(0,n,1):
        finalknots[i+1] = knots[i]
    return finalknots


# Will use this function to get the Global Path further in the Task 
def cubicSplineSelf(x_crdnts: np.ndarray = None, y_crdnts: np.ndarray = None, new_x_crdnts: np.ndarray = None) -> np.ndarray:
    "returns numpy array of required y coordinates of corresponding to"
    "new_x_crdnts Interpolated to x_crdnts and y_crdnts"

    KNOTS = knots(x_crdnts, y_crdnts)
    new_y_crdnts = []

    for new_x in new_x_crdnts:

        for idx in range(len(x_crdnts)):
            if new_x < x_crdnts[idx+1]:
                

                ans = 0

                A = KNOTS[idx]/6*( ((new_x-x_crdnts[idx+1])**3)/(x_crdnts[idx]-x_crdnts[idx+1]) - (new_x-x_crdnts[idx+1]) * (x_crdnts[idx]-x_crdnts[idx+1]) )
                B = KNOTS[idx+1]/6*( ((new_x-x_crdnts[idx])**3)/(x_crdnts[idx]-x_crdnts[idx+1]) - (new_x-x_crdnts[idx]) * (x_crdnts[idx]-x_crdnts[idx+1]) )
                C = (y_crdnts[idx] * (new_x - x_crdnts[idx+1]) - y_crdnts[idx+1] * (new_x - x_crdnts[idx])) / (x_crdnts[idx] - x_crdnts[idx+1])

                ans = A + B + C
                new_y_crdnts.append(ans)
                break


    new_y_crdnts = np.array(new_y_crdnts)
    return new_y_crdnts



x_i = np.array([1,2,3,4,5,6,7,8,9,10,11,12])
y_i = np.array([3,7,10,11,11,8,10,5,3,6,8,10])

def main(x_i: np.ndarray, y_i: np.ndarray, xs: float = 0.025):
    "Takes two vecotrs x_i and y_i, do cubic interpolation between them and show them using pyplot"

    x_points = np.arange(x_i[0], x_i[-1],xs)
    y_points = cubicSplineSelf(x_i, y_i, x_points)

    pyplot.plot(x_points,y_points,color='green')
    pyplot.scatter(x_i,y_i,color='black')

    pyplot.title("Spline Interpolation")
    pyplot.xlabel("x_i")
    pyplot.ylabel("y_i")

    pyplot.show()

if __name__ == '__main__':
    main(x_i, y_i)