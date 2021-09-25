import numpy as np 
import math

#--------------------------------------- Polynomial Class ---------------------------------------------------#

class Polynomial:

    # Calculate all  coefficient of this quintic polynomial 
    def __init__(self, xstart, vstart, astart, xend, vend, aend, time):
        self.a0 = xstart
        self.a1 = vstart
        self.a2 = astart / 2.0
        self.a3 = (20 * xend - 8 * time * vend + time * time * aend - 20 * xstart - 12 * vstart * time - 3 * astart * time * time) / (2 * (time ** 3))
        self.a4 = (20 * xend - 4 * time * vend - 20 * xstart - 16 * vstart * time - 6 * astart * time * time - 8 * self.a3 * (time ** 3)) / (4 * (time ** 4))
        self.a5 = (4 * time * vend - 4 * time * vstart - 4 * astart * (time ** 2) - 12 * self.a3 * (time ** 3) - 16 * self.a4 * (time ** 4)) / (20 * (time ** 5))

    # Calculate value of polynomial at t 
    def value(self, time ):

        ans = self.a0 + self.a1 * time  + self.a2 * (time **2) + self.a3 * (time  ** 3) + self.a4 * (time  ** 4) + self.a5 * (time  ** 5)
        return ans

    # Calculate value of first derivative of polynomial at t 
    def derivative_1(self, time ):

        ans = self.a1 + 2 * self.a2 * time  + 3 * self.a3 * (time  ** 2) + 4 * self.a4 * (time  ** 3) + 5 * self.a5 *  (time  ** 4)
        return ans

    # Calculate value of second derivative of polynomial at t 
    def derivative_2(self, time ):

        ans = 2 * self.a2 + 6 * self.a3 * time  + 12 * self.a4 * (time  ** 2) + 20 * self.a5 * (time  ** 3)
        return ans

    # Calculate value of third derivative of polynomial at t 
    def derivative_3(self, time ):

        ans = 6 * self.a3 + 24 * self.a4 * time  + 60 * self.a5 * (time  ** 2)
        return ans

#--------------------------------------------------------------------------------------------------------------------#