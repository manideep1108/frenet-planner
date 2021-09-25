from planner import * 

cv2.namedWindow("Simulation")


def main():

    # Calculating the Global Path to move on 
    GP = globalPath(MAP, 20)
    GPATH = globalPath(MAP, 5)
    for i in range(len(GPATH)):
        MAP[GPATH[i][0]-2: GPATH[i][0]+3, GPATH[i][1]-2: GPATH[i][1]+3] = [0,100,100]


    x0, y0 = GP[0]
    x1, y1 = GP[1]

    i0 = ego(MAP)[0]
    j0 = ego(MAP)[1]

    itr = 1

    running = True

    while x0 < GP[-1][0] and not (x0 == x1 and y0 == y1) and running:

        PATH = finalFrenet((x0,y0), (x1,y1), MAP)

        if PATH is not None:
            print("Path found..... moving")

            for (i,j) in zip(PATH.x_cartesian[0:len(PATH.time)//2], PATH.y_cartesian[0:len(PATH.time)//2]):

                cv2.circle(MAP,(j,i),1,(255,255,255))

                cv2.circle(MAP,(j0,i0),R,(0,0,0),-1)
                cv2.circle(MAP,(j,i),R,(0,255,0),-1)

                cv2.imshow("Simulation", MAP)
                k = cv2.waitKey(1)
                if k == ord('q'):
                    running = False
                    break

                i0 = i
                j0 = j


            x0 = PATH.x_cartesian[len(PATH.time)//2]
            y0 = PATH.y_cartesian[len(PATH.time)//2]

            itr1 = itr

            while itr < len(GP) - 1 and GP[itr][0] < x0:
                itr+= 1

            itr2 = itr

            if itr < len(GP) - 1 and itr1 == itr2:
                itr += 1

            x1 = GP[itr][0]
            y1 = GP[itr][1]
            



    cv2.imshow("Simulation", MAP)
    cv2.waitKey(0)

if __name__ == "__main__":
    main()