from openravepy import *
from numpy import *
#from velctl import *
import time
import sys
from scipy.spatial import distance


class simulation:
    env = Environment()
    robot = 0
    ikmodel = 0
    basemanip = 0
    def __init__(self):
        self.env.SetViewer('qtcoin')
        self.env.Load(sys.argv[2])

    def setRobot(self):
        self.robot = self.env.GetRobots()[0] # get the first robot
        self.ikmodel = databases.inversekinematics.InverseKinematicsModel(self.robot, iktype=IkParameterization.Type.Transform6D)
        self.basemanip=interfaces.BaseManipulation(self.robot)
        self.robot.SetActiveDOFs([0, 1, 2, 3, 4, 5])    

    def moveCyl(self,c,Tplace):
        cyl = RaveGetEnvironment(1).GetKinBody(c)
        Tcyl = cyl.GetTransform()
        # print Tcyl
        Tpick = eye(4)
        Tpick[0:3,3] = Tcyl[0:3,3]
        Tpick[0:3,0:3] = array([[-1,0,0],[0,1,0],[0,0,-1]])
        Tpick[2,3] = Tpick[2,3] + 0.071
        solutions = self.ikmodel.manip.FindIKSolutions(Tpick,IkFilterOptions.CheckEnvCollisions)
        # print Tpick

        # Move to pick location
        traj = self.basemanip.MoveActiveJoints(goal=solutions[0])
        self.robot.WaitForController(0)

        # Grab cylinder
        with self.env:
            self.robot.Grab(cyl)

        # Move to place location
        place = Tpick.copy()
        place[0,3] = Tplace[0]
        place[1,3] = Tplace[1]
        place[2,3] = Tplace[2]
        # print place
        solutions = self.ikmodel.manip.FindIKSolutions(place,IkFilterOptions.CheckEnvCollisions)
        traj = self.basemanip.MoveActiveJoints(goal=solutions[0])
        self.robot.WaitForController(0)

        # Drop object
        self.robot.ReleaseAllGrabbed()


def OrderStack(mySim, outputStack):
    inputStack,tempStack = stackOrder();
    destLocation = [-0.25, 0.62, 0.105]
    # print inputStack,tempStack
    if len(inputStack)+len(tempStack)<len(outputStack):
        for block in outputStack:
            mySim.moveCyl(block,destLocation)
            destLocation[2] += 0.061
        return 0

    inputBase = locationOf(inputStack[-1])

    inputBase[2] = 0
    tempBase = (locationOf(tempStack[-1]) if tempStack!=[] 
                else [locationOf(inputStack[-1])[0]+0.16, 0.6,0])
    tempBase[2] = 0.105
    # print tempBase
    outputStack = outputStack[::-1]
    ans = []
    inputLocation = inputBase
    inputLocation[2] = 0.03 + (len(inputStack) * 0.061) 
            #0.03 = base position of centre of object # 0.06 is height of block

    tempLocation = tempBase
    tempLocation[2] = tempLocation[2] + (len(tempStack) * 0.061 )

    while len(outputStack) != 0:
        first = outputStack.pop(0)
        while 1:
            if len(inputStack) > 0 and inputStack[0] == first:
                moveBlock = inputStack.pop(0)
                mySim.moveCyl(moveBlock,destLocation)
                destLocation[2] += 0.061
                inputLocation[2] -= 0.06
                break

            elif len(tempStack) > 0 and tempStack[0] == first:
                moveBlock = tempStack.pop(0)
                mySim.moveCyl(moveBlock,destLocation)
                destLocation[2] += 0.061
                tempLocation[2] -= 0.06
                break

            else:

                if len(inputStack) == 0:
                    tempStack, inputStack = inputStack, tempStack;
                    tempLocation, inputLocation = inputLocation, tempLocation
                    tempLocation[2] = 0.105
                        #Take top block from input loaction

                moveBlock = inputStack.pop(0)
                tempStack = [moveBlock] + tempStack
                        # Put on block on temp location
                # print tempLocation
                mySim.moveCyl(moveBlock, tempLocation)
                tempLocation[2] += 0.061

###############################################################################
# locationOf : char -> Location
# GIVEN  : char c
# WHERE  : c represents the cyclinder color
# RETURN : location of the cylinder
def locationOf(c):
    return RaveGetEnvironment(1).GetKinBody(c).GetTransform()[0:3,3]

###############################################################################
# stackOrder : void -> CharStack
# GIVEN    : Nothing
# RETURN   : Ordered stack of cyclinders that exists in the environment
def stackOrder():
    input1 = list()
    input2 = list()
    cylLocList = []
    for i in range(65,91):
        try :
         lColor = RaveGetEnvironment(1).GetKinBody(chr(i)).GetTransform()[0:3,3]
         lColor = list(lColor) + [chr(i)]
         cylLocList.append(lColor)
        except :
            continue

    for l in cylLocList:
        if input1 == [] or isInRadius(input1[-1],l):
            input1.append(l)
        elif input2 == [] or isInRadius(input2[-1],l):
            input2.append(l)
      
    input1.sort(key=lambda x: -x[2])
    input2.sort(key=lambda x: -x[2])

    return [[l[3] for l in input1],[l[3] for l in input2]]

##############################################################################
# compare : Location Location -> Boolean
# GIVEN   : two locations
# RETURN  : True iff l2 is above of l1
def compare(l1,l2):
    return  1 if l1[2] > l2[2] else -1
   
###############################################################################
# isStacked : Location Location -> Boolean
# GIVEN   : Location lAbove, Location lBelow
# RETURN  : True iff lAbove is the center of cylinder stacked exactly upon the 
#           cylinder with lBelow
# EXAMPLE : isStacked([0.2,0.1,0.01],[0.2,0.1,0.07]) => True
#           isStacked([0.3,0.1,0.01],[0.2,0.1,0.07]) => False
#           isStacked([0.3,0.1,0.07],[0.2,0.1,0.01]) => False
# STRATEGY: combine simpler functions
def isStacked(lAbove,lBelow):
    return (isInRadius(lAbove,lBelow) and 
            lAbove[2]-lBelow[2] < 0.061 and
            lAbove[2]-lBelow[2] >= 0)

###############################################################################
# insertInStack : CharStack Char Char -> CharStack
# GIVEN  : CharStack stack1, char above, char below
# RETURN : a CharStack containing the order of cyclinders in the environment
# WHERE  : above and below represents cyclinders which needs to be inserted in 
#          the stack. Here,
#            "R" represents Red Cylinder
#            "G" represents Green Cylinder
#            "B" represents Blue Cylinder
# EXAMPLE: insertInStack([],"R","G") => ["R","G"]
#          insertInStack(["R","G"],"B","R") => ["B","R","G"]
# STRATEGY: Case on the existing elements of the input CharStack
def insertInStack(stack1,above,below):
    if stack1 == []:
            stack1.append(above)
            stack1.append(below)
    elif above in stack1:
        stack1.insert(stack1.index(above)+1,below)
    elif below in stack1:
        stack1.insert(stack1.index(below),above)
    else:
        stack1.append(below)
        stack1.append(above)
    return stack1

###############################################################################
# isInRadius : Location Location -> Boolean
# GIVEN   : Location l1, Location l2
# RETURN  : True iff the projection of cirles with center l1 and l2 and radius
#           0.03 intersects in XY plane
# EXAMPLE : isInRadius([0,0,0],[0,0.01,0.2]) => True
#           isInRadius([0,0,0],[0,0.04,0]) => False
# STRATEGY: distance(l1,l2)<R1+R2
def isInRadius(l1,l2):
    return distance.euclidean(l1[0:2],l2[0:2]) < 0.06


if __name__ == "__main__":
    mySim = simulation()
    mySim.setRobot()
    raw_input("")
    OrderStack(mySim, list(sys.argv[1]))
    raw_input("")