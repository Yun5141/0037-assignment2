import rospy

from explorer_node_base import ExplorerNodeBase

from nav_msgs.msg import Odometry

import random

# Part 2.3

# This class implements a frontier-based explorer using wave front detection 
# and picking the closest frontier as the next waypoint.

class ExplorerNode(ExplorerNodeBase):

    def __init__(self):

        self.blackList_cellCoords = []

        # to get self position for searching the frontiers
        self.searchStartCell = None # search start cell / current cell coords
        self.pose = None

        # for wave frontier detection
        self.frontierList = []  
        self.visitedList = []
        self.boundaryCells = []

        # for picking the closest frontier
        self.frontierDisInfo = []# frontierDisInfo: [[dis, cell]]

        ExplorerNodeBase.__init__(self)  # not sure but has to be put here, or it can't find the above two attributes
        
        self.current_pose_subscriber = rospy.Subscriber('/robot0/odom', Odometry, self.current_pose_callback)


    # pose callback, to get self cell position
    def current_pose_callback(self, data):

        pose = data.pose.pose
        pos = pose.position
        try:
            self.occupancyGrid
        except AttributesError:
            return
        
        pose = self.occupancyGrid.getCellCoordinatesFromWorldCoordinates((pos.x,pos.y))
        self.pose = pose

        
    # if a goal is found unreachable, add it to the blacklist
    def destinationReached(self, goal, goalReached):
        if goalReached is False:
            # print 'Adding ' + str(goal) + ' to the naughty step'
            self.blackList_cellCoords.append(goal)

    # ---------- Wave Front Detection -----------
    def clearOldFrontiersInfo(self):
        rospy.loginfo("clearing old info")
        self.frontierList = []  
        self.visitedList = []
        self.boundaryCells = []  
        self.frontierDisInfo = [] # frontierDisInfo: [dis, cell]
    
    # in A and in B
    def getIntersectList(self, listA, listB):
        return [x for x,_ in filter(lambda xs : xs[0] == xs[1], [[x,y] for x in listA for y in listB])]

    # in A but not in B
    def getDiffList(self, listA, listB):
        return [x for x in listA if x not in listB]

    # return a list of coords of the 8 neighbouring cells
    def getNeighboursCoordsList(self,centerCellCoords):
        offset = [-1,0,1]
        offset2 = [[offsetX,offsetY] for offsetX in offset for offsetY in offset]
        return [[centerCellCoords[0] + offsetX, centerCellCoords[1] + offsetY] \
                    for offsetX, offsetY in offset2 \
                        if not [offsetX, offsetY] == [0,0]]

    # say the cell is invalid if it is in the following cases
    def checkIfCellInvalid(self,cellCoords):
        # outside of the overall grid (x)
        # outside of the overall grid (y)
        # blocked
        # is boundary cell 
        # blacklisted
        return cellCoords[0] not in range(0, self.occupancyGrid.getWidthInCells()) or \
                cellCoords[1] not in range(0, self.occupancyGrid.getHeightInCells()) or \
                self.occupancyGrid.getCell(cellCoords[0],cellCoords[1]) != 0.0 or \
                cellCoords in self.boundaryCells or \
                cellCoords in self.visitedList or \
                cellCoords in self.blackList_cellCoords
    
    # return two list: validNeighbours, boundaryNeighbours
    def getDelicateNeighbours(self, centerCell):
        roughNeighbours = self.getNeighboursCoordsList(centerCell)

        boundaryNeighbours = [cellCoords for cellCoords in roughNeighbours \
                                if self.checkIfCellIsUnknown(cellCoords[0], cellCoords[1], 0, 0)]

        validNeighbours = [cell for cell in roughNeighbours \
                            if not self.checkIfCellInvalid(cell)]

        return validNeighbours, boundaryNeighbours

    # depth first search, starting with self position
    def searchFrontiers(self, depth, currentCell, frontierList):
        if(depth == 900):
            return 

        self.visitedList.append(currentCell)

        validNeighbours, boundaryNeighbours = self.getDelicateNeighbours(currentCell)

        if(self.isFrontierCell(currentCell[0],currentCell[1])):

            if len(validNeighbours) != 0:

                intersectList = []
                for boundaryCell in boundaryNeighbours:
                    neighboursOfBoundary,_ = self.getDelicateNeighbours(boundaryCell)
                    intersectList = intersectList + self.getDiffList(self.getIntersectList(validNeighbours, neighboursOfBoundary), intersectList)
                    # rospy.loginfo("intersect list: " + str(intersectList))

                for intersectCell in intersectList:
                    resList = self.searchFrontiers(depth + 1, intersectCell,frontierList + [currentCell])
                    if resList:
                        return frontierList + self.getDiffList(resList, frontierList)
            else:   # All neighbours are invalid
                # rospy.loginfo("'Dead end' (all neighbours are invalid or visited). \ncurrentFrontierList: " + str(frontierList) + " currentCell: " + str(currentCell))
                return frontierList      
        
        for neighbour in validNeighbours:

            resL = self.searchFrontiers(depth + 1, neighbour,frontierList)
            if resL:
                return frontierList + self.getDiffList(resL, frontierList)
    
    def getArbitraryFreeCell(self):
        rospy.loginfo("initial search cell is None\n")
        width, height = self.occupancyGrid.getWidthInCells(), \
            self.occupancyGrid.getHeightInCells()
        cellMatrix = [[x,y] for x in range(0, width) for y in range(0, height)] 
        return filter(lambda cellCoords : not self.checkIfCellInvalid(cellCoords), \
                                                cellMatrix)[0]

    # wave front detection, using the current self cell position as the search start cell
    def updateFrontiers(self):
        rospy.loginfo("\n")
        rospy.loginfo("enter update frontiers")

        if not self.searchStartCell:
            self.searchStartCell = self.getArbitraryFreeCell()
        
        if not self.pose:
            self.pose = self.searchStartCell
        
        pose = self.pose

        self.clearOldFrontiersInfo()

        rospy.loginfo("search start cell: (%d, %d)\n",pose[0],pose[1])
        frontierList = self.searchFrontiers(0, pose, [])

        if frontierList and len(frontierList) != 0:
            # remove unwanted entry
            l = frontierList[:]
            l = filter(lambda xs : len(xs) != 0, l)
            l = list(set([tuple(t) for t in l]))
            self.frontierList = filter(lambda x : x not in self.blackList_cellCoords, l)
            return True
        
        rospy.loginfo("no frontierList or len(frontierList) == 0")
    
    # ------- Heuristic for picking the new destination ------
    def getDistance(self, cellACoords, cellBCoords):
        return sum(map(lambda xs : (xs[0] - xs[1]) ** 2, zip(cellACoords, cellBCoords)))

    def getFrontiersDisInfo(self, frontierCoordsList):
        x,y = self.searchStartCell
        rospy.loginfo("self coords in get frontier dis info: (%d, %d)\n", x, y)
        #rospy.loginfo("frontierList in getDisInfo: " + str(frontierCoordsList))
        for goalCellCoords in frontierCoordsList:
            dis = self.getDistance([x,y],goalCellCoords)
            #rospy.loginfo("dis: %d\n", dis)
            self.frontierDisInfo.append([dis, goalCellCoords])
        #rospy.loginfo("DisInfoList: " + str(self.frontierDisInfo))

    def getNewDestination(self, frontierDisInfo):
        candidate = [[]]

        # the closest cell will be picked first
        candidate = [cell for dis,cell in frontierDisInfo if dis == min([dis for dis,cellCoords in frontierDisInfo])]

        # if the distances are same the pick cell with the gratest x value
        if len(candidate) > 1:
            rospy.loginfo("found same distance, picking the one with greatest x \n")
            candidate = [cell for cell in candidate if cell[0] == max([x for x,_ in candidate])]

        # if x values are same then pick the one with gratest y value
        if len(candidate) > 1:
            rospy.loginfo("found same distance and same x, picking the one with greatest y \n")
            candidate = [cell for cell in candidate if cell[1] == max([y for _,y in candidate])]
        
        return candidate[0]

    def getArbitraryUnknownCell(self):
        rospy.loginfo("initial search cell is None\n")
        width, height = self.occupancyGrid.getWidthInCells(), \
            self.occupancyGrid.getHeightInCells()
        cellMatrix = [[x,y] for x in range(0, width) for y in range(0, height)] 
        l = filter(lambda cellCoords : not self.checkIfCellInvalid(cellCoords) and \
                                        self.checkIfCellIsUnknown(cellCoords[0],cellCoords[1], 0, 0), \
                        cellMatrix)[0]
        index = random.randint(0, len(l))
        return l[index]


    # pick the closest one
    # due to property of the priority queue, if the distances are same, it will
    # pick the smallest cell.
    def chooseNewDestination(self):

        res = self.updateFrontiers()

        if res:
            frontierCoordsList = self.frontierList[:]
            #rospy.loginfo("frontierList in chooseNewDestination: " + str(frontierCoordsList))
            self.getFrontiersDisInfo(frontierCoordsList)

            if len(self.frontierDisInfo) != 0:
                newDest= self.getNewDestination(self.frontierDisInfo)
                rospy.loginfo("new destination: (%d, %d)",newDest[0],newDest[1])
                return True, (newDest[0], newDest[1])    # the first one is newDestinationAvailable
        '''
        if self.getCoverage() < 75:
            dest = self.getArbitraryUnsureCell()
            rospy.loginfo("randoming the destination: (%d,%d)\n", dest[0], dest[1])
            return True, (dest[0], dest[1])  
        '''
        rospy.loginfo("can't find new destination\n")
        return False, None

            
