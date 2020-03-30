import rospy

from explorer_node_base import ExplorerNodeBase

from nav_msgs.msg import Odometry

# Part 2.3

# This class is a base class implementing the wave front detection.
class ExplorerNodeWaveFrontDetectionBase(ExplorerNodeBase):

    def __init__(self):

        self.blackList = []

        # to get self position for searching the frontiers
        self.searchStartCell = None # search start cell / current cell coords
        self.position = None

        # for wave frontier detection
        self.frontierList = []  
        visitedList = []

        # for picking the closest frontier
        self.frontierDisInfo = []# frontierDisInfo: [[dis, cell]]

        ExplorerNodeBase.__init__(self)  # not sure but has to be put here, or it can't find the above two attributes
        
        self.current_pose_subscriber = rospy.Subscriber('/robot0/odom', Odometry, self.current_pose_callback)


    # pose callback to get self cell position
    def current_pose_callback(self, data):

        pose = data.pose.pose
        pos = pose.position
        try:
            self.occupancyGrid
        except AttributesError:
            return
        
        self.position = self.occupancyGrid.getCellCoordinatesFromWorldCoordinates((pos.x,pos.y))
        #self.position = position
    
    # if a goal is found unreachable, add it to the blacklist
    def destinationReached(self, goal, goalReached):
        if goalReached is False:
            # print 'Adding ' + str(goal) + ' to the naughty step'
            self.blackList.append(goal)

    def clearOldFrontiersInfo(self):
        rospy.loginfo("clearing old info")
        self.frontierList = []  
    '''
    def isEmpty(self,cell):
        if self.isInBoundary(cell) and \
                self.occupancyGrid.getCell(cell[0],cell[1]) == 0:
                return True
    '''

    def isInBoundary(self, cell):
        width, height = self.occupancyGrid.getWidthInCells(), self.occupancyGrid.getHeightInCells()
        return cell[0] in range(0,width) and cell[1] in range(0,height)
    
    def getNeighbours(self, centerCell):
        offset = [-1,0,1]
        offset2 = [[offsetX,offsetY] for offsetX in offset for offsetY in offset]
        l = [[centerCell[0] + offsetX, centerCell[1] + offsetY] \
                    for offsetX, offsetY in offset2 \
                        if not [offsetX, offsetY] == [0,0]]
        l = filter(lambda x:self.isInBoundary(x),l)
        return l
  
    # depth first search, starting with self position
    def searchFrontiers(self, searchStartCell, frontierList):
        rospy.loginfo("Searching frontiers")

        # init
        currentCell = searchStartCell 
        waitingList = [searchStartCell] 
        visitedList = []  

        while len(waitingList) != 0:

            currentCell = waitingList.pop(-1)

            if currentCell in visitedList or currentCell in self.blackList:
                continue

            # found a frontier
            if self.isFrontierCell(currentCell[0], currentCell[1]):

                # init to trace along the frontiers
                currentPotentialFrontier = currentCell
                waitingPotentialFrontierList = [currentCell] 

                while len(waitingPotentialFrontierList) != 0:
                    currentPotentialFrontier = waitingPotentialFrontierList.pop(-1)
                    
                    if currentPotentialFrontier in visitedList or \
                        currentPotentialFrontier in self.blackList:
                        continue
                    
                    if self.isFrontierCell(currentPotentialFrontier[0], currentPotentialFrontier[1]):
                        frontierList.append(currentPotentialFrontier)

                        for neighbours in self.getNeighbours(currentPotentialFrontier):
                            if neighbours not in visitedList:
                                waitingPotentialFrontierList.append(neighbours)
                    
                    visitedList.append(currentPotentialFrontier)

            # add target neighbours into waiting list
            for neighbours in self.getNeighbours(currentCell):
                if neighbours not in waitingList and neighbours not in visitedList \
                                        and self.isInBoundary(neighbours):  
                    waitingList.append(neighbours)

            visitedList.append(currentCell)

        return frontierList

    def checkIfCellInvalid(self,cell):
        return self.isInBoundary(cell) and \
            cell not in self.blackList

    def isEmptyCell(self, cell):
        return self.occupancyGrid.getCell(cell[0], cell[1]) == 0.0

    def getArbitraryFreeCell(self):
        rospy.loginfo("initial search start cell is None\n")
        width, height = self.occupancyGrid.getWidthInCells(), \
            self.occupancyGrid.getHeightInCells()
        cellMatrix = [[x,y] for x in range(0, width) for y in range(0, height)] 
        return filter(lambda cell : self.isEmptyCell(cell), cellMatrix)[0]

    def checkSelfPosition(self):
        if not self.searchStartCell:
            self.searchStartCell = self.getArbitraryFreeCell()
        if not self.position:
            self.position = self.searchStartCell
        return self.position

    # wave front detection, using the current self cell position as the search start cell
    def updateFrontiers(self):
        rospy.loginfo("Update frontiers")

        self.clearOldFrontiersInfo()

        searchStartCell = self.checkSelfPosition()
        rospy.loginfo("search start cell: (%d, %d)\n",searchStartCell[0],searchStartCell[1])

        frontierList = self.searchFrontiers(searchStartCell, [])

        if len(frontierList) != 0:
            # remove unwanted entry
            l = frontierList[:]
            l = filter(lambda xs : len(xs) != 0, l)
            l = list(set([tuple(t) for t in l]))    # remove duplicate values
            self.frontierList = filter(lambda x : x not in self.blackList, l)
            return True
        else:
            return False

            
            

    
