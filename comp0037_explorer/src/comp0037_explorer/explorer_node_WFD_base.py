import rospy

from explorer_node_base import ExplorerNodeBase

from nav_msgs.msg import Odometry

# Part 2.3

# This class is a base class implementing the wave front detection
# refering the seudocode in the suggested paper 'Frontier Based Exploration for Autonomous Robot'
class ExplorerNodeWFDBase(ExplorerNodeBase):

    def __init__(self):

        self.blackList = []

        # to get self position for searching the frontiers
        self.searchStartCell = None # search start cell / current cell coords
        self.position = None

        # for wave frontier detection
        self.frontierList = []
        self.initFrontierInfo() 

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
        
        position = self.occupancyGrid.getCellCoordinatesFromWorldCoordinates((pos.x,pos.y))
        self.position = position
    
    # if a goal is found unreachable, add it to the blacklist
    def destinationReached(self, goal, goalReached):
        if goalReached is False:
            # print 'Adding ' + str(goal) + ' to the naughty step'
            self.blackList.append(goal)
    
    def chooseNewDestination(self):
        pass

    # ------------------------------------
    def initFrontierInfo(self):
        rospy.loginfo("Clearing old frontier info")

        # use instance variable for efficiency
        self.qM = []
        self.mapOpenList = []
        self.mapCloseList = []

        self.qF = []
        self.newFrontier = []
        self.frontierOpenList = []
        self.frontierCloseList = []

    def isInBoundary(self, cell):
        width, height = self.occupancyGrid.getWidthInCells(), self.occupancyGrid.getHeightInCells()
        return cell[0] in range(0,width) and cell[1] in range(0,height)

    def isEmptyCell(self, cell):
        return self.occupancyGrid.getCell(cell[0], cell[1]) == 0.0
        
    def getNeighbours(self, centerCell):
        offset = [-1,0,1]
        offset2 = [[offsetX,offsetY] for offsetX in offset for offsetY in offset]
        l = [[centerCell[0] + offsetX, centerCell[1] + offsetY] \
                    for offsetX, offsetY in offset2 \
                        if not [offsetX, offsetY] == [0,0]]
        l = filter(lambda x:self.isInBoundary(x),l)
        return l

    def hasAtLeastOneOpenNeighbours(self, cell):
        for neighbours in self.getNeighbours(cell):
            if self.isInBoundary(neighbours) and self.isEmptyCell(neighbours):
                return True
  
    # depth first search, starting with self position
    def searchFrontiers0(self, searchStartCell, frontierList):
        rospy.loginfo("Searching frontiers")

        self.qM = [searchStartCell]
        while len(self.qM) != 0:

            p = self.qM.pop(0)

            if p in self.mapCloseList or p in self.blackList:
                continue

            # found a frontier
            if self.isFrontierCell(p[0], p[1]):

                # init to trace along the frontiers
                self.qF = []
                self.newFrontier = []
                self.qF.append(p)
                self.frontierOpenList.append(p)

                while len(self.qF) != 0:
                    
                    q = self.qF.pop(0)
                    
                    if q in self.mapCloseList or q in self.frontierCloseList or q in self.blackList:
                        continue
                    
                    if self.isFrontierCell(q[0], q[1]):
                        
                        self.newFrontier.append(q)

                        for w in self.getNeighbours(q):
                            if w not in self.frontierOpenList and w not in self.frontierCloseList \
                                    and w not in self.mapCloseList:
                                self.qF.append(w)
                                self.frontierOpenList.append(w)
                    
                    self.frontierCloseList.append(q)
                frontierList += self.newFrontier
                self.mapCloseList += self.newFrontier

            for v in self.getNeighbours(p):
                if v not in self.mapOpenList and v not in self.mapCloseList \
                                and self.hasAtLeastOneOpenNeighbours(v):  
                    self.qM.append(v)
                    self.mapOpenList.append(v)
            self.mapCloseList.append(p)

        return frontierList

    def searchFrontiers(self,searchStartCell,frontierList):
        
        currentCell = searchStartCell
        waitingList = [searchStartCell]
        visitedList = []

        while len(waitingList) != 0:
            currentCell = waitingList.pop(0)

            if currentCell in visitedList or currentCell in self.blackList:
                continue
            
            if self.isFrontierCell(currentCell[0], currentCell[1]):

                currentPotentialFrontier = currentCell
                waitingPotentialFrontierList = [currentCell]

                while len(waitingPotentialFrontierList) != 0:
                    currentPotentialFrontier = waitingPotentialFrontierList.pop(0)

                    if currentPotentialFrontier in visitedList or currentPotentialFrontier in self.blackList:
                        continue

                    if self.isFrontierCell(currentPotentialFrontier[0], currentPotentialFrontier[1]):
                        frontierList.append(currentPotentialFrontier)

                        for neighbours in self.getNeighbours(currentPotentialFrontier):
                            if neighbours not in visitedList:
                                waitingPotentialFrontierList.append(neighbours)
                        
                    visitedList.append(currentPotentialFrontier)

            for neighbours in self.getNeighbours(currentCell):
                if neighbours not in waitingList and neighbours not in visitedList and self.hasAtLeastOneOpenNeighbours(neighbours):
                    waitingList.append(neighbours)
                
            visitedList.append(currentCell)
        
        return frontierList

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

        rospy.loginfo("clearing old info")
        self.initFrontierInfo()

        searchStartCell = self.checkSelfPosition()
        rospy.loginfo("search start cell: (%d, %d)\n",searchStartCell[0],searchStartCell[1])

        frontierList = self.searchFrontiers(searchStartCell, [])

        if len(frontierList) != 0:
            # remove unwanted entry
            self.frontierList = filter(lambda x : x not in self.blackList, frontierList)
            return True
        else:
            return False

            
            

    
