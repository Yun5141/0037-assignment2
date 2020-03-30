import rospy
from explorer_node_WFD_base import ExplorerNodeWFDBase

# Part 2.3

# This class implements an explorer 
# which picks the closest cell as the next destination.
class ExplorerNode(ExplorerNodeWFDBase):

    def __init__(self):
        ExplorerNodeWFDBase.__init__(self)

    def getDistance(self, cellACoords, cellBCoords):
        return sum(map(lambda xs : (xs[0] - xs[1]) ** 2, zip(cellACoords, cellBCoords)))

    def getFrontiersDisInfo(self, frontierCoordsList):
        x,y = self.position
        frontierDisInfo = []
        rospy.loginfo("self coords in get frontier dis info: (%d, %d)\n", x, y)
        for goalCellCoords in frontierCoordsList:
            dis = self.getDistance(goalCellCoords,[x,y])
            frontierDisInfo.append([dis, goalCellCoords])
        return frontierDisInfo

    def getClosestCell(self, frontierList):
        frontierDisInfo = self.getFrontiersDisInfo(frontierList)
        
        try:
            candidate = [cell for dis,cell in frontierDisInfo \
                if dis == min(filter(lambda d : d > 0.5, [dis for dis,cellCoords in frontierDisInfo]))]
        except ValueError:
            return []

        return candidate[0]

    # pick the closest one as the new destination
    def chooseNewDestination(self):

        res = self.updateFrontiers()
        if len(self.frontierList) != 0:
            newDest= self.getClosestCell(self.frontierList)
            if newDest:
                rospy.loginfo("new destination: (%d, %d)",newDest[0],newDest[1])
                return True, newDest    # the first one is newDestinationAvailable
    
        rospy.loginfo("Can't find new destination\n")
        return False, None
