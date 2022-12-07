import sys
import math
import os
import geopy
import geopy.distance   
from geopy.distance import great_circle
import time
import random


class Path:
    counter = 1
    erasingFactor = 3
    inf = sys.maxsize

    # Constractor of the class Path. Responsiple for creating X Y coordinates of the area in LatLon.
    def __init__(self, xRange, yRange, lowerLeft, upperLeft, lowerRight, upperRight):
        self.Tiles = xRange * yRange
        self.lowerLeft = lowerLeft
        self.upperRight = upperRight
        self.upperLeft = upperLeft
        self.lowerRight = lowerRight
        self.X = []
        self.Y = []
        self.X.append(self.lowerLeft[0])
        self.Y.append(self.lowerLeft[1])
        self.listV = [0 for i in range(0, self.Tiles)]
        self.visitedSeq = [0 for i in range(0, self.Tiles)]
        bearingY = Path.calculateBearing(self, self.lowerLeft, self.upperLeft)
        bearingX = Path.calculateBearing(self, self.lowerLeft, self.lowerRight)
        # a=vincenty(lowerLeft, upperLeft).meters
        a = great_circle(self.lowerLeft, self.upperLeft).meters
        a /= ((yRange - 1) * 1000)  # distance between nodes =10m

        start = geopy.Point(self.lowerLeft)
        # calculation of the first column of points.
        # print('\nSquare Area nodes\' coordinates\n')
        for i in range(1, yRange):

            d = geopy.distance.distance(kilometers=a)
            uvar = d.destination(point=start, bearing=bearingY)
            self.X.append(uvar.latitude)
            self.Y.append(uvar.longitude)
            # First Column Points of path
            # print(self.X[i],', ',self.Y[i])
            p1 = (start)
            start = geopy.Point(uvar.latitude, uvar.longitude)
            p2 = (start)
            bearingY = Path.calculateBearing(self, p1, p2)

        # a=vincenty(lowerLeft, lowerRight).meters
        a = great_circle(self.lowerLeft, self.lowerRight).meters
        a /= ((xRange-1) * 1000) # distance between nodes =10m
        # calculation of all rows based on the first column.
        # for r in range (1,xRange):
        pointsToCalculate = self.Tiles - yRange
        for g in range(0, pointsToCalculate):
            start = geopy.Point(self.X[g], self.Y[g])
            i += 1
            d = geopy.distance.distance(kilometers=a)
            uvar = d.destination(point=start, bearing=bearingX)
            self.X.append(uvar.latitude)
            self.Y.append(uvar.longitude)

    # Points of path.
    # print(uvar.latitude,', ', uvar.longitude)
    # p1=(start)
    # start = geopy.Point(uvar.latitude, uvar.longitude)
    # p2=(start)
    # bearingY=Path.calculateBearing(self,p1,p2)
    # print('----------------------------------------\n\n')

    # auxiliary function that calculates the bearing between two LatLon points.
    def calculateBearing(self, pointA, pointB):
        lat1 = math.radians(pointA[0])
        lat2 = math.radians(pointB[0])
        diffLong = math.radians(pointB[1] - pointA[1])
        x = math.sin(diffLong) * math.cos(lat2)
        y = math.cos(lat1) * math.sin(lat2) - (math.sin(lat1) * math.cos(lat2) * math.cos(diffLong))
        initial_bearing = math.atan2(x, y)
        initial_bearing = math.degrees(initial_bearing)
        compass_bearing = (initial_bearing + 360) % 360
        return compass_bearing

    def calculateCircuits(self, batteries, currentP, speed):
        listV1 = [0 for i in range(0, self.Tiles)]
        # print "listV1: ", listV1
        # listV1 = visitedNodes
        # print("listV: ", listV1)
        # fleet size
        UAVs = len(batteries)
        batteries = list(batteries)
        # currentP = list(currentP)
        # unit's constant speed in m/s
        # speed=5
        # auxiliary matrix for indexing that no more battery is left for each unit.
        availability = [0 for i in range(0, UAVs)]
        # Initialization of the AdjacencyMatrix and the calculation of the TravellingCost.
        TravellingCost = [[0 for i in range(self.Tiles)] for j in range(self.Tiles)]
        AdjacencyMatrix = [[0 for i in range(self.Tiles)] for j in range(self.Tiles)]
        # t0=time.time()
        # t2=0
        for i in range(0, self.Tiles):
            for j in range(i, self.Tiles):
                if i == j:
                    TravellingCost[i][j] = Path.inf
                    AdjacencyMatrix[i][j] = 0
                elif (i != j):
                    AdjacencyMatrix[i][j] = 1
                    # dist = vincenty([self.X[i],self.Y[i]], [self.X[j],self.Y[j]]).meters
                    dist = great_circle([self.X[i], self.Y[i]], [self.X[j], self.Y[j]]).meters
                    TravellingCost[i][j] = dist / speed  ### + random.randrange(-2,2)
                    TravellingCost[j][i] = TravellingCost[i][j]
        # t2=time.time();
        # print t2-t0
        positions = [0 for i in range(0, UAVs)]

        for z in range(0, UAVs):
            posDiff = Path.inf - 1
            for i in range(0, self.Tiles):
                if posDiff > great_circle([self.X[i], self.Y[i]], [currentP[z][0], currentP[z][1]]).meters:
                    posDiff = great_circle([self.X[i], self.Y[i]], [currentP[z][0], currentP[z][1]]).meters
                    positions[z] = i

        # Initialization of Set S matrices and CircuitX, CircuitY.
        Set_S_source = [[] for i in range(0, UAVs)]
        Set_S_destination = [[] for i in range(0, UAVs)]
        Set_S_cost = [[] for i in range(0, UAVs)]
        circuitsX = [[] for i in range(0, UAVs)]
        circuitsY = [[] for i in range(0, UAVs)]
        # print("POSS: ",positions[:])
        for z in range(0, UAVs):
            listV1[positions[z]] = 1
        # print positions
        # print "before listV: ", listV1
        # assignment of the first K nodes.
        for i in range(0, UAVs):
            futureCost = Path.inf - 1
            for j in range(0, self.Tiles):
                if (listV1[j] == 0):
                    if (futureCost >= TravellingCost[positions[i]][j]):
                        futureCost = TravellingCost[positions[i]][j]
                        node = j
            if (batteries[i] - futureCost) >= TravellingCost[node][positions[i]]:
                # print "visiting node: ", node
                batteries[i] -= futureCost
                listV1[node] = 1
                Set_S_source[i].append(positions[i])
                Set_S_destination[i].append(positions[i])
                Set_S_destination[i].append(node)
                Set_S_cost[i].append(futureCost)
                circuitsX[i].append(self.X[positions[i]])
                circuitsX[i].append(self.X[node])
                circuitsY[i].append(self.Y[positions[i]])
                circuitsY[i].append(self.Y[node])
            else:
                availability[i] = 1
        # print "middle listV: ", listV1
        uav_index = 0
        counter = 0
        sourceNode = 0
        destinationNode = 0
        # assignment of the rest nodes.
        while (sum(listV1) < self.Tiles and sum(availability) < UAVs):
            uav_index = counter % UAVs
            i = Set_S_destination[uav_index][-1]
            futureCost = Path.inf - 1
            for j in range(0, self.Tiles):
                if (listV1[j] == 0):
                    # print j, " not visited with cost: ", TravellingCost[i][j]
                    if (futureCost >= TravellingCost[i][j]):
                        futureCost = TravellingCost[i][j]
                        sourceNode = i
                        destinationNode = j
            if (batteries[uav_index] - futureCost >= TravellingCost[destinationNode][positions[uav_index]]):
                # print "visiting node: ", destinationNode
                batteries[uav_index] -= futureCost
                Set_S_source[uav_index].append(sourceNode)
                Set_S_destination[uav_index].append(destinationNode)
                Set_S_cost[uav_index].append(futureCost)
                circuitsX[uav_index].append(self.X[destinationNode])
                circuitsY[uav_index].append(self.Y[destinationNode])
                listV1[destinationNode] = 1
            else:
                availability[uav_index] = 1
            counter += 1
        # print "end listV: ", listV1
        # print "availability: ", availability

        # circuits=[[0] for i in range(0,UAVs)]
        for k in range(0, UAVs):
            # batteries[k]-=TravellingCost[Set_S_destination[k][-1]][0]
            circuitsX[k].append(self.X[positions[k]])
            circuitsY[k].append(self.Y[positions[k]])
        return (circuitsX, circuitsY)

    def visited(self, nodes):
        pass
        # for x in nodes:
        # self.listV[x]=1
        # self.visitedSeq=Path.counter
        # print("listV: ", self.listV)




def getPaths(drone_ids,lowerLeft,upperLeft, lowerRight, upperRight,batteries, currentPositions, currentAlts, xRange=10, yRange=10):

    # drone_ids = ['drone1', 'drone2', 'drone3', 'drone4']
    # lowerLeft = (35.144590, 33.413255)
    # upperLeft = (35.143711, 33.413477)
    # lowerRight = (35.144357, 33.412174)
    # upperRight = (35.14459, 33.413255)
    # batteries = [3600, 3600, 3600, 3600]
    # currentPosition = [[35.144590, 33.413255], [35.144590, 33.413255], [35.144590, 33.413255], [35.144590, 33.413255]]
    # currentAlts = [23,23,20,20]

    test = Path(xRange, yRange, lowerLeft, upperLeft, lowerRight, upperRight)


    currentPositions = [lowerLeft for _ in batteries] #Let's make them all start at the lower left corner
    (circuitX, circuitY) = test.calculateCircuits(batteries, currentPositions, 4)

    result = []
    for k in range(0, len(batteries)):
        obj = {
            'drone_id':drone_ids[k],
            'path':[[y,x,currentAlts[k]] for y,x in zip(circuitY[k], circuitX[k] )]
        }
        result.append(obj)


    return result

# import pprint
# pprint.pprint(getPaths('','','','','','','',''))