import random
import time
import heapq
class Controller:
    def __init__(self, currentDistance = 0, calculatedDistance = 0):
        self.currentDistance = currentDistance
        self.calculatedDistance = calculatedDistance
        self.timecycle = 0
        self.g = Grid(0,0,0,0)

    def setupEnvironment(self):
        rows, cols = map(int, input("enter rows and columns: ").split())
        robotNum = int(input("enter number of robots: "))
        exitNum = int(input("enter number of exits: "))
        self.g = Grid(rows, cols, robotNum, exitNum)
        rpoints = []
        epoints = []
        for _ in range(robotNum):
            point = (random.randint(0, rows-1), random.randint(0, cols-1))
            while point in rpoints:
                point = (random.randint(0, rows-1), random.randint(0, cols-1))
            rpoints.append(point)
            robot = Robot(point)
            self.g.robots.append(robot)
            cell = self.g.cells[point[0]][point[1]]
            cell.setRobot(robot) 
        self.g.printGrid()
        for i in range(exitNum):
            y,x = map(int, input("enter location of exit " +  str(i) +" in form of y,x: ").split(","))
            while (y,x)  in epoints or (y,x) in rpoints:
               y,x = map(int, input("Location already exist!, enter location of exit " +  str(i) +" in form of y,x again: ").split(",")) 
            epoints.append((y,x))
            exit = ExitCell((y,x))
            self.g.exits.append(exit)
            cell = self.g.cells[y][x]
            cell.setExit(exit)
        for i in range(rows):
            for j in range(cols):
                print(self.g.cells[i][j])
            print()
        print("\n")

    def runSimulation(self):
        #assignTimerCycle (already done in the initializer)
        self.g.printGrid()
        robotList = self.g.getRobotList()
        exitList = self.g.getExitList()
        for r in robotList:
            price,cell,path = self.getShortestPath(r)
            r.setShortDistance(price)
            r.setPath(path)
            r.setTargetExit(cell.getExit().getPoint())
            print(f"robot info: {r}")
            print("Robot path: ")
            for i in r.getPath():
                print(i.getPoint(), end =", ")
            print()
        #WILL BE FIXED LATER (DSA)
        # for r in robotList:
        #     currentDistance = None
        #     beste = None
        #     for e in exitList:
        #         price, cell, path = self.calculateDistance(r,e)
        #         shortDist = r.getShortDistance()
        #         if price < currentDistance:
        #             currentDistance = price
        #             beste = e
        #     if price == (3/2)*(shortDist):
        #         p = beste.getPoint()
        #         r.setTargetExit(p)
        #         r.setShortDistance(currentDistance)
        #         r.setPath(path)
        #     print(f"robot info: {r}")
        #     print("Robot path: ")
        while len(robotList) != 0:
            #simulate cycle
            print(len(robotList))
            print(f"at cycle {self.timecycle}: ")
            self.g.printGrid()
            for r in robotList:
                #set direction 
                #set speed
                #check collision
                path = r.getPath()
                #check if path is empty (we reached the exit)
                if not path:
                    rPos = r.getPoint()
                    self.g.cells[rPos[0]][rPos[1]].setRobot(None)
                    robotList.remove(r)
                #check if no collision
                elif path[0].getRobot() == None:
                    rPos = r.getPoint()
                    self.g.cells[rPos[0]][rPos[1]].setRobot(None)
                    r.move()
                    rPos = r.getPoint()
                    self.g.cells[rPos[0]][rPos[1]].setRobot(r)
                    r.popPath() 
            self.incrementTimer()
            time.sleep(3)
        self.g.printGrid()
                
    def getShortestPath(self, robot):
        #using BFS to find the shortest path (also must return the price, and assign targetExit to robot)
        def bfs(robot):
            robotPoint = robot.getPoint()
            robotY, robotX = robotPoint[0],robotPoint[1]
            queue = []#add initial states
            if robotY+1 < self.g.cols:
                queue.append((self.g.cells[robotY+1][robotX].getCost(), self.g.cells[robotY+1][robotX], []))
            if robotY-1 >= 0:
                queue.append((self.g.cells[robotY-1][robotX].getCost(), self.g.cells[robotY-1][robotX], []))
            if robotX-1 >= 0:
                queue.append((self.g.cells[robotY][robotX-1].getCost(), self.g.cells[robotY][robotX-1], []))
            if robotX+1 < self.g.rows:
                queue.append((self.g.cells[robotY][robotX+1].getCost(), self.g.cells[robotY][robotX+1], [])) 
            visited = []
            while queue:
                cost, u, path = queue.pop(0) 
                if u.getExit() != None:
                    return (cost+u.getCost() ,u,path+[u])
                row, col = u.getPoint()
                lst = []
                if row+1 < self.g.rows:
                    lst.append((self.g.cells[row+1][col].getCost(), self.g.cells[row+1][col], path+[u]))
                if row-1 >= 0:
                    lst.append((self.g.cells[row-1][col].getCost(), self.g.cells[row-1][col], path+[u]))
                if col-1 >= 0:
                    lst.append((self.g.cells[row][col-1].getCost(), self.g.cells[row][col-1], path+[u]))
                if col+1 < self.g.cols:
                    lst.append((self.g.cells[row][col+1].getCost(), self.g.cells[row][col+1], path+[u]))   
                for neighbor in lst:
                    if neighbor not in visited:
                        visited.append(neighbor)
                        queue.append((cost+neighbor[0], neighbor[1], path + [u]))
            return None
        price, cell, path = bfs(robot)
        return (price, cell, path)

    def calculateDistance(self,robot, exit):
        def dsa(robot):
            robotPoint = robot.getPoint()
            robotY, robotX = robotPoint[0],robotPoint[1]
            queue = heapq.heapify([])#add initial states
            if robotY+1 < self.g.cols:
                heapq.heappush(queue, [self.g.cells[robotY+1][robotX].getCost(), self.g.cells[robotY+1][robotX], []])
            if robotY-1 >= 0:
                heapq.heappush(queue, [self.g.cells[robotY-1][robotX].getCost(), self.g.cells[robotY-1][robotX], []])
            if robotX-1 >= 0:
                heapq.heappush(queue, [self.g.cells[robotY][robotX-1].getCost(), self.g.cells[robotY][robotX-1], []])
            if robotX+1 < self.g.rows:
                heapq.heappush(queue, [self.g.cells[robotY][robotX+1].getCost(), self.g.cells[robotY][robotX+1], []]) 
            visited = []
            while queue:
                cost, u, path = queue.pop(0) 
                if u.getExit() != None:
                    return cost,u,path
                row, col = u.getPoint()
                lst = []
                if row+1 < self.g.rows:
                    lst.append([self.g.cells[row+1][col].getCost(), self.g.cells[row+1][col], path+[u]])
                if row-1 >= 0:
                    lst.append([self.g.cells[row-1][col].getCost(), self.g.cells[row-1][col], path+[u]])
                if col-1 >= 0:
                    lst.append([self.g.cells[row][col-1].getCost(), self.g.cells[row][col-1], path+[u]])
                if col+1 < self.g.cols:
                    lst.append([self.g.cells[row][col+1].getCost(), self.g.cells[row][col+1], path+[u]])  
                for neighbor in lst:
                    if neighbor not in visited:
                        visited.append(neighbor)
                        heapq.heappush(queue, [cost+neighbor[0], neighbor[1], path + [u]])
        price, cell, path = dsa(robot)
        return (price, cell, path) 
            
    def assignRobotNumbers(self, robots):
        pass

    def assignExitCells(self, exitCells):
        pass

    def assignLocation(self, location):
        pass

    def incrementTimer(self):
        self.timecycle += 1


class Grid:
    def __init__(self, rows, cols, numOfExits, numOfRobots):
        self.rows = rows
        self.cols = cols
        self.numOfExits = numOfExits
        self.numOfRobots = numOfRobots
        self.cells = [[Cell(point=(i,j)) for j in range(cols)] for i in range(rows)]
        self.robots = []
        self.exits = []
    def setRobotNumbers(self, robots):
        pass

    def addRobot(self, robot):
        self.robots.append(robot)
    
    def getRobotList(self):
        return self.robots
    
    def getExitList(self):
        return self.exits

    def __getitem__(self, point):
        y,x = point
        return self.cells[y][x]
    
    def printGrid(self):
        print("-" + "--"*(2*self.cols-1) + "--")
        for i in range(self.rows):
            print("|",end="")
            for j in range(self.cols):
                if self.cells[i][j].getRobot() is not None:
                    print(" R ", end = "|")
                elif self.cells[i][j].getExit() is not None: 
                    print(" E ", end = "|")
                else:
                    print(f" {self.cells[i][j].getCost()} ",end = "|")
            print("\n|" + "---|"*(self.cols)) if i < (self.rows-1) else print()
        print("-" + "--"*(2*self.cols-1) + "--")

class Robot:
    direction = ['l','r','u','d']
    def __init__(self, point: tuple = (), shortDistance = 0, path = None):
        self.speed  = random.randint(1,2)
        self.direction = Robot.direction[random.randint(0,3)]
        self.currentPos = point #point will be made from Grid class
        self.targetExit = None #point
        self.shortDistance = shortDistance
        self.path = path
    
    def setTargetExit(self, point):
        self.targetExit = point
        pass

    def setSpeed(self, n):
        self.speed = n

    def setDirection(self, dir):
        self.direction = dir
    
    def setPosition(self,pos):
        self.currentPos = pos

    def popPath(self):
        self.path.pop(0)

    def move(self):
        cell = self.path[0]
        #get difference between robot and next cell distance
        movement = tuple(map(lambda x,y: x-y,cell.getPoint(),self.currentPos))
        print(movement)
        #change position of robot
        self.currentPos = tuple(map(lambda x,y: x+y, movement,self.currentPos))

    
    def getShortDistance(self):
        return self.shortDistance
        
    def setShortDistance(self,sd):
        self.shortDistance = sd
        
    def getPoint(self):
        return self.currentPos

    def setPath(self, path):
        self.path = path
    
    def getPath(self):
        return self.path
    def __str__(self):
        return f"currentPos: {self.currentPos} path : {self.path} with cost of {self.shortDistance}"

class ExitCell:
    def __init__(self, position: tuple):
        self.position = position
    def getPoint(self):
        return self.position

class Cell:
    def __init__(self, rb=None, exit=None, cost = 1, point=0):
        self.rb = rb
        self.exit = exit
        self.cost = random.randint(1, 5)
        self.point = point
    def getRobot(self):
        return self.rb
    def getExit(self):
        return self.exit
    def getCost(self):
        return self.cost
    def setCost(self, c):
        self.cost = c
    def setRobot(self, r:Robot):
        self.rb = r
    def setExit(self, e:ExitCell):
        self.exit = e
    def getPoint(self):
        return self.point
    def __str__(self):
        return f"cost = {self.cost}, robot = {self.rb}, exit = {self.exit}, point = {self.point}"


a = Controller()
a.setupEnvironment()
a.runSimulation()