import random
import time
import heapq
import math


class Controller:
    def __init__(self, currentDistance = 0, calculatedDistance = 0):
        self.currentDistance = currentDistance
        self.calculatedDistance = calculatedDistance
        self.timecycle = 0
        self.g = Grid(0,0,0,0)

    def notifyAll(self):
        robotList = self.g.getRobotList()
        for r in robotList:
                print(f"Iterating over Robot #{r.getId()}")
                r.determineSpeed()
                r.determineDirection()
                path = r.getPath()
                #check if path is empty (we reached the exit)
                if not path:
                    print(f"Robot {r.getId()} has reached exit!!!")
                    rPos = r.getPoint()
                    self.g.cells[rPos[0]][rPos[1]].setRobot(None)
                    print(f"Deleting Robot #{r.getId()}")
                    rCopy = r
                    robotList.remove(rCopy)
                elif path[r.getSpeed()-1].getRobot() is None:
                    rPos = r.getPoint()
                    self.g.cells[rPos[0]][rPos[1]].setRobot(None)
                    r.move()
                    print(f"Robot {r.getId()} has moved!")
                    rPos = r.getPoint()
                    self.g.cells[rPos[0]][rPos[1]].setRobot(r)
                    r.popPath() 


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

    def runSimulation(self):
        #assignTimerCycle (already done in the initializer)
        robotList = self.g.getRobotList()
        exitList = self.g.getExitList()
        for r in robotList:
            price,cell,path = self.getShortestPath(r)
            r.setShortDistance(price)
            r.setPath(path)
            r.setTargetExit(cell.getExit().getPoint())
            
        for r in robotList:
            currentDistance = math.inf
            beste = None
            for e in exitList:
                price, cell, path = self.calculateDistance(r,e)
                shortDist = r.getShortDistance()
                if price < currentDistance:
                    currentDistance = price
                    beste = e
            if price >= (3/2)*(shortDist):
                p = beste.getPoint()
                r.setTargetExit(p)
                r.setShortDistance(currentDistance)
                r.setPath(path)
            # check if you can move twice
            #        x  y    x  y
            # ex -> (1, 0), (2, 0), (3, 0) ok (right, right)
            # ex2 -> (0,0)  (1,0), (1,1) not ok (right, up)
            # make sure you navigate through the line segment in the speed of two
            # check if no collision occurs in speed of two
            # if there are collisions
            # try speed of 1
            # if there are collisions
            # try stopping
            # check if no collision
        while len(robotList) != 0:
            #simulate cycle
            print(f"at cycle {self.timecycle}: ")
            self.g.printGrid()
            self.notifyAll()
            self.incrementTimer()
            time.sleep(2)
        self.g.printGrid()
                
    def getShortestPath(self, robot):
        #using BFS to find the shortest path (also must return the price, and assign targetExit to robot)
        def bfs(robot):
            robotPoint = robot.getPoint()
            robotY, robotX = robotPoint[0],robotPoint[1]
            queue = []#add initial states
            if robotY+1 < self.g.rows:
                queue.append((self.g.cells[robotY+1][robotX].getCost(), self.g.cells[robotY+1][robotX], []))
            if robotY-1 >= 0:
                queue.append((self.g.cells[robotY-1][robotX].getCost(), self.g.cells[robotY-1][robotX], []))
            if robotX-1 >= 0:
                queue.append((self.g.cells[robotY][robotX-1].getCost(), self.g.cells[robotY][robotX-1], []))
            if robotX+1 < self.g.cols:
                queue.append((self.g.cells[robotY][robotX+1].getCost(), self.g.cells[robotY][robotX+1], [])) 
            visited = []
            while queue:
                print("searching BFS...")
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
                    if neighbor[1] not in visited:
                        visited.append(neighbor[1])
                        queue.append((cost+neighbor[0], neighbor[1], path + [u]))
            return None
        price, cell, path = bfs(robot)
        return (price, cell, path)

    def calculateDistance(self,robot, exit):
        def dsa(robot):
            robotPoint = robot.getPoint()
            robotY, robotX = robotPoint[0],robotPoint[1]
            queue = []#add initial states
            if robotY+1 < self.g.rows:
                heapq.heappush(queue, (self.g.cells[robotY+1][robotX].getCost(), self.g.cells[robotY+1][robotX], []))
            if robotY-1 >= 0:
                heapq.heappush(queue, (self.g.cells[robotY-1][robotX].getCost(), self.g.cells[robotY-1][robotX], []))
            if robotX-1 >= 0:
                heapq.heappush(queue, (self.g.cells[robotY][robotX-1].getCost(), self.g.cells[robotY][robotX-1], []))
            if robotX+1 < self.g.cols:
                heapq.heappush(queue, (self.g.cells[robotY][robotX+1].getCost(), self.g.cells[robotY][robotX+1], [])) 
            visited = []
            while queue:
               # print("Searching...")
                cost, u, path = heapq.heappop(queue)
                if u.getExit() != None and u.getExit() == exit:
                    print("Exit found")
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
                    if neighbor[1] not in visited:
                        visited.append(neighbor[1])
                        heapq.heappush(queue, (cost+neighbor[0], neighbor[1], path + [u]))
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
                    print(f" R{self.cells[i][j].getRobot().getId()}", end = "|")
                elif self.cells[i][j].getExit() is not None: 
                    print(" E ", end = "|")
                else:
                    print(f" {self.cells[i][j].getCost()} ",end = "|")
            print("\n|" + "---|"*(self.cols)) if i < (self.rows-1) else print()
        print("-" + "--"*(2*self.cols-1) + "--")

class Robot:
    globalid = 1
    direction = ['l','r','u','d']
    def __init__(self, point: tuple = (), shortDistance = 0, path = None):
        self.speed  = random.randint(1,2)
        self.direction = Robot.direction[random.randint(0,3)]
        self.currentPos = point #point will be made from Grid class
        self.targetExit = None #point
        self.shortDistance = shortDistance
        self.path = path
        self.id = Robot.globalid
        Robot.globalid+=1
    
    def setTargetExit(self, point):
        self.targetExit = point
        pass

    def getId(self):
        return self.id
    def determineSpeed(self):
        if len(self.path) >= 2:
            if self.path[1].getRobot() is None: 
                cell = self.path[1]
                movement = tuple(map(lambda x,y: x-y,cell.getPoint(),self.currentPos))
                if (movement[0] > 0 and movement[1] == 0) or (movement[0] == 0 and movement[1] > 0) or (movement[0] < 0 and movement[1] == 0) or (movement[0] == 0 and movement[1] < 0):
                    self.speed = 2
                else:
                    self.speed = 1
        else:
            self.speed = 1

    def determineDirection(self):
        if not self.path:
            pass
        else:
            cell = self.path[self.speed-1]
            movement = tuple(map(lambda x,y: x-y,cell.getPoint(),self.currentPos))
            if movement[0] == 0 and movement[1] > 0: 
                self.direction = 'r'
            elif movement[0] == 0 and movement[1] < 0:
                self.direction ='l'
            elif movement[0] < 0 and movement[1] == 0:
                self.direction = 'u'
            elif movement[0] > 0 and movement[1] == 0:
                self.direction = 'd'
    
    def popPath(self):
        del self.path[0:self.speed]

    def move(self):
            cell = self.path[self.speed-1]
            if cell.getRobot() is None:
                movement = tuple(map(lambda x,y: x-y,cell.getPoint(),self.currentPos))
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

    def getSpeed(self):
        return self.speed
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

    def __lt__(self, other):
        return self.cost < other.cost


a = Controller()
a.setupEnvironment()
a.runSimulation()