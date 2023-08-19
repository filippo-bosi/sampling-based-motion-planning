import random
import math
import pygame

class RRTMap:
    def __init__(self, start, goal, MapDimensions, obsdim, obsnum):
        self.start = start
        self.goal = goal
        self.MapDimensions = MapDimensions
        self.Maph, self.Mapw = self.MapDimensions

        # window settings
        self.MapWindowName = '1.2 - Sampling-based algorithm with domain knowledge'
        pygame.display.set_caption(self.MapWindowName)
        self.map = pygame.display.set_mode((self.Mapw, self.Maph))
        self.map.fill((255, 255, 255))
        self.nodeRad = 2
        self.edgeThickness = 1

        self.obstacles = []
        self.obsdim = obsdim
        self.obsNumber = obsnum

        # Colors definition
        self.Red = (255, 0, 0)
        self.Green = (0, 255, 0)
        self.Blue = (0, 0, 255)
        self.grey = (50, 50, 50)
        self.white = (255, 255, 255)
        self.black = (0, 0, 0)

    def drawMap(self, obstacles):
        pygame.draw.circle(self.map, self.Green, self.start, self.nodeRad * 3, 0)
        pygame.draw.circle(self.map, self.Green, self.goal, self.nodeRad * 10, 1)
        self.drawObs(obstacles)

    def drawObs(self, obstacles):
        obstaclesList = obstacles.copy()
        while (len(obstaclesList) > 0):
            obstacle = obstaclesList.pop(0)
            pygame.draw.rect(self.map, self.grey, obstacle)

    def drawPath(self, path):
        for currNode, nxtNode in zip(path, path[1:]):
            pygame.draw.line(self.map, self.Red, currNode, nxtNode, self.edgeThickness*2)
class RRTGraph:
    def __init__(self, start, goal, dmax, MapDimensions, obsdim, obsnum):
        (x, y) = start
        self.start = start
        self.goal = goal
        self.dmax = dmax
        self.goalFlag = False
        self.maph, self.mapw = MapDimensions
        self.x = []
        self.y = []
        self.parent = []
        # initialize the tree with initial node
        self.x.append(x)
        self.y.append(y)
        self.parent.append(0)

        # the obstacles
        self.obstacles = []
        self.obsDim = obsdim
        self.obsNum = obsnum
        # path
        self.goalstate = goal
        self.path = []

    def makeRandomRect(self):
        uppercornerx = int(random.uniform(0, self.mapw - self.obsDim))
        uppercornery = int(random.uniform(0, self.maph - self.obsDim))
        return (uppercornerx, uppercornery)

    def makeStaticObs(self):
        obs = []
        with open('obstacles_list.txt', 'r') as input:
            for line in input:
                # remove linebreak from a current name
                # linebreak is the last character of each line
                coord = line[:-1]
                upperx = int(coord.split(',')[0][1:])
                uppery = int(coord.split(',')[1][1:].replace(')',''))
                upper = (upperx, uppery)
                rectang = pygame.Rect(upper, (self.obsDim, self.obsDim))

                # add current item to the list
                obs.append(rectang)
            print("Done loading obstacles")
        self.obstacles = obs.copy()
        return obs

    def makeRandomObs(self):
        obs = []
        upperCorners = []
        for i in range(0, self.obsNum):
            rectang = None
            startgoalcol = True
            while startgoalcol:
                upper = self.makeRandomRect()
                rectang = pygame.Rect(upper, (self.obsDim, self.obsDim))
                if rectang.collidepoint(self.start) or rectang.collidepoint(self.goal):
                    startgoalcol = True
                else:
                    startgoalcol = False
            upperCorners.append(upper)
            obs.append(rectang)
        # Save rectangles upper corners to file to store the map
        with open('obstacles_list.txt', 'w') as output:
            for item in upperCorners:
                output.write(f'{item}\n')
            print('Done saving obstacles')
        self.obstacles = obs.copy()
        return obs

    def add_node(self, n, x, y):
        self.x.insert(n, x)
        self.y.append(y)

    def remove_node(self, n):
        self.x.pop(n)
        self.y.pop(n)

    def add_edge(self, parent, child):
        self.parent.insert(child, parent)

    def remove_edge(self, n):
        self.parent.pop(n)
    def number_of_nodes(self):
        return len(self.x)

    def distance(self, n1, n2):
        (x1, y1) = (self.x[n1], self.y[n1])
        (x2, y2) = (self.x[n2], self.y[n2])
        px = (float(x1) - float(x2)) ** 2
        py = (float(y1) - float(y2)) ** 2
        return (px + py) ** (0.5)

    def nearest(self, n):
        dmin = self.distance(0, n)
        nnear = 0
        for i in range(0, n):
            if self.distance(i, n) < dmin:
                dmin = self.distance(i, n)
                nnear = i
        return nnear
    def sample_xrand(self):
        # Sample a random configuration in the free space
        n = self.number_of_nodes()
        x = int(random.uniform(0, self.mapw))
        y = int(random.uniform(0, self.maph))
        self.add_node(n, x, y)
        if self.isFree():
            return x, y
        else:
            return self.sample_xrand()

    def sample_xtree_heuristic(self):
        n = self.number_of_nodes()
        # Add goal to the list of nodes
        self.add_node(n, self.goal[0], self.goal[1])
        # Pick the node closer to the goal
        xtree = self.nearest(n)
        # Remove goal from the list of nodes
        self.remove_node(n)
        return xtree

    def isFree(self):
        n = self.number_of_nodes() - 1
        (x, y) = (self.x[n], self.y[n])
        obs = self.obstacles.copy()
        while len(obs) > 0:
            rectang = obs.pop(0)
            rectang = rectang.scale_by(1.2)
            if rectang.collidepoint(x, y):
                self.remove_node(n)
                return False
        return True

    def connect(self, n1, n2, STEP_SIZE=5):
        (x1, y1) = (self.x[n1], self.y[n1])
        (x2, y2) = (self.x[n2], self.y[n2])
        d = self.distance(n1, n2)
        if d > STEP_SIZE:
            segments = int(d/STEP_SIZE)
            nodes = self.interpolate(n1, n2, segments)
            self.remove_node(n2)
            n = self.number_of_nodes()
            for j in range(0,len(nodes)-1):
                self.add_node(n+j, nodes[j][0], nodes[j][1])
                if self.isFree():
                    if j==0:
                        self.add_edge(n1, n+j)
                    else:
                        self.add_edge(n+j-1, n + j)
                    if abs(nodes[j][0] - self.goal[0]) <= self.dmax and \
                            abs(nodes[j][1] - self.goal[1]) <= self.dmax:
                        self.goalstate = n + j + 1
                        self.add_node(self.goalstate, self.goal[0], self.goal[1])
                        self.connect(n+j, self.goalstate)
                        self.goalFlag = True
                        return True
                else:
                    return False
            return True
        else:
            self.add_edge(n1, n2)
        return True

    def interpolate(self, n1, n2, segments):
        segments= segments+1
        (x1, y1) = (self.x[n1], self.y[n1])
        (x2, y2) = (self.x[n2], self.y[n2])
        x_delta = (x2 - x1) / segments
        y_delta = (y2 - y1) / segments
        points = []
        for i in range(1, segments):
            points.append([x1 + i * x_delta, y1 + i * y_delta])
        points.append([x2, y2])
        return points

    def extend(self):
        xtree = self.sample_xtree_heuristic()
        n = self.number_of_nodes()
        x, y = self.sample_xrand()
        self.connect(xtree, n)
        return self.x, self.y, self.parent

    def path_to_goal(self):
        if self.goalFlag:
            self.path = []
            self.path.append(self.goalstate)
            newpos = self.parent[self.goalstate]
            while (newpos != 0):
                self.path.append(newpos)
                newpos = self.parent[newpos]
            self.path.append(0)
        return self.goalFlag

    def getPathCoords(self):
        pathCoords = []
        for node in self.path:
            x, y = (self.x[node], self.y[node])
            pathCoords.append((x, y))
        return pathCoords

    def cost(self, n):
        ninit = 0
        n = n
        parent = self.parent[n]
        c = 0
        while n is not ninit:
            c = c + self.distance(n, parent)
            n = parent
            if n is not ninit:
                parent = self.parent[n]
        return c