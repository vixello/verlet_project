import math

import pygame
import numpy
width = 600
height = 1000
display = pygame.display.set_mode((width , height))
g = 9.81
friction = 0.95

LEFT = 1
RIGHT = 3
pickedParticle = None

eventt = pygame.USEREVENT + 1
pygame.time.set_timer(eventt, 1500)

dt = 0.01


class Shape:
    def __init__(self, width, height, point):
        self.width = width
        self.height = height
        self.point = point

    def distance(self, point1, point2):

        return (
                       (point1["coordinations"][0] - point2["coordinations"][0]) ** 2 +
                       (point1["coordinations"][1] - point2["coordinations"][1]) ** 2
               ) ** 0.5

    def shape(self, points, joints, pinned=[]):
        # a list containing the distances between the points  that we will be adding to the shape
        lengths = [
            self.distance(points[j[0]], points[j[1]]) for j in joints
            # distance(points[ [j[i][0]], points[j[i][1]] ]) i in range{len(joints)
        ]

        return {
            "pinned": pinned,
            "points": points,
            "joints": joints,
            "lengths": lengths
        }

    def moveTheShape(self, shape):
        for p in shape["points"]:

            if not p in shape["pinned"]:
                # else:
                if pygame.mouse.get_pressed()[0]:
                    # if pickedParticle == True:
                    pos = pygame.mouse.get_pos()
                    # for i in range(len(shape["points"]) - 1):
                    deltax = p["coordinations"][0] - pos[0]
                    deltay = p["coordinations"][1] - pos[1]
                    deltalengthx = numpy.linalg.norm(deltax)
                    deltalengthy = numpy.linalg.norm(deltay)
                    if shape == b and deltalengthx < 30 and deltalengthy < 30:
                        p["coordinations"] = [pos[0], pos[1]]
                    elif shape != b and deltalengthx < 60 and deltalengthy < 400:
                        p["coordinations"] = [pos[0], pos[1]]

                # shape["points"][i]["coordinations"] = [pos[0], pos[1]]
                if shape != b:
                    self.point.movepoint(p,g)
                    if pygame.mouse.get_pressed()[1]:
                        p["coordinations"] = shape["points"][len(shape["points"] / 2)]["coordinations"]
                else:
                    self.point.movepoint(p,g*dt*0.5)
                    p["r"] = 20
                    self.point.simulatingPoint(p,(255,100,100))

    def constrainningTheShape(self, shape):
        # for every joint or segment of shape
        for i in range(len(shape["joints"])):
            length = shape["lengths"][i]
            # every length corresponds to a joint
            # first point of 0

            point1 = shape["points"][shape["joints"][i][0]]
            point2 = shape["points"][shape["joints"][i][1]]
            distan = self.distance(point1, point2)
            dx = point1["coordinations"][0] - point2["coordinations"][0]
            dy = point1["coordinations"][1] - point2["coordinations"][1]

            updatex = 0.5 * dx * (length - distan) / (distan + 0.001)
            updatey = 0.5 * dy * (length - distan) / (distan + 0.001)

            if not (point1 in shape["pinned"] or point2 in shape["pinned"]):
                point1["coordinations"][0] += updatex
                point1["coordinations"][1] += updatey
                point2["coordinations"][0] -= updatex
                point2["coordinations"][1] -= updatey

            # poin1 not pinned, poin2 pinned
            if not point1 in shape["pinned"] and point2 in shape["pinned"]:
                point1["coordinations"][0] += 2 * updatex
                point1["coordinations"][1] += 2 * updatey

            if point1 in shape["pinned"] and point2 not in shape["pinned"]:
                point2["coordinations"][0] -= 2 * updatex
                point2["coordinations"][1] -= 2 * updatey

    def collideTheShape(self, shape, width, height):
        for p in shape["points"]:
            self.point.collidePoint(p, width, height)

    def renderTheShape(self, shape, color=(50, 100, 200), thickness=3):
        for j in shape["joints"]:
            starPosition = shape["points"][j[0]]["coordinations"][0], shape["points"][j[0]]["coordinations"][1]
            endPosition = shape["points"][j[1]]["coordinations"][0], shape["points"][j[1]]["coordinations"][1]

            pygame.draw.line(display, color, starPosition, endPosition, thickness)

    def simulateTheShape(self, shape,color, width=600, height=500):

        self.moveTheShape(shape)
        for i in range(30):
            self.constrainningTheShape(shape)
            self.collideTheShape(shape, self.width, self.height)

        self.renderTheShape(shape, color)

    def box(self, x=100, y=10, length=100, r=3, color=(200, 200, 200)):
        joints = [[0, 1], [1, 2], [2, 3], [3, 0], [1, 3], [2, 0]]
        points = [self.point.pointp(x, y, x - 10, y - 10, r),
                  self.point.pointp(x + length, y, x + length - 10, y - 10, r),
                  self.point.pointp(x + length, y + length, x + length - 10, y + length - 10, r),
                  self.point.pointp(x, y + length, x - 10, y + length - 10, r)]

        return self.shape(points, joints)

    def fabric(self, x=100, y=100, numOfPointsHor=10, numOfPointsVer=3, hole=20, r=3,
               color=(100, 222, 50)):
        joints = []
        # grid of points
        # for each of row
        # jump the distance of the hole , create another point, repeat till we have enough points horizontally
        points = [self.point.pointp(x + i * hole, y + j * hole, x + i * hole, y + j * hole, r) for j in
                  range(numOfPointsVer) for i
                  in range(numOfPointsHor)]

        for i in range(len(points)):
            # exclude the last points so the lines dont go over them
            if i % numOfPointsHor != numOfPointsHor - 1:
                joints.append([i, i + 1])

        for i in range(len(points) - numOfPointsHor):
            joints.append([i, i + numOfPointsHor])

        pinned = [points[0], points[numOfPointsHor - 1]]

        return self.shape(points, joints, pinned=pinned)


class Pointt:
    def __init__(self, width=600, height=500):
        self.width = width
        self.height = height
        self.vx = 0
        self.vy = 0
    def pointp(self, x, y, oldx, oldy, r):
        return {
            "coordinations": [x, y],
            "old_coordinations": [oldx, oldy],
            "r": r}

    def collidePoint(self, p, width, height):
        # recompute velocities
        coordinations = p["coordinations"]
        oldCoordinations = p["old_coordinations"]

        self.vx = coordinations[0] - oldCoordinations[0]
        self.vy = coordinations[1] - oldCoordinations[1]
        v = [self.vx, self.vy]

        width -= p["r"]
        height -= p["r"]

        if coordinations[0] > width:
            p["coordinations"][0] = width
            p["old_coordinations"][0] = p["coordinations"][0] + self.vx
        if coordinations[0] < p["r"]:
            p["coordinations"][0] = p["r"]
            p["old_coordinations"][0] = p["coordinations"][0] + self.vx
        if coordinations[1] > height:
            p["coordinations"][1] = height
            p["old_coordinations"][1] = p["coordinations"][1] + self.vy
        if coordinations[1] < p["r"]:
            p["coordinations"][1] = p["r"]
            p["old_coordinations"][1] = p["coordinations"][1] + self.vy


    def movepoint(self, p,g):
        #xn+1 = 2xn - xn-1
        #xn+1 = xn +dt^2F/m


        coordin = p["coordinations"]
        oldCoordinations = p["old_coordinations"]

        self.vx = coordin[0] - oldCoordinations[0]
        self.vy = coordin[1] - oldCoordinations[1]

        v = [self.vx, self.vy]

        # force
        v[1] = (v[1] + g) * friction

        v[0] *= friction
        # update the new position
        p["old_coordinations"] = coordin

        # update the current position with velocity vector to get the next position
        p["coordinations"] = [coordin[0] + v[0] +dt*dt/0.1, coordin[1] + v[1]+dt*dt/0.1]

    def rendering(self, p,color = (200, 255, 200)):
        center = p["coordinations"][0], p["coordinations"][1]
        radius = p["r"]

        pygame.draw.circle(display, color, center, radius)

    def simulatingPoint(self, p, color,width=600, height=500):

        self.rendering(p,color)


class Ball:
    def __init__(self, shape, width=600, height=500):
        self.width = width
        self.height = height
        self.shape = shape
        self.box = b
        self.vx = None
        self.vy = None
        self.coordinations = None
        self.oldCoordinations = None
        self.time_elapsed_since_last_action = 0
        self.angle = 0
        self.s =None

    def pointB(self, x, y, newx, newy, r):
        return {
            "coordinations": [x, y],
            "old_coordinations": [newx, newy],
            "r": r}

    def distance(self, point1, point2):

        return (
                       (point1["coordinations"][0] - point2["coordinations"][0]) ** 2 +
                       (point1["coordinations"][1] - point2["coordinations"][1]) ** 2
               ) ** 0.5

    def collideBall(self, p, width, height, clock):
        # recompute velocities
        self.coordinations = p["coordinations"]
        self.oldCoordinations = p["old_coordinations"]
        self.distan = 0
        self.vx = self.coordinations[0] - self.oldCoordinations[0]
        self.vy = self.coordinations[1] - self.oldCoordinations[1]
        v = [self.vx, self.vy]

        width -= p["r"]
        height -= p["r"]
        for bb in self.box["points"]:
            if self.distance(p, bb) < p["r"]+20:
                bb["coordinations"][1] = bb["coordinations"][1] - 30*friction
                bb["old_coordinations"][1] = bb["coordinations"][1] - self.vy
                p["coordinations"][1] += p["r"]
                self.vy *=-1

        if self.coordinations[0] > width:
            p["coordinations"][0] = width
            p["old_coordinations"][0] = p["coordinations"][0] + self.vx
        if self.coordinations[0] < p["r"]:
            p["coordinations"][0] = 0
            p["old_coordinations"][0] = p["coordinations"][0] + self.vx
        if self.coordinations[1] > height:
            p["coordinations"][1] = height
            p["old_coordinations"][1] = p["coordinations"][1] + self.vy
        if self.coordinations[1] < p["r"]:
            p["coordinations"][1] = p["r"]
            p["old_coordinations"][1] = p["coordinations"][1] + self.vy

        for s in self.shape["points"]:
            # and s["coordinations"][0] - p["coordinations"][0] < p["r"] / 2.1

            if self.distance(p, s) < p["r"]-1:
                self.s = s

                # print(s["coordinations"][0],"    ",s["old_coordinations"][0], "      ",self.distan)
                # print(s["coordinations"][1],"    ",s["old_coordinations"][1])

                p["coordinations"][1] = s["coordinations"][1] - p["r"]
                p["old_coordinations"][1] = p["coordinations"][1] + self.vy

                for i in range(0, len(self.shape["points"])):
                    T = -20 * 30 * (s["old_coordinations"][1] - s["coordinations"][1])

                    if self.shape["points"][i] == s:

                        if self.shape["points"][i - 1]["coordinations"][1] >= s["coordinations"][1]:

                            p["old_coordinations"][0] = p["coordinations"][0] + 0.008 * self.shape["points"][i - 1]["coordinations"][0]

                        if self.shape["points"][i - 1]["coordinations"][1] < s["coordinations"][1]:
                            p["old_coordinations"][0] = p["coordinations"][0] - 0.008 * self.shape["points"][i - 1]["coordinations"][0]

            # - (s["old_coordinations"][1] - s["coordinations"][1])
            # p["coordinations"][1] = s["coordinations"][1] - (s["old_coordinations"][1] - s["coordinations"][1]) * friction + self.vy * math.sin(math.degrees(self.angle))


            # if pygame.event.get(eventt):
            #     p["coordinations"] = self.shape["points"][int(len(self.shape["points"]) / 2)]["coordinations"]
            #     p["old_coordinations"] = p["coordinations"]


    def moveBall(self, p,mouseup, mouseupr):
        self.coordinations = p["coordinations"]
        self.oldCoordinations = p["old_coordinations"]
        self.mass = 2
        self.vx = self.coordinations[0] - self.oldCoordinations[0]
        self.vy = self.coordinations[1] - self.oldCoordinations[1]
        v = [self.vx, self.vy]

        # force
        v[1] = (v[1] + g/self.mass) * friction

        v[0] *= friction
        # update the old position
        p["old_coordinations"] = self.coordinations

        # update the current position
        p["coordinations"] = [self.coordinations[0] + v[0], self.coordinations[1] + v[1]]

        #reset position with rightclick
        if mouseupr:
            p["coordinations"] = self.shape["points"][int(len(self.shape["points"]) / 2)]["coordinations"]
            p["old_coordinations"] = p["coordinations"]

        if mouseup:
            self.distan = (
                                  (self.s["coordinations"][0] - self.s["old_coordinations"][0]) ** 2 +
                                  (self.s["coordinations"][1] - self.s["old_coordinations"][1]) ** 2
                          ) ** 0.5

            self.angle = math.atan2(self.s["coordinations"][1] - self.s["old_coordinations"][1],
                                    self.s["coordinations"][0] - self.s["old_coordinations"][0]) * 180 / math.pi

            # print(self.s["old_coordinations"][1])
            # print(self.angle )

            #after pulling, release ball with force
            p["coordinations"][1] = self.s["coordinations"][1] - (self.distan) + self.vy * math.sin(math.degrees(self.angle))
            p["coordinations"][0] = self.s["coordinations"][0] + self.vx * 10 * math.cos(math.degrees(self.angle))


    def rendering(self, p):
        color = (200, 255, 200)
        center = p["coordinations"][0], p["coordinations"][1]
        radius = p["r"]

        pygame.draw.circle(display, color, center, radius)

    def simulatingBall(self, p, clock, mouseup, mouseupr):

        # if pygame.mouse.get_pressed()[0]:
        #     pos = pygame.mouse.get_pos()
        #     p["coordinations"] = [pos[0],pos[1]]
        self.moveBall(p, mouseup, mouseupr)
        self.collideBall(p, self.width, self.height, clock)
        self.rendering(p)


p = Pointt(width, height)
# pp = p.pointp(100, 100, 95, 95, 10)

shape1 = Shape(width, height, p)
b = shape1.box()
fabricshape = shape1.fabric(100, 600, numOfPointsHor=21)

ball = Ball(fabricshape, width, height)
balll = ball.pointB(120, 100, 95, 95, 15)


# fabricshape = fabric(numOfPointsHor=30)

def run():
    clock = pygame.time.Clock()
    run = True
    j = 0
    mouseup = False
    mouseupR = False
    global pickedParticle, mousePosition
    while run:
        display.fill((25, 25, 25))

        for event in pygame.event.get():
            if event.type == pygame.QUIT:
                run = False
            if event.type == pygame.MOUSEBUTTONUP and event.button == LEFT:
                mouseup = True
            if event.type == pygame.MOUSEBUTTONUP and event.button == RIGHT:
                mouseupR = True

        if pygame.mouse.get_pressed()[1]:
            print("hello")
        shape1.simulateTheShape(fabricshape,(50, 100, 200), width, height)
        shape1.simulateTheShape(b,(200, 100, 200),width, height)
        ball.simulatingBall(balll, clock, mouseup, mouseupR)
        # update the display surface
        mouseup = False
        mouseupR = False
        pygame.display.update()
        clock.tick(40)
    pygame.quit()


run()
