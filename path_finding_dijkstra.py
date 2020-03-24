import math    #euclidian distance er rootover r manhatan er abs er jonno
import random
import time
import sys
import queue
from queue import PriorityQueue    #priorityQueue dijkster er jonno use korsi

# Size of grid N

N = int(input("Size of the Grid: "))     #grid input nisi row colum same
grid = {}
for i in range(0, N):      # row i 0 thek n
    for j in range(0, N):    #colum j 0 theke n projonto
        grid[i, j] = '-'     #intially sob -empty cell kore rakhsi
'''
for i in range(0, N):
    for j in range(0, N):
        print(grid[i, j], end=" ")
    print("\n")
'''

class Point:   #point use kortese  point e object akare thaktese... x,y er ak sathe point
    ''' Represents a point
    '''

    def __init__(self, x=0, y=0):
        self.x = x
        self.y = y

    def __eq__(self, other):
        return self.x == other.x and self.y == other.y

    def __hash__(self):
        return hash((self.x, self.y))

    def __str__(self):
        return '({0}, {1})'.format(self.x, self.y)


def is_obstacle(point):           #obstcal er function
    return grid[point.x, point.y] == '#'  #obstcal gula k # die define kore disi
   #point.x point.y die check krotese ami  obstcles kina
   #obsstcales check kortese
   #define theke def


def cost(src, dest):
    euclidean_distance = math.sqrt((src.x - dest.x) ** 2 + (src.y - dest.y) ** 2)

    # source and destination er value pathisi then eulcadian distancer ber kortese
    return euclidean_distance


def _adjacent_points(point):
    ''' given a point, finds all adjacent points that are not obstacles
    '''
    adjacent_points = []
    # asea pase jara ase tader jonno nissi. left right up down, all corner
    # can take a step into either directions
    # can take a step into either directions
    for x in range(-1, 2):  ## -1 <- x -> +1
        # 0 <= adj_x <= self.max_x - 1
        adj_x = min(max(point.x + x, 0), N - 1)
        # x row cover
        for y in range(-1, 2):  ## -1 <- y -> +1
            # 0 <= adj_y <= self.max_y - 1
            adj_y = min(max(point.y + y, 0), N - 1)
             #y cover column
            if adj_x == point.x and adj_y == point.y:
                continue     #nijer ghor hoile continue korbo, list e rakhbo na
            adjacent_point = Point(adj_x, adj_y)   #akta new point banaisi x,y, ei x,y paisi
            if is_obstacle(adjacent_point):
                continue     #obscale hoile nibo na list na..just contitune kore jabo
            # all checks passed
            adjacent_points.append(adjacent_point)    #sob thik thakle list e rakhlam ami
    return adjacent_points                   #return kore dilam list ta



Count = 0

no_of_obstacles = int(input("No of Obstacles: "))     #obstcal er no nisi
for i in range(no_of_obstacles):                       # i theke range no obstacles projont
    x = int(input("X of obstacle no " + str(i) + ": "))   # obstcaler  x point
    y = int(input("Y of obstacle no " + str(i) + ": "))     #obstacler y point
    # print(x)
    # print(y)
    grid[x, y] = '#'   #now obstcal e hash die obscal set korlam jei point nelam
    # print(grid)
robot_x = int(input("X of robot: "))   #robot er x position
robot_y = int(input("Y of robot: "))     #robot er y position
robot_point = Point(robot_x, robot_y)    # x y er position class point e pathay dilam statri
grid[robot_x, robot_y] = 's'              #start location s die define korlam
target_x = int(input("X of target: "))    # target er x position
target_y = int(input("Y of target: "))     #target er y position
grid[target_x, target_y] = 't'            #target location t die define korsi
target_point = Point(target_x, target_y)   #targer er x position y position pont class e patha

#piority queue e valure rakha jai.. object rakhar jonno eita ki edit korsi
class MyPriorityQueue(PriorityQueue):
    def __init__(self):
        PriorityQueue.__init__(self)
        self.counter = 0

    def put(self, item, priority):
        PriorityQueue.put(self, (priority, self.counter, item))
        self.counter += 1

        # priority hoilo value , object
        # object store korar jonno nisi

    def get(self, *args, **kwargs):
        _, _, item = PriorityQueue.get(self, *args, **kwargs)
        return item


frontier = MyPriorityQueue()     #froniter varible re priorityqueue banainielam
frontier.put(robot_point, 0)      #proirty ke te intially starting point rakhlaam... , 0 die p
came_from = {}
cost_so_far = {}
came_from[robot_point] = None
cost_so_far[robot_point] = 0
last = None

while not frontier.empty():   #queue khali na hoa pojonto ami loop chalassi
    current = frontier.get()    #top ta re nilam..robot nisi..pore chnage hobe..je thakbe top

    if current.x == target_point.x and current.y == target_point.y:
        break                    #check koralm she amer target kina
    Count += 1
    for next in _adjacent_points(current):    #or ase pase k k ache
        new_cost = cost_so_far[current] + cost(current, next)   #new cost ber korlam node pluse edge er cost
        if next not in cost_so_far or new_cost < cost_so_far[next]:  #next aisi already count korsi kina and notun cost ag
            cost_so_far[next] = new_cost  #choto tai new cost set kore dilam
            priority = new_cost
            frontier.put(next, priority)   #list e rkhalm, new priority thik korlam
            came_from[next] = current         #jiekhan theke asise notun node e root e save rakhlam jata pore path paite subidha hoi


last_element = None
for node in came_from:
    last_element = node   #target er object ta nilam.. save silo came from e

current = last_element
while came_from[current] is not None:  #came form kar root e  k. last theke sobar path pabo r t
    # print(current.x, end=" ")
    # print(current.y)
    if current is not last_element:
        grid[current.x, current.y] = 'p'
    current = came_from[current]    #backtracking
    #porer current pichone jacche
'''

for node in came_from:
    print(node.x, end=" ")
    print(node.y)
    # grid[node.x, node.y] = 'p'
'''
for i in range(0, N):
    for j in range(0, N):
        print(grid[i, j], end=" ")
    print("\n")
#full grid print korsi