import numpy as np
import cv2
import matplotlib.pyplot as plt
from Priority_queue import PriorityQueue


# Creating the Map using opencv
def gen_map():
    
# Base Black image of size (450 , 250 )
    map = np.zeros((250,400,3), np.uint8)

# Circle of radius= 65
    cv2.circle(map,(300,65), 40, (0,255,255), -1)

# Hexagon
    pts = np.array([[200, 109], [234, 129], [234, 170], [200, 190], [165, 170], [165, 129]], np.int32)
    cv2.fillPoly(map,[pts],(0,255,255))


# Polygon
    pts = np.array([[36, 65], [115, 40], [80, 70], [105, 150]], np.int32)
    cv2.fillPoly(map,[pts],(0,255,255))
    
    return map
#to record the video of path planning visualisation
output= cv2.VideoWriter('result.mp4', cv2.VideoWriter_fourcc(*'mp4v'),10, (400, 250))

# We can also use the below function to generate our map
def generate_map():
    #Generating Map using the obstacle space

    map = np.zeros((250, 400, 3), np.uint8)


    for x in range(0, map.shape[1]):
     for y in range(0, map.shape[0]):
        if is_polygon(x,  y) or is_hexagon(x,  y) or is_circle(x, y):
            map[y][x] = [0, 130, 190]

    return map

#define all the obstacle shapes or spaces individually

def is_polygon(x, y):
    l1 = (0.316 * x + 173.608 - y) >= 0
    l2 = (0.857 * x + 111.429 - y) <= 0
    lm = (-0.114 * x + 189.091 - y) <= 0
    l3 = (-3.2 * x + 436 - y) >= 0
    l4 = (-1.232 * x + 229.348 - y) <= 0

    if (l1 and l2 and lm) or (l3 and l4 and not lm):
         return True
    else:
        return False



def is_hexagon(x, y):
    l1 = (-0.571 * x + 174.286 - y) <= 0
    l2 = (165 - x) <= 0
    l3 = (0.571 * x + 25.714 - y) >= 0
    l4 = (-0.571 * x + 254.286 - y) >= 0
    l5 = (235 - x) >= 0
    l6 = (0.571 * x - 54.286 - y) <= 0

    if l1 and l2 and l3 and l4 and l5 and l6:
        return True
    else:
        return False

def is_circle(x ,y):
    circ_eq = ((x - 300)**2 + (y - 185)**2 - 40*40) <= 0
    if circ_eq:
        return True
    else:
        return False

def polygon_space(x, y):
    l1 = (0.316 * x + 178.608 - y) >= 0
    l2 = (0.857 * x + 106.429 - y) <= 0
    lmid = (-0.114 * x + 189.091 - y) <= 0
    l3 = (-3.2 * x + 450 - y) >= 0
    l4 = (-1.232 * x + 220.348 - y) <= 0

    return ((l1 and l2 and lmid) or (l3 and l4 and not lmid)) 


def hexagon_space(x, y):
    l1 = (-0.575 * x + 169 - y) <= 0
    l2 = (160 - x) <= 0
    l3 = (0.575 * x + 31 - y) >= 0
    l4 = (-0.575 * x + 261 - y) >= 0
    l5 = (240 - x) >= 0
    l6 = (0.575 * x - 61 - y) <= 0

    return l1 and l2 and l3 and l4 and l5 and l6

def circle_space(x, y):
    circ = ((x - 300) ** 2 + (y - 185) ** 2 - 45 * 45) <= 0
    
    return circ

def is_obstacle(y,x):
    if  is_circle(x, y) or is_hexagon(x,y) or is_polygon(x,y):
        return True
    else: 
        return False

#clearance is added in this function
def check_obs_space(y_pos, x_pos):
    return polygon_space(x_pos, y_pos) or hexagon_space(x_pos, y_pos) or circle_space(x_pos, y_pos) \
        or (maze.shape[1] - 5 <= x_pos) or x_pos <= 4 or (maze.shape[0] - 5 <= y_pos) or y_pos <= 4



# This finds the cost between the nodes
def cost(a, b):
    x1, y1 = a
    x2, y2 = b
    return abs(x1 - x2) + abs(y1 - y2)


# This represent all the actions needed in a 8-action space

actions={
    "left": (0,-1,1),
    "right":(0,1,1),                                                                                                                  
    "up":(-1,0,1), 
    "down":(1,0, 1),
    "up-left":(-1,-1,1.4),
    "up-right":(-1,1,1.4),
    "down-left":(1,-1, 1.4),
    "down-right":(1,1,1.4)
}

# This function is used to check whether a position is a legal position or not 
# considering the obstacle space and checking the tolerance of the robot with respect 
# to the obstacle space
def is_viable(map, pos):
    i, j = pos
    num_rows = len(map)
    num_cols = len(map[0])
    return 0 <= i < num_rows and 0 <= j < num_cols and not is_obstacle(i,j) and not check_obs_space(i,j)


#[dict_path] - This dictionary contains all the explored nodes in the map used.
#backtracking to find path
def get_path(dict_path, start, goal):
    current = goal
    path = []
    while current != start:
        path.append(current)
        current = dict_path[current]
    path.append(start)
    path.reverse()
    #print(dict_path)
    return path,dict_path


#dijkstra algorithm implementation
def dijkstra(map, start, goal):
    
    pq = PriorityQueue() # Empty priority queue inititalisation
    pq.put(start, 0) # put start node into queue
    predecessors = {start: None} # Closed List
    cost_to_come = {start: 0} # To store the cost to come with Node info

    while not pq.is_empty(): # Checking all the explored nodes
        current_cell = pq.get() # Popping the element based upon priority(cost)
        if current_cell == goal: # if this is goal we return the shortest path
            return get_path(predecessors, start, goal)
        for direction in actions:
            row_offset= actions[direction][0]
            col_offset = actions[direction][1]
             # We check for all the direction in the path using the above for loop
            neighbour = (current_cell[0] + row_offset, current_cell[1] + col_offset)
            if is_viable(map, neighbour) and neighbour not in cost_to_come:
                
                # if true enters the following loop, where cost is added to node based on which it is arranged in the priority queue
                
                new_cost = cost_to_come[current_cell] + actions[direction][2]
                cost_to_come[neighbour] = new_cost
                
                priority_wt = new_cost + cost(current_cell, neighbour)
                pq.put(neighbour, priority_wt)
                predecessors[neighbour] = current_cell          
    return None


if __name__ == "__main__":
    
    
    global maze
    maze = generate_map()
    fig, ax=plt.subplots()
    ax.imshow(maze[::-1,:,:])

    # select point from the given map output

    start,goal = plt.ginput(2)
    
    start=(249- int(start[1]),int(start[0]))
    goal=(249 - int(goal[1]),int(goal[0]))

    print("You have selected the start position as:",start)
    print("You have selected the goal position as:",goal)

    
    if is_obstacle(goal[0],goal[1]) or is_obstacle(start[0],start[1]) or check_obs_space(start[0],start[1]): 
         print("Please enter different start_pos and goal_pos away from the obstacle space")
    else:
        result,predec = dijkstra(maze, start, goal)
        for key in predec:
            if not is_obstacle(key[0],key[1]):
                maze[key[0]][key[1]] = [255,255,255]
                cv2.imshow('map',maze[::-1,:,:])
                output.write(maze[::-1,:,:])
                keyCode = cv2.waitKey(1)
                if (keyCode & 0xFF) == ord("q"):
                 cv2.destroyAllWindows()
                 break
        for i in result:
            maze[i[0]][i[1]] = [255,0,255]
            cv2.imshow('map',maze[::-1,:,:])
            output.write(maze[::-1,:,:])
            keyCode = cv2.waitKey(1)
            if (keyCode & 0xFF) == ord("q"):
                 cv2.destroyAllWindows()
                 break
        output.release()
        #imageio.mimsave(res, result,fps=60)
          
    





