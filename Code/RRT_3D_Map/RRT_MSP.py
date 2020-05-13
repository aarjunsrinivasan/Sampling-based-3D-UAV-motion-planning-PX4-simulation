import cv2
import numpy as np
import time
import math
import random 
from sklearn.neighbors import KDTree
from mpl_toolkits import mplot3d
import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d import axes3d 
import matplotlib.pyplot as plt 
from matplotlib import style 
import copy
plt.ion()
# style.use('ggplot') 
length = 20
breadth = 15
height = 10
fig = plt.figure()
ax = plt.axes(projection='3d')

ax.set_xlim3d(0, 20)
ax.set_ylim3d(0,15)
ax.set_zlim3d(0,10)

c = 1



# To check if a point is in obstacle space or not
def obstacle_check(i,j,k):
    a = b = c = d = e = f = g = 0
    if(5-c <= i <=6+c) and (5-c <= j <= 6+c) and (0 <= k <= 10):
        a = 1
    elif (8-c <= i <=9+c) and (11-c <= j <= 12+c) and (0 <= k <= 10):
        b = 1
    elif (12-c <= i <=13+c) and (14-c <= j <= 15+c) and (0 <= k <= 10):
        c = 1
    elif (10-c <= i <11+c) and (2-c <= j <= 3+c) and (0 <= k <= 10):
        d = 1
    elif (16-c <= i <=17+c) and (7-c <= j <= 8+c) and (0 <= k <= 10):
        e = 1
    elif (3-c <= i <=4+c) and (10-c <= j <= 11+c) and (0 <= k <= 10):
        f = 1
    elif (17-c <= i <=18+c) and (3-c <= j <= 4+c) and (0 <= k <= 10):
        g = 1
    # elif (11-c <= i <=12+c) and (7-c <= j <= 8+c) and (0 <= k <= 10):
    #     h = 1

    if  ((a == 1) or (b == 1) or (c == 1) or (d == 1) or (e == 1) or (f == 1) or (g == 1) ):
        return True
    else:
        return False

# Plots the osbstacles in the 3D map
def obstacle_map():
    # defining x, y, z co-ordinates for bar position 
    x = [5,8,12,10,16,3,17] 
    y = [5,11,14,2,7,10,3] 
    z = np.zeros(7)

    # size of bars 
    dx = np.ones(7)              # length along x-axis 
    dy = np.ones(7)              # length along y-axs 
    dz = [10,10,10,10,10,10,10]   # height of bar 

    # setting color scheme 
    color = [] 
    for h in dz: 
        if h > 5: 
            color.append('b') 
        else: 
            color.append('b') 

    ax.bar3d(x, y, z, dx, dy, dz, color = color) 



def boundary_check(i, j, k):
    if (i < 0) or (j < 0) or (k < 0) or (i >= length) or (j >= breadth) or (k >= height):
        return True
    else:
        return False



def line_obstacle_check(j, i):
    k = (i[0] - j[0], i[1] - j[1], i[2] - j[2])
    k_mod = math.sqrt(k[0]**2 + k[1]**2 + k[2]**2)
    vec = (k[0]/k_mod, k[1]/k_mod, k[2]/k_mod)
    new_point = (j[0]+0.1*vec[0], j[1]+ 0.1*vec[1], j[2]+0.1*vec[2])
    return new_point

def cost2go(pt1, pt2):
    dist = math.sqrt((pt2[0] - pt1[0]) ** 2 + (pt2[1] - pt1[1]) ** 2 + (pt2[2] - pt1[2]) ** 2) 
    return dist


def generate_seed():
    x = round(random.uniform(0 , length)*2)/2
    y = round(random.uniform(0 , breadth)*2)/2
    z = round(random.uniform(0 , height)*2)/2
#     print(x,y,z)
    return (x,y,z)
    
def goalcheck_circle(x, y, z, goal_x, goal_y, goal_z):
    if ((x - goal_x) ** 2 + (y - goal_y) ** 2 + (z - goal_z) **2 <= (1 ** 2)):
        return True
    else:
        return False
    
def max_step_prop(j, i):
    k = (i[0] - j[0], i[1] - j[1], i[2] - j[2])
    k_mod = math.sqrt(k[0]**2 + k[1]**2 + k[2]**2)
    vec = (k[0]/k_mod, k[1]/k_mod, k[2]/k_mod)
    new_point = (j[0]+5*vec[0], j[1]+ 5*vec[1], j[2]+ 5*vec[2])
    return new_point

obstacle_map()
start = (0,0,0)
goal_x, goal_y, goal_z = (20,15,10)
goal = (goal_x, goal_y, goal_z)
visited_nodes = set()
all_nodes = []
parent_list = []
seed_list = []
parent_dict = {}

visited_nodes.add(start)
all_nodes.append(start)
parent_dict[start] = "okay"

seed = (0,0,0)
# print("visitednodes",visited_nodes)
# print("all_nodes", all_nodes)
print("\n")

solution = 0
while(goalcheck_circle(goal_x, goal_y, goal_z, seed[0], seed[1], seed[2]) == False):
    if solution == 1:
        break
    seed = generate_seed()
    # print("generated_seed", seed)
    if ((seed not in visited_nodes) and not obstacle_check(seed[0], seed[1], seed[2])):
        
        all_nodes.insert(0, seed)    
        X = np.array(all_nodes)
        tree = KDTree(X, leaf_size=2) 
        dist, ind = tree.query(X[:1], k=2)  
        p = ind[0][1]
        parent = all_nodes[p]
        
        
        if (cost2go(parent,seed) > 5):
            seed = max_step_prop(parent, seed)
            
            
        par = seed
        s = parent
        a = 0
        # print(s)
        while(cost2go(par,s)>=0.1):
            a = line_obstacle_check(s, par)
            # print(a)
            if obstacle_check(a[0], a[1], a[2]):
#                 print("inside")
#                 print("stop point", a)
                break
            s = a

        s = (round(s[0], 1), round(s[1], 1), round(s[2], 1))

        # s = seed
        if s not in visited_nodes:
            all_nodes[0] = s 
            visited_nodes.add(s)
            parent_dict[s] = parent 
            parent_list.append((parent[0], parent[1], parent[2]))
            seed_list.append((s[0], s[1], s[2]))
            ax.plot3D((parent[0],s[0]), (parent[1], s[1]), (parent[2], s[2]), 'black')
            # plt.show()
            # print("\n")

            # seed_check = s
            # goal_check = goal
            # temp1 = seed_check
            # temp2 = goal_check
            # if (seed_check != goal_check):

            #     while(cost2go(goal_check, seed_check)>=0.1):
            #         a = line_obstacle_check(seed_check, goal_check)
            #         if obstacle_check(a[0], a[1], a[2]):
            #             break
            #         seed_check = a
            #     goal_check = temp1

            # if (cost2go(seed_check, temp2)) < 0.2:
            #     print("termination")
            #     solution = 1
            #     all_nodes.insert(0, goal_check)
            #     visited_nodes.add(goal_check)
            #     parent_dict[temp2] = temp1 
            #     print("parent", temp1)
            #     print("goal", temp2)

            #     # goal_cost = cost_dict[s] + cost2go(seed_check, goal_check)
            #     # cost_dict[goal_check] = goal_cost
            #     parent = temp1
        else:
            all_nodes.pop(0)
        

print("Goal_Reached")
# print("dict", parent_dict)

path = []
path.append((s[0], s[1], s[2]))
# print("parent of goal", parent)
while parent != 'okay':
    temp = parent_dict.get(parent)
    path.append(parent)
#     print(path)
    parent = temp
    if parent == (start):
        break
path.append(start)
print("Backtracking done - shortest path found")

path = path[::-1]
# path.append(temp2)



x_path = [path[i][0] for i in range(len(path))]
y_path = [path[i][1] for i in range(len(path))]
z_path = [path[i][2] for i in range(len(path))]




# l = 0

# while l < len(seed_list):
#     ax.plot3D((parent_list[l][0], seed_list[l][0]), (parent_list[l][1], seed_list[l][1]), (parent_list[l][2], seed_list[l][2]),  'black')
#     l = l + 1
#     plt.show()
#     plt.pause(0.000000000000000000000000000000000001)




ax.plot3D(x_path, y_path, z_path, "-r")
print(path)
ax.set_xlabel('x-axis') 
ax.set_ylabel('y-axis') 
ax.set_zlabel('z-axis') 
plt.show()
plt.pause(5)
plt.savefig("output1.png")
plt.close()

