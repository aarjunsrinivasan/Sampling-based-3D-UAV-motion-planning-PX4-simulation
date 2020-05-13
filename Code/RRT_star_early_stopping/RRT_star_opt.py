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
from scipy import spatial
import copy
from queue import PriorityQueue 
q = PriorityQueue() 
plt.ion()
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


# Boundary check to avoid exploration outside the map space
def boundary_check(i, j, k):
    if (i < 0) or (j < 0) or (k < 0) or (i >= length) or (j >= breadth) or (k >= height):
        return True
    else:
        return False


# Returms the point 0.1m away from parent to child
def line_obstacle_check(j, i):
    k = (i[0] - j[0], i[1] - j[1], i[2] - j[2])
    k_mod = math.sqrt(k[0]**2 + k[1]**2 + k[2]**2)
    vec = (k[0]/k_mod, k[1]/k_mod, k[2]/k_mod)
    new_point = (j[0]+0.1*vec[0], j[1]+ 0.1*vec[1], j[2]+0.1*vec[2])
    return new_point

# Returns Euclidean distance between two 3D points
def cost2go(pt1, pt2):
    dist = math.sqrt((pt2[0] - pt1[0]) ** 2 + (pt2[1] - pt1[1]) ** 2 + (pt2[2] - pt1[2]) ** 2) 
    return dist

# Generates a random seed in 3D rounded off to 0.5
def generate_seed():
    x = round(random.uniform(0 , length)*2)/2
    y = round(random.uniform(0 , breadth)*2)/2
    z = round(random.uniform(0 , height)*2)/2
    return (x,y,z)
    
# Sphere to check Goal covergence 
def goalcheck_circle(x, y, z, goal_x, goal_y, goal_z):
    if ((x - goal_x) ** 2 + (y - goal_y) ** 2 + (z - goal_z) **2 <= (0.5 ** 2)):
        return True
    else:
        return False
    
# Returns the point after maximum step propagation disance of 2m
def max_step_prop(j, i):
    k = (i[0] - j[0], i[1] - j[1], i[2] - j[2])
    k_mod = math.sqrt(k[0]**2 + k[1]**2 + k[2]**2)
    vec = (k[0]/k_mod, k[1]/k_mod, k[2]/k_mod)
    new_point = (j[0]+2*vec[0], j[1]+ 2*vec[1], j[2]+2*vec[2])
    return new_point

# Returns the set if neighbours for a seed in a given radius
def neighbours(seed, r, tree):
    results = tree.query_ball_point((seed), r)
    nearby_points = X[results]
    return nearby_points

obstacle_map()
start = (2.5,12,0)
goal_x, goal_y, goal_z = (20,2,10)
visited_nodes = set()
all_nodes = []
parent_list = []
seed_list = []
parent_dict = {}
cost_dict = {}

visited_nodes.add(start)
all_nodes.append(start)
parent_dict[start] = "okay"
cost_dict[start] = 0

seed = (0,0,0)
print("\n")

i = 0
solution = 0
while(goalcheck_circle(goal_x, goal_y, goal_z, seed[0], seed[1], seed[2])) == False:
    print("solution", solution)
    
    if solution == 1:
        break
        
    seed = generate_seed()

    if ((seed not in visited_nodes) and not obstacle_check(seed[0], seed[1], seed[2])):
        
        X = np.asarray(all_nodes)
        tree = spatial.KDTree(X)
        
        r = 2
        n = (0,0,0)

        while(1):
            n = neighbours(seed, r,tree)
            if(n == seed).all():
                r = r + 1
            else:
                break        
        
        for pt in n:
            pt = tuple(pt)
            cost = cost_dict[pt]
            cost_new = cost + cost2go(pt, seed)
            q.put((cost_new, pt, cost))
        
        parent = q.get()
        q = PriorityQueue() 
        parent = parent[1] 
                     
        if (cost2go(parent,seed) > 2):
            seed = max_step_prop(parent, seed)
            seed = (round(seed[0], 1), round(seed[1], 1), round(seed[2], 1))
            
            
        par = seed
        s = parent
        a = 0

        while(cost2go(par,s)>=0.1):
            a = line_obstacle_check(s, par)
            if obstacle_check(a[0], a[1], a[2]):
                break
            s = a

        s = (round(s[0], 1), round(s[1], 1), round(s[2], 1))
         
        if s not in visited_nodes:
            neww_cost = cost2go(seed, parent) + cost_dict[parent]  
            all_nodes.insert(0, s)
            visited_nodes.add(s)
            parent_dict[s] = parent 
            cost_dict[s] = neww_cost
            parent_list.append((parent[0], parent[1], parent[2]))
            seed_list.append((s[0], s[1], s[2]))
            ax.plot3D((parent[0],s[0]), (parent[1], s[1]), (parent[2], s[2]), 'black')

            for nei in n:
                nei = tuple(nei)
                if nei != parent:
                    if cost_dict[nei] > (cost_dict[s] + cost2go(s, nei)):
                        parent_dict[nei] = s
                        cost_dict[nei] = cost_dict[s] + cost2go(s, nei)

            
            seed_check = s
            goal_check = (goal_x, goal_y, goal_z)
            temp1 = seed_check
            temp2 = goal_check
            if (seed_check != goal_check):

                while(cost2go(goal_check, seed_check)>=0.1):
                    a = line_obstacle_check(seed_check, goal_check)
                    if obstacle_check(a[0], a[1], a[2]):
                        break
                    seed_check = a
                goal_check = temp1

            if (cost2go(seed_check, temp2)) < 0.2:
                print("termination")
                solution = 1
                all_nodes.insert(0, goal_check)
                visited_nodes.add(goal_check)
                parent_dict[temp2] = temp1 
                print("parent", temp1)
                print("goal", temp2)

                goal_cost = cost_dict[s] + cost2go(seed_check, goal_check)
                cost_dict[goal_check] = goal_cost
                parent = temp1
        else:
            all_nodes.pop(0)

print("goal_converged")
print("parent", parent_dict)
print("parent", parent)
path = []
path.append((s[0], s[1], s[2]))

while parent != 'okay':
    temp = parent_dict.get(parent)
    path.append(parent)
    parent = temp
    if parent == (start):
        break
path.append(start)
print("Backtracking done - shortest path found")

path = path[::-1]
path.append(temp2)
oroginal_path = path.copy()

for i in range(100):
    choice1 = random.choice(path)
    choice2 = random.choice(path)
    ch1 = choice1
    ch2 = choice2
                    
    ind1 = path.index(choice1)
    ind2 = path.index(choice2)

    if (choice1 != choice2):
        
        while(cost2go(choice2, choice1)>=0.1):
            a = line_obstacle_check(choice1, choice2)
            if obstacle_check(a[0], a[1], a[2]):
                break
            choice1 = a
        choice2 = ch1
        
    if (cost2go(choice1, ch2)) < 0.2:
        if ind1< ind2:
            del path[ind1+1:ind2]

        if ind2 < ind1:
            del path[ind2+1:ind1]


#     print("after_optimise")
#     print("choice1", choice2)
#     print("choice2", choice1)

#     print("\n")

            

    

x_path_original = [oroginal_path[i][0] for i in range(len(oroginal_path))]
y_path_original = [oroginal_path[i][1] for i in range(len(oroginal_path))]
z_path_original = [oroginal_path[i][2] for i in range(len(oroginal_path))]

x_path = [path[i][0] for i in range(len(path))]
y_path = [path[i][1] for i in range(len(path))]
z_path = [path[i][2] for i in range(len(path))]

ax.scatter(x_path, y_path, z_path, c='g', marker = 'o')

# Uncomment next line to see the plotting of the path before optimisation
ax.plot3D(x_path_original, y_path_original, z_path_original, "-g")

ax.plot3D(x_path, y_path, z_path, "-r")


# Uncomment below lines to see the real time exploration of the tree
# l = 0
# while l < len(seed_list):
#     ax.plot3D((parent_list[l][0], seed_list[l][0]), (parent_list[l][1], seed_list[l][1]), (parent_list[l][2], seed_list[l][2]),  'black')
#     l = l + 1
#     plt.show()
#     plt.pause(0.000000000000000000000000000000000001)



print("before", oroginal_path)
print("after", path)
ax.set_xlabel('x-axis') 
ax.set_ylabel('y-axis') 
ax.set_zlabel('z-axis') 
plt.show()
plt.pause(15)
plt.savefig("trial.png")
plt.close()