import numpy as np
import math
from numpy.linalg import norm

def swap(ta1,tb1):
    ta = tb1
    tb = ta1
    return ta, tb

def list_diff(list1,list2):
    difference = [];
    zip_object = zip(list1, list2)
    for list1_i, list2_i in zip_object:
        difference.append(list1_i-list2_i)
    return difference

def random_config():
    r = np.zeros(2);
    x_dim = 50;                                 #x size of room
    y_dim = 50;                                  #y size of room
    for x in range (0,2):
        r[x] = np.random.random_sample();
    r[0] = x_dim*r[0];
    r[1] = y_dim*r[1];
    return r

def on_segment(p, q, r):
    if r[0] <= max(p[0], q[0]) and r[0] >= min(p[0], q[0]) and r[1] <= max(p[1], q[1]) and r[1] >= min(p[1], q[1]):
        return True
    return False

def orientation(p, q, r):
    val = ((q[1] - p[1]) * (r[0] - q[0])) - ((q[0] - p[0]) * (r[1] - q[1]))
    if val == 0 : return 0
    return 1 if val > 0 else -1
def intersects(seg1, seg2):
    p1, q1 = seg1
    p2, q2 = seg2
    o1 = orientation(p1, q1, p2)
    o2 = orientation(p1, q1, q2)
    o3 = orientation(p2, q2, p1)
    o4 = orientation(p2, q2, q1)
    if o1 != o2 and o3 != o4:
        return True
    if o1 == 0 and on_segment(p1, q1, p2) : return True
    if o2 == 0 and on_segment(p1, q1, q2) : return True
    if o3 == 0 and on_segment(p2, q2, p1) : return True
    if o4 == 0 and on_segment(p2, q2, q1) : return True
    return False

#tested
def collision_checker(q_curr,q_next,obstacle):
    collision = False
    for i in obstacle:
        collision_pt = intersects((q_curr,q_next),obstacle[i])
        collision = collision or collision_pt
    return collision

#tested
def find_nearest(tree,q_rand):
    length = len(tree);
    d = 100000;
    for i in range (0,length-1):
        test_w_inx = tree[i];
        test = test_w_inx[0:2];
        diff = list_diff(q_rand,test);
        d1 = np.sqrt(np.sum(np.square(diff)));
        if d1 < d:
            q_near_pos = i;
            q_near = test;
            d = d1;
    q_rand.append(q_near_pos);
    return q_near, q_rand_ind

def limit(q_rand, q_near):
    step_size = xxxx;                           #define step_size
    q_diff = q_rand[0:2]-q_near[0:2];
    q_int = q_near + step_size*q_diff;
    return q_int

def local_planner(q_near, q_int, step_size, obstacle):
    delta_q = list_diff(q_near[0:2]-q_int[0:2])
    num_steps = math.ceil((norm(delta_q)/step_size))
    step = delta_q/num_steps
    collision = False
    q = q_int
    for i in range (0,num_steps-1):
        q_step = q + step
        point_collision = collision_checker(q, q_step, obstacle)
        q = q_step
        collision = collision or point_collision
    success = not collision
    return success

def rrt_extend_single(tree1,q_rand,step_length,obstacle):
    q_target = []
    q_near, q_rand = find_nearest(tree1,q_rand) #all qs now have three values (x,y,i)
    q_int = limit(q_rand, q_near, step_length)
    result = local_planner(q_near, q_int, step_size, obstacle)
    if result == True:
        tree1.append(q_int)
        q_target = q_int
    return result, tree1, q_target

def rrt_extend_multiple(tree2, q_target, step_length,obstacle):
    q_near = find_nearest(tree2, q_target[0:2])
    q_int = limit(q_target,q_near,step_length)
    q_last = q_near_pos
    num_steps = math.ceil(norm(list_diff(q_target[0:2],q_near[0:2])/step_length)
    for i in range (0,num_steps-1):
        result = local_planner(q_int,q_last,step_size,obstacle)
        if result == False:
            return result, tree2, q_connect
        tree2.append(q_int)
        q_connect = q_int
        if i < num_steps:
            q_last = q_int
            q_int = limit(q_target,q_int,step_length)
    return result, tree2, q_connect

def rrt_connect(q_start,q_goal,step_length,obstacle):
    tree1 = []
    tree1.append(q_start)
    tree2 = []
    tree2.append(q_goal)
    success = False
    for i in range(0,1000):
        q_rand = random_config()
        result, tree1, q_target = rrt_extend_single(tree1,q_rand,step_length,obstacle)
        if result == True:
            result2, tree2, q_connect = rrt_extend_multiple(tree2,q_target,step_length,obstacle)
            if result2 == True:
                success = True
                return success, q_connect, tree1, tree2

        tree1, tree2 = swap(tree1,tree2)
    return success

#Tested
def findpath(tree1, q_connect):
    q_parent = q_connect[2]
    length = len(tree1)
    q_path = []
    q_path.append(int(length-1)) #q_connect position is last point
    q_path.append(int(q_parent)) #q_connects parent
    while (q_parent != 0):
        q = tree1[int(q_parent)]
        q_parent = q[2]
        q_path.append(int(q_parent))
    return q_path

#Tested
def path_publisher(path1, tree1):
    len1 = len(path1)
    current_pos = tree1[1]
    for i in path1:
        #each itteration publishes a position and checks for collision for next pos.
        next_pos = tree1[i]
        pub = next_pos[0:2]
        print(pub)
        #get lidar data
        collision = collision_checker(next_pos,current_pos,obstacle)
        if (collision == True):
            return collision, current_pos, obstacle
        current_pos = tree1[i]
    return collision, current_pos, obstacle #False, path completed successfully.

def main_loop(q_start,q_goal,step_length,obstacle):
    success, q_connect, tree1, tree2 = rrt_connect(q_start,q_goal,step_length,obstacle)
    if (success == True):
         path1 = findpath(tree1, q_connect)
         path1 = path1[::-1] #flip
         path2 = findpath(tree2 ,q_connect)
         collision, current_pos, obstacle = path_publisher(path1, tree1)
         if (collision == False):
             collision, current_pos, obstacle = path_publisher(path2,tree2)
             if (collision == False):
                 total_success = True
                 return total_success # = true and finish
             else:
                 q_start = current_pos
                 return total_success, q_start, obstacle #false
         else:
             q_start = current_pos
             return total_success, q_start, obstacle #flase

#MAIN CODE:
total_success = False
q_start =
q_goal =
step_length =
obstacle = []
while total_success == False:
    total_success, q_start, obstacle = main_loop(q_start,q_goal,step_length,obstacle)
