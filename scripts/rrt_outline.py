import numpy as np
import math

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

def collision_checker(pos,vertices):
    #given verticies, find closest one and find distance to both of its edges
    for i in vertices:
        diff = pos-verticies[i]
        d1 = np.sqrt(np.sum(np.square(diff)));

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


def local_planner():
    collision_checkeer()

def rrt_extend_single():
    find_nearest()
    limit()
    local_planner()
#ONLY NEEDED FOR RRT CONNECT
def rrt_extend_multiple():
    find_nearest()
    limit()
    local_planner()

def rrt():
    tree_1(q_start);
    tree_2(q_goal);
    random_config()
    rrt_extend_single()
    rrt_extend_multiple()
