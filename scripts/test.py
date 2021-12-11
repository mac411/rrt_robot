import numpy as np
import math
from numpy.linalg import norm
def random_config():
    r = [0, 0];
    x_dim = 10 #x size of room
    y_dim = 10 #y size of room
    for x in range (0,2):
        r[x] = np.random.random_sample();
    r[0] = x_dim*r[0];
    r[1] = y_dim*r[1];
    return r
def list_diff(list1,list2):
    difference = [];
    zip_object = zip(list1, list2)
    for list1_i, list2_i in zip_object:
        difference.append(list1_i-list2_i)
    return difference

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
            q_near = test_w_inx;
            d = d1;
    q_rand.append(q_near_pos);
    return q_near, q_rand

tree = [];
rstart = [2,5,0];
tree.append(rstart)
r1 = random_config();
r1.append(1)
tree.append(r1)
r2 = random_config();
r2.append(2)
tree.append(r2)
r3 = random_config();
r3.append(3)
tree.append(r3)
tree = np.array(tree)
print(np.array(tree))
tree1 = tree[::-1]
merge_tree = np.concatenate((tree, tree1), axis = 1)
print(merge_tree)

#q_rand = random_config();
#q_near, q_rand = find_nearest(tree,q_rand);
