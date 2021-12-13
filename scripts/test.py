import numpy as np
import math
from numpy.linalg import norm
import array
import matplotlib.pyplot as plt
figure, axes = plt.subplots()


angle = np.linspace(0, 2*np.pi,72)
radius = 2
x_center = 5
y_center = 2
x = radius *np.cos(angle) + x_center
y = radius *np.sin(angle) + y_center
idx = np.argsort(x)
#print(x)
#print(y)
xs = []
ys = []
for i in idx:
    xs = np.append(xs,x[i])
    ys = np.append(ys,y[i])
print(xs)
print(ys)

for i in range (0,int(len(xs)/2)):
    x_test = math.floor(xs[2*i])
    y_test_min = math.floor(ys[2*i])
    y_test_max = math.floor(ys[2*i+1])


    print(x_test, y_test_min, y_test_max)


#x.append(y)
#points = points[points[:,0].argsort()]
#print(xs)
#print(ys)


def random_config():
    r = np.zeros(2);
    x_dim = 50;                                 #x size of room
    y_dim = 50;                                  #y size of room
    for x in range (0,2):
        r[x] = np.random.random_sample();
    r[0] = x_dim*r[0]-25; #orgin in center of room so -25
    r[1] = y_dim*r[1]-25;
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
        diff = q_rand- test
        d1 = np.sqrt(np.sum(np.square(diff)));
        if d1 < d:
            q_near_pos = i;
            q_near = test_w_inx;
            d = d1;
    q_rand = np.append(q_rand, q_near_pos)
    return q_near, q_rand



tree = [];
rstart = [2,5,0];
tree.append(rstart)
r1 = random_config();
q_near_pos = 0
r1 = np.append(r1, q_near_pos)
tree.append(r1)
r2 = random_config();
r2 = np.append(r2, [1])
tree.append(r2)
r3 = random_config();
r3 = np.append(r3, [1])
tree.append(r3)
tree = np.array(tree)
print(tree)
test_w_inx = tree[2]
test = test_w_inx[0:2]
rt = random_config()
diff = rt-test
q_int = test+.1*diff
q_near = r3
dq = q_near[0:2] - q_int[0:2]
print(math.ceil(norm(dq)/5))
q_rand = random_config();
q_near, q_rand = find_nearest(tree,q_rand);
print(q_near, q_rand)
