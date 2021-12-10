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

def point_to_line(pos, v1, v2):
    xp = pos[0];
    yp = pos[1];
    x1 = v1[0];
    y1 = v1[1];
    x2 = v2[0];
    y2 = v2[1];

    a = xp-x1;
    b = yp-y1;
    c = x2-x1;
    d = y2-y1;
    dot = a*c + b*d;
    len_sq = c*c + d*d;
    param = -1;
    if len_sq != 0:
        param = dot/len_sq
    if param < 0:
        xx = x1;
        yy = y1;
    elif param >1:
        xx = x2;
        yy = y2;
    else:
        xx = x1+param*c;
        yy = y1+param*d;
    dx = xp-xx;
    dy = yp-yy;
    d = np.sqrt(dx*dx+dy*dy)
    return d

def onSegment(p:tuple, q:tuple, r:tuple) -> bool:

    if ((q[0] <= max(p[0], r[0])) &
        (q[0] >= min(p[0], r[0])) &
        (q[1] <= max(p[1], r[1])) &
        (q[1] >= min(p[1], r[1]))):
        return True

    return False

# To find orientation of ordered triplet (p, q, r).
# The function returns following values
# 0 --> p, q and r are collinear
# 1 --> Clockwise
# 2 --> Counterclockwise
def orientation(p:tuple, q:tuple, r:tuple) -> int:

    val = (((q[1] - p[1]) *
            (r[0] - q[0])) -
           ((q[0] - p[0]) *
            (r[1] - q[1])))

    if val == 0:
        return 0
    if val > 0:
        return 1 # Collinear
    else:
        return 2 # Clock or counterclock

def doIntersect(p1, q1, p2, q2):

    # Find the four orientations needed for
    # general and special cases
    o1 = orientation(p1, q1, p2)
    o2 = orientation(p1, q1, q2)
    o3 = orientation(p2, q2, p1)
    o4 = orientation(p2, q2, q1)

    # General case
    if (o1 != o2) and (o3 != o4):
        return True

    # Special Cases
    # p1, q1 and p2 are collinear and
    # p2 lies on segment p1q1
    if (o1 == 0) and (onSegment(p1, p2, q1)):
        return True

    # p1, q1 and p2 are collinear and
    # q2 lies on segment p1q1
    if (o2 == 0) and (onSegment(p1, q2, q1)):
        return True

    # p2, q2 and p1 are collinear and
    # p1 lies on segment p2q2
    if (o3 == 0) and (onSegment(p2, p1, q2)):
        return True

    # p2, q2 and q1 are collinear and
    # q1 lies on segment p2q2
    if (o4 == 0) and (onSegment(p2, q1, q2)):
        return True

    return False

# Returns true if the point p lies
# inside the polygon[] with n vertices
def is_inside_polygon(points:list, p:tuple) -> bool:

    n = len(points)

    # There must be at least 3 vertices
    # in polygon
    if n < 3:
        return False

    # Create a point for line segment
    # from p to infinite
    extreme = (10000, p[1])
    count = i = 0

    while True:
        next = (i + 1) % n

        # Check if the line segment from 'p' to
        # 'extreme' intersects with the line
        # segment from 'polygon[i]' to 'polygon[next]'
        if (doIntersect(points[i],
                        points[next],
                        p, extreme)):

            # If the point 'p' is collinear with line
            # segment 'i-next', then check if it lies
            # on segment. If it lies, return true, otherwise false
            if orientation(points[i], p,
                           points[next]) == 0:
                return onSegment(points[i], p,
                                 points[next])

            count += 1

        i = next

        if (i == 0):
            break

    # Return true if count is odd, false otherwise
    return (count % 2 == 1)

#only checks one polygon
#need to check if circle completly in polygon
def collision_checker(pos,vertices):
    pos = pos[0:2]
    collision = False
    radius = 2;                                     #radius of robot
    for i in vertices:
        diff = pos-verticies[i]
        d1 = np.sqrt(np.sum(np.square(diff)))
        if d1 < radius:
            collision = True
            return collision
        d = point_to_line(pos, vertices[i], vertices[i-1])
        if d < radius:
            collision = True
            return collision
    #check if inside polygon
    if (is_inside_polygon(points = vertices, p = pos)):
        collision = True
        return collision
    return collision

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

def local_planner(q_near, q_int, step_size):
    delta_q = list_diff(q_near[0:2]-q_int[0:2])
    num_steps = math.ceil((norm(delta_q)/step_size))
    step = delta_q/num_steps
    collision = False
    q = q_int
    for i in range (0,num_steps-1):
        q = q + step
        point_collision = collision_checker(q)
        collision = collision or point_collision
    success = not collision
    return success


def rrt_extend_single(tree1,q_rand,step_length):
    q_target = []
    q_near, q_rand = find_nearest(tree1,q_rand) #all qs now have three values (x,y,i)
    q_int = limit(q_rand, q_near, step_length)
    result = local_planner(q_near, q_int, step_size)
    if result == True:
        tree1.append(q_int)
        q_target = q_int
    return result, tree1, q_target

def rrt_extend_multiple(tree2, q_target, step_length):
    q_near = find_nearest(tree2, q_target[0:2])
    q_int = limit(q_target,q_near,step_length)
    q_last = q_near_pos
    num_steps = math.ceil(norm(list_diff(q_target[0:2],q_near[0:2])/step_length)
    for i in range (0,num_steps-1):
        result = local_planner(q_int,q_last,step_size)
        if result == False:
            return result, tree2, q_connect
        tree2.append(q_int)
        q_connect = q_int
        if i < num_steps:
            q_last = q_int
            q_int = limit(q_target,q_int,step_length)
    return result, tree2, q_connect

def rrt_connect(q_start,q_goal,step_length):
    tree1 = []
    tree1.append(q_start)
    tree2 = []
    tree2.append(q_goal)
    success = False
    for i in range(0,1000):
        q_rand = random_config()
        col = collision_checker(q_rand)
        if col == False:
            result, tree1, q_target = rrt_extend_single(tree1,q_rand,step_length)
            if result == True:
                result2, tree2, q_connect = rrt_extend_multiple(tree2,q_target,step_length)
                if result2 == True:
                    success = True
                    return success, q_connect, tree1, tree2

        tree1, tree2 = swap(tree1,tree2)
    return success
