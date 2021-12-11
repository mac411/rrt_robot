import numpy as np

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

pos = [2, 6]
v1 = [4,9]
v2 = [4,12]

d = point_to_line(pos, v1, v2)
print(d)
