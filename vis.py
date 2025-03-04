
import matplotlib.pyplot as plt
import numpy as np
from math import sqrt, cos, sin, radians
import matplotlib.pyplot as plt
e = np.array([0, 0, 1])
x = (1/sqrt(3)) * np.array([1., 1., 1.])
v = radians(60)
e_cross = np.array([ [0, -e[2], e[1]], [e[2], 0, -e[0]], [-e[1], e[0], 0] ])
A = cos(v)*np.eye(3) - sin(v)*e_cross + (1-cos(v))*np.outer(e,e)
x_rotated = np.transpose(A)@x # active rotation
x_parallel = np.dot(x, e) * e
x_per = x - x_parallel
x_per_rot = x_rotated - x_parallel
sinv_e_cross_x = sin(v) * e_cross @ x
cosv_x_per = cos(v) * x_per
fig = plt.figure(1)
ax = fig.add_subplot(111, projection='3d')
ax.quiver(0, 0, 0, x[0], x[1], x[2], color='r', label='x' )
ax.quiver(0, 0, 0, x_rotated[0], x_rotated[1], x_rotated[2], color='g',
label='A(e,v)x' )
ax.quiver(0, 0, 0, e[0], e[1], e[2], color='purple', label='e' )
ax.quiver(0, 0, 0, x_parallel[0], x_parallel[1], x_parallel[2], color='black',
label='x_parallel' )
ax.quiver(x_parallel[0], x_parallel[1], x_parallel[2], x_per[0], x_per[1],
x_per[2], color='r', linestyle='--', label='x_perpendicular')
ax.quiver(x_parallel[0], x_parallel[1], x_parallel[2], x_per_rot[0], x_per_rot[1],
x_per_rot[2], color='magenta', linestyle='--', label='A(e,v)x perpendicular part("this")')
ax.quiver(x_parallel[0], x_parallel[1], x_parallel[2], cosv_x_per[0],
cosv_x_per[1], cosv_x_per[2], color='darkorange', linewidth = 4,
label='cos(v)*x_perpendicular')
ax.quiver(cosv_x_per[0], cosv_x_per[1], x_parallel[2], sinv_e_cross_x[0],
sinv_e_cross_x[1], sinv_e_cross_x[2], color='blue', linewidth = 4,
label='sin(v)*[ex]x')
ax.legend( loc='upper left', fontsize = 12 )
ax.set_xlim3d(-1, 1)
ax.set_ylim3d(-1,1)
ax.set_zlim3d(-1,1)
plt.show()
