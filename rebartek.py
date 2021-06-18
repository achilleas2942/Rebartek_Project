import math
import functools as ft
import numpy as np
import matplotlib as mpl
from mpl_toolkits.mplot3d import Axes3D
import matplotlib.pyplot as plt
from matplotlib.patches import FancyArrowPatch
from mpl_toolkits.mplot3d import proj3d

# Initial values
dbase, alpha, dtool = 10, 6, 2  # robot design parameters

theta1, theta2, theta3, theta4, theta5, theta6 = 0, 0, 0, 0, 0, 0

T = [[1, 0, 0, 6] , [0, -1, 0, 0], [0, 0, -1, 8], [0, 0, 0, 1]]
r11 = T[0][0]
r12 = T[0][1]
r13 = T[0][2]
r21 = T[1][0]
r22 = T[1][1]
r23 = T[1][2]
r31 = T[2][0]
r32 = T[2][1]
r33 = T[2][2]
px = T[0][3]
py = T[1][3]
pz = T[2][3]

####################################################################
######################## forward kinematics ########################

# Given the angle of each joint -> the position and orientation of the end effector will be calculated
def forwordKinematics():
      global theta1, theta2, theta3, theta4, theta5, theta6
      global r11, r12, r13, r21, r22, r23, r31, r32, r33, px, py, pz
      print("If you want to give desired angles for each DoF press Y/y")
      choice1 = input()
      if(choice1 == 'y' or choice1 == 'Y'):
            print("theta1 = ")
            theta1 = float(input())
            print("theta2 = ")
            theta2 = float(input())
            print("theta3 = ")
            theta3 = float(input())
            print("theta4 = ")
            theta4 = float(input())
            print("theta5 = ")
            theta5 = float(input())
            print("theta6 = ")
            theta6 = float(input())

      # Homogenous transformation matrices
      T1 = [[math.cos(math.radians(theta1)), -math.sin(math.radians(theta1)), 0, 0],
            [math.sin(math.radians(theta1)), math.cos(math.radians(theta1)), 0, 0],
            [0, 0, 1, dbase],
            [0, 0, 0, 1]]

      T2 = [[math.cos(math.radians(theta2)), -math.sin(math.radians(theta2)), 0, 0],
            [0, 0, 1, 0],
            [-math.sin(math.radians(theta2)), -math.cos(math.radians(theta2)), 0, 0],
            [0, 0, 0, 1]]

      T3 = [[math.cos(math.radians(theta3)), -math.sin(math.radians(theta3)), 0, alpha],
            [math.sin(math.radians(theta3)), math.cos(math.radians(theta3)), 0, 0],
            [0, 0, 1, 0],
            [0, 0, 0, 1]]

      T4 = [[math.cos(math.radians(theta4)), -math.sin(math.radians(theta4)), 0, 0],
            [0, 0, 1, dtool],
            [-math.sin(math.radians(theta4)), -math.cos(math.radians(theta4)), 0, 0],
            [0, 0, 0, 1]]

      T5 = [[math.cos(math.radians(theta5)), -math.sin(math.radians(theta5)), 0, 0],
            [0, 0, -1, 0],
            [math.sin(math.radians(theta5)), math.cos(math.radians(theta5)), 0, 0],
            [0, 0, 0, 1]]

      T6 = [[math.cos(math.radians(theta6)), -math.sin(math.radians(theta6)), 0, 0],
            [0, 0, 1, 0],
            [-math.sin(math.radians(theta6)), -math.cos(math.radians(theta6)), 0, 0],
            [0, 0, 0, 1]]

      T = ft.reduce(np.dot, [T1, T2, T3, T4, T5, T6])

      r11 = T[0][0]
      r12 = T[0][1]
      r13 = T[0][2]
      r21 = T[1][0]
      r22 = T[1][1]
      r23 = T[1][2]
      r31 = T[2][0]
      r32 = T[2][1]
      r33 = T[2][2]
      px = T[0][3]
      py = T[1][3]
      pz = T[2][3]

      return T


####################################################################
######################## inverse kinematics ########################

# Given the orientation and position of the end effector -> the angles of each joint will be calculated
def inverseKinematics():
      global theta1, theta2, theta3, theta4, theta5, theta6
      global r11, r12, r13, r21, r22, r23, r31, r32, r33, px, py, pz
      print("If you want to give desired rotation and position parameters press Y/y")
      choice2 = input()
      if(choice2 == 'y' or choice2 == 'Y'):
            print("r11 = ")
            r11 = float(input())
            print("r12 = ")
            r12 = float(input())
            print("r13 = ")
            r13 = float(input())
            print("r21 = ")
            r21 = float(input())
            print("r22 = ")
            r22 = float(input())
            print("r23 = ")
            r23 = float(input())
            print("r31 = ")
            r31 = float(input())
            print("r32 = ")
            r32 = float(input())
            print("r33 = ")
            r33 = float(input())
            print("px = ")
            px = float(input())
            print("py = ")
            py = float(input())
            print("pz = ")
            pz = float(input())

      R = [[r11, r12, r13], [r21, r22, r23], [r31, r32, r33]]
      P = [[px], [py], [pz]]

      # Given the position of the end effector -> the angles theta1, theta2, theta3 will be calculated
      r1 = math.sqrt(px ** 2 + py ** 2)
      r2 = pz - dbase
      r3 = math.sqrt(r1 ** 2 + r2 ** 2)
      f1 = math.degrees(math.acos((dtool ** 2 - alpha ** 2 - r3 ** 2) / (- 2 * alpha * r3)))
      f2 = math.degrees(math.atan2(r2, r1))
      f3 = math.degrees(math.acos((r3 ** 2 - alpha ** 2 - dtool ** 2) / (- 2 * alpha * dtool)))
      theta1 = math.degrees(math.atan2(py, px))
      theta2 = -(f2 + f1)
      theta3 = 90 - f3

      # Given the orientation of the end effector -> the angles theta1, theta2, theta3 will be calculated
      R1 = [[math.cos(math.radians(theta1)), -math.sin(math.radians(theta1)), 0],
            [math.sin(math.radians(theta1)), math.cos(math.radians(theta1)), 0],
            [0, 0, 1]]

      R2 = [[math.cos(math.radians(theta2)), -math.sin(math.radians(theta2)), 0],
            [0, 0, 1],
            [-math.sin(math.radians(theta2)), -math.cos(math.radians(theta2)), 0]]

      R3 = [[math.cos(math.radians(theta3)), -math.sin(math.radians(theta3)), 0],
            [math.sin(math.radians(theta3)), math.cos(math.radians(theta3)), 0],
            [0, 0, 1]]

      R03 = ft.reduce(np.dot, [R1, R2, R3])

      R36 = ft.reduce(np.dot, [R03.transpose(), R])

      theta4 = math.degrees(math.atan2(R36[2][2], -R36[0][2]))
      theta5 = math.degrees(math.atan2(math.sqrt(1 - R36[1][2] ** 2), R36[1][2]))
      theta6 = math.degrees(math.atan2(-R36[1][1], R36[1][0]))

      return {"theta1":theta1, "theta2":theta2, "theta3":theta3, "theta4":theta4, "theta5":theta5, "theta6":theta6}

####################################################################
################## figure for coordinate systems ###################
      
# 3D arrows
class Arrow3D(FancyArrowPatch):
    def __init__(self, xs, ys, zs, *args, **kwargs):
        FancyArrowPatch.__init__(self, (0, 0), (0, 0), *args, **kwargs)
        self._verts3d = xs, ys, zs

    def draw(self, renderer):
        xs3d, ys3d, zs3d = self._verts3d
        xs, ys, zs = proj3d.proj_transform(xs3d, ys3d, zs3d, renderer.M)
        self.set_positions((xs[0], ys[0]), (xs[1], ys[1]))
        FancyArrowPatch.draw(self, renderer)

if __name__ == '__main__':
    center = [0, 0, 0]
    length = 1
    width = 1
    height = 1
    fig = plt.figure()
    ax1 = fig.add_subplot(111, projection='3d')

    ax1.set_xlabel('X')
    ax1.set_xlim(-18, 18)
    ax1.set_ylabel('Y')
    ax1.set_ylim(-18, 18)
    ax1.set_zlabel('Z')
    ax1.set_zlim(-18, 18)

    # Creating the arrows:
    arrow_prop_dict = dict(mutation_scale=20, arrowstyle='->', shrinkA=0, shrinkB=0)

    a0 = Arrow3D([0, 1], [0, 0], [0, 0], **arrow_prop_dict, color='r')
    ax1.add_artist(a0)
    a0 = Arrow3D([0, 0], [0, 1], [0, 0], **arrow_prop_dict, color='b')
    ax1.add_artist(a0)
    a0 = Arrow3D([0, 0], [0, 0], [0, 1], **arrow_prop_dict, color='g')
    ax1.add_artist(a0)

    ax1.text(1.1, 0, 0, r'$x0$')
    ax1.text(0, 1.1, 0, r'$y0$')
    ax1.text(0, 0, 1.1, r'$z0$')

    a1 = Arrow3D([0, 1], [0, 0], [dbase, dbase], **arrow_prop_dict, color='r')
    ax1.add_artist(a1)
    a1 = Arrow3D([0, 0], [0, 1], [dbase, dbase], **arrow_prop_dict, color='b')
    ax1.add_artist(a1)
    a1 = Arrow3D([0, 0], [0, 0], [dbase, dbase + 1], **arrow_prop_dict, color='g')
    ax1.add_artist(a1)

    ax1.text(1.1, 0, dbase, r'$x1$')
    ax1.text(0, 1.1, dbase, r'$y1$')
    ax1.text(0, 0, dbase + 1.1, r'$z1$')

    a2 = Arrow3D([0, 1], [0, 0], [dbase, dbase], **arrow_prop_dict, color='r')
    ax1.add_artist(a2)
    a2 = Arrow3D([0, 0], [0, 0], [dbase, dbase - 1], **arrow_prop_dict, color='b')
    ax1.add_artist(a2)
    a2 = Arrow3D([0, 0], [0, 1], [dbase, dbase], **arrow_prop_dict, color='g')
    ax1.add_artist(a2)

    ax1.text(1.6, 0, dbase, r'$x2$')
    ax1.text(0, 0, dbase - 1.1, r'$y2$')
    ax1.text(0, 1.6, dbase, r'$z2$')

    a3 = Arrow3D([alpha, alpha + 1], [0, 0], [dbase, dbase], **arrow_prop_dict, color='r')
    ax1.add_artist(a3)
    a3 = Arrow3D([alpha, alpha], [0, 0], [dbase, dbase - 1], **arrow_prop_dict, color='b')
    ax1.add_artist(a3)
    a3 = Arrow3D([alpha, alpha], [0, 1], [dbase, dbase], **arrow_prop_dict, color='g')
    ax1.add_artist(a3)

    ax1.text(alpha + 1.1, 0, dbase, r'$x3$')
    ax1.text(alpha, 0, dbase - 1.1, r'$y3$')
    ax1.text(alpha, 1.1, dbase, r'$z3$')

    a4 = Arrow3D([alpha, alpha + 1], [0, 0], [dbase - dtool, dbase - dtool], **arrow_prop_dict, color='r')
    ax1.add_artist(a4)
    a4 = Arrow3D([alpha, alpha], [0, -1], [dbase - dtool, dbase - dtool], **arrow_prop_dict, color='b')
    ax1.add_artist(a4)
    a4 = Arrow3D([alpha, alpha], [0, 0], [dbase - dtool, dbase - dtool - 1], **arrow_prop_dict, color='g')
    ax1.add_artist(a4)

    ax1.text(alpha + 1.1, 0, dbase - dtool, r'$x4$')
    ax1.text(alpha, -1.1 , dbase -dtool, r'$y4$')
    ax1.text(alpha, 0, dbase - dtool - 1.1, r'$z4$')

    a5 = Arrow3D([alpha, alpha + 1], [0, 0], [dbase - dtool, dbase - dtool], **arrow_prop_dict, color='r')
    ax1.add_artist(a5)
    a5 = Arrow3D([alpha, alpha], [0, 0], [dbase - dtool, dbase - dtool - 1], **arrow_prop_dict, color='b')
    ax1.add_artist(a5)
    a5 = Arrow3D([alpha, alpha], [0, 1], [dbase - dtool, dbase - dtool], **arrow_prop_dict, color='g')
    ax1.add_artist(a5)

    ax1.text(alpha + 1.6, 0, dbase - dtool, r'$x5$')
    ax1.text(alpha, 0, dbase - dtool - 1.6, r'$y5$')
    ax1.text(alpha, 1.1 , dbase -dtool, r'$z5$')

    a6 = Arrow3D([alpha, alpha + 1], [0, 0], [dbase - dtool, dbase - dtool], **arrow_prop_dict, color='r')
    ax1.add_artist(a6)
    a6 = Arrow3D([alpha, alpha], [0, -1], [dbase - dtool, dbase - dtool], **arrow_prop_dict, color='b')
    ax1.add_artist(a6)
    a6 = Arrow3D([alpha, alpha], [0, 0], [dbase - dtool, dbase - dtool - 1], **arrow_prop_dict, color='g')
    ax1.add_artist(a6)

    ax1.text(alpha + 2.1, 0, dbase - dtool, r'$x6$')
    ax1.text(alpha, -1.6 , dbase -dtool, r'$y6$')
    ax1.text(alpha, 0, dbase - dtool - 2.1, r'$z6$')    

    
####################################################################
########################## path planning ###########################

def pathPlanning():
      global theta1, theta2, theta3, theta4, theta5, theta6
      global r11, r12, r13, r21, r22, r23, r31, r32, r33, px, py, pz
      inverseKinematics()
      q0 = [theta1, theta2, theta3, theta4, theta5, theta6] # angles #0 goal position and orientation
      print("Give the first box position and orientation")
      inverseKinematics()
      goalpx = px
      goalpy = py
      goalpz = pz + 1
      pz = goalpz
      inverseKinematics()
      q4 = [theta1, theta2, theta3, theta4, theta5, theta6] # angles #4 goal position and orientation
      time = 100

      for i in range(1, 4, 1):
            print("Give the next box position and orientation")
            inverseKinematics()
            q2 = [theta1, theta2, theta3, theta4, theta5, theta6] # angles for #2 position and orientation
            nextpx = px
            nextpy = py
            nextpz = pz
            if(nextpx >= goalpx):
                  px = goalpx + 1
            else: px = goalpx - 1
            if(py >= goalpy):
                  py = goalpy + 1
            else: py = goalpy - 1
            pz = goalpz + 1
            inverseKinematics()
            q1 = [theta1, theta2, theta3, theta4, theta5, theta6] # angles for #1 position and orientation
            trajectory(q0, q1, time)
            q2
            trajectory(q1, q2, time)
            q3 = q1 # angles for #3 position and orientation
            trajectory(q2, q3, time)
            q0 = q4 # angles for #4 position and orientation
            trajectory(q3, q4, time)
            goalpy = pz + 1
            pz = goalpz
            inverseKinematics()
            q4 = [theta1, theta2, theta3, theta4, theta5, theta6] # angles for #4 position and orientation

def trajectory(qstart, qstop, tv):
      step = max(tv)
      t = tv / step
      qd0 = [0, 0, 0, 0, 0, 0]
      qd1 = qd0
      A = 6 * (qstop - qstart) - 3 * (qd1 + qd0) * step
      B = -15 * (qstop - qstart) + (8 * qd0 + 7 * qd1) * step
      C = 10 * (qstop - qstart) - (6 * qd0 + 4 * qd1) * step
      E = qd0 * step
      F = qstart

      tt = [t ** 5, t ** 4, t ** 3, t ** 2, t, np.ones(len(t))];
      c = [A, B, C, np.zeros(len(A)), E, F]

      qt = tt * c
      return qt


####################################################################
########################## call function ###########################

print("Puma 560, Achilleas Version\n16/06/2021\n")
print("If you want to experiment with forward kinematics press 1\nIf you want to experiment with inverse kinematics press 2")
print("If you want to plot the Denavit-Hartenberg coordinate systems for the initial position press 3")
print("If you want to go to path planning press 4")
print("If you want to exit press any other number")
choice = int(input())
while(choice == 1 or choice == 2 or choice == 3 or choice ==4):
      if(choice == 1):
            transMaxtrix = forwordKinematics()
            print("The Transformation Matrix of the system:\n", transMaxtrix, "\n")
      elif(choice == 2):
            thetas = inverseKinematics()
            print(thetas, "\n")
      elif(choice == 3):
            plt.show()
      elif(choice == 4):
            pathPlanning()
      print("If you want to experiment with forward kinematics press 1\nIf you want to experiment with inverse kinematics press 2")
      print("If you want to plot the Denavit-Hartenberg coordinate systems for the initial position press 3")
      print("If you want to go to path planning press 4")
      print("If you want to exit press any other number")
      choice = int(input())