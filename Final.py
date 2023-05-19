import networkx as nx
import matplotlib
import sys
from math import *
import time
import numpy as np
from NatNetClient import NatNetClient
from util import quaternion_to_euler_angle_vectorized1
import socket
import time
import cv2 as cv
from itertools import permutations




IP_ADDRESS = '192.168.0.209'




# Connect to the robot
s = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
s.connect((IP_ADDRESS, 5000))
print('Connected')




positions = {}
rotations = {}




# This is a callback function that gets connected to the NatNet client. It is called once per rigid body per frame
def receive_rigid_body_frame(robot_id, position, rotation_quaternion):
    # Position and rotation received
    positions[robot_id] = position
    # The rotation is in quaternion. We need to convert it to euler angles

    rotx, roty, rotz = quaternion_to_euler_angle_vectorized1(rotation_quaternion)

    rotations[robot_id] = rotz




# Function to calculate weight of edges
def sqdist(p1, p2):
    return sqrt( ((p1["x"] - p2["x"])**2) + ((p1["y"] - p2["y"])**2) )



def sqdistOpti(p, x, y):
    return sqrt( ((p["x"] - x)**2) + ((p["y"] - y)**2)  )




# Create graph
G = nx.path_graph(68)




# Add coordinates to graph points
pos = {
    0: {"x": 5.09, "y": 3.26},  
    1: {"x": 5.20, "y": 1.78},
    2: {"x": 5.33, "y": 0.93},
    3: {"x": 5.47, "y": 0.04},
    4: {"x": 5.27, "y": -0.84},
    5: {"x": 5.14, "y": -1.39},
    6: {"x": 5.09, "y": -1.99},
    7: {"x": 4.57, "y": 0.04},
    8: {"x": 3.91, "y": 3.29},
    9: {"x": 4.01, "y": 2.28},
    10: {"x": 3.96, "y": 1.02},
    11: {"x": 3.88, "y": -0.02},
    12: {"x": 3.90, "y": -0.82},
    13: {"x": 3.90, "y": -1.83},
    14: {"x": 3.08, "y": -1.88},
    15: {"x": 2.90, "y": -0.83},
    16: {"x": 2.81, "y": 0.03},
    17: {"x": 2.85, "y": 1.04},
    18: {"x": 2.80, "y": 1.84},
    19: {"x": 2.85, "y": 3.24},
    20: {"x": 1.90, "y": 3.21},
    21: {"x": 1.90, "y": 2.17},
    22: {"x": 1.87, "y": 1.34},
    23: {"x": 1.88, "y": 0.01},
    24: {"x": 1.89, "y": -0.96},
    25: {"x": 1.94, "y": -1.88},
    26: {"x": 1.07, "y": -1.89},
    27: {"x": 1.02, "y": -1.03},
    28: {"x": 0.96, "y": -0.10},
    29: {"x": 1.04, "y": 1.17},
    30: {"x": 1.06, "y": 2.08},
    31: {"x": 1.06, "y": 3.21},
    32: {"x": 0.22, "y": 3.21},
    33: {"x": 0.10, "y": 2.28},
    34: {"x": 0.07, "y": 1.25},
    35: {"x": 0.22, "y": 0.02},
    36: {"x": 0.25, "y": -1.02},
    37: {"x": 0.18, "y": -1.91},
    38: {"x": -0.60, "y": -1.96},
    39: {"x": -0.66, "y": -1.06},
    40: {"x": -0.69, "y": -0.08},
    41: {"x": -0.72, "y": 0.88},
    42: {"x": -0.65, "y": 2.01},
    43: {"x": -0.69, "y": 3.11},
    44: {"x": -1.45, "y": 3.05},
    45: {"x": -1.45, "y": 2.01},
    46: {"x": -1.42, "y": 1.04},
    47: {"x": -1.39, "y": -0.08},
    48: {"x": -1.55, "y": -1.09},
    49: {"x": -1.43, "y": -1.96},
    50: {"x": -2.16, "y": -1.99},
    51: {"x": -2.24, "y": -1.25},
    52: {"x": -2.28, "y": -0.11},
    53: {"x": -2.17, "y": 1.10},
    54: {"x": -2.12, "y": 2.03},
    55: {"x": -2.19, "y": 3.03},
    56: {"x": -2.87, "y": 3.11},
    57: {"x": -2.82, "y": 1.99},
    58: {"x": -2.86, "y": -0.83},
    59: {"x": -2.82, "y": -0.13},
    60: {"x": -2.81, "y": -1.06},
    61: {"x": -2.87, "y": -2.08},
    62: {"x": -3.89, "y": -2.18},
    63: {"x": -3.71, "y": -1.15},
    64: {"x": -3.88, "y": -0.12},
    65: {"x": -3.85, "y": 0.95},
    66: {"x": -3.85, "y": 1.93},
    67: {"x": -3.89, "y": 3.18}
}
nx.set_node_attributes(G, pos)




# Add edges between points with weights
G.add_edge(0, 1, weight = sqdist(G.nodes[0], G.nodes[1]))
# G.add_edge(0, 9, weight = sqdist(G.nodes[0], G.nodes[9]))
G.add_edge(0, 8, weight = sqdist(G.nodes[0], G.nodes[8]))
G.add_edge(1, 8, weight = sqdist(G.nodes[1], G.nodes[8]))
# G.add_edge(1, 10, weight = sqdist(G.nodes[1], G.nodes[10]))
G.add_edge(1, 2, weight = sqdist(G.nodes[1], G.nodes[2]))
# G.add_edge(2, 9, weight = sqdist(G.nodes[2], G.nodes[9]))
G.add_edge(2, 10, weight = sqdist(G.nodes[2], G.nodes[10]))
G.add_edge(2, 7, weight = sqdist(G.nodes[2], G.nodes[7]))
G.add_edge(2, 3, weight = sqdist(G.nodes[2], G.nodes[3]))
G.add_edge(3, 7, weight = sqdist(G.nodes[3], G.nodes[7]))
G.add_edge(3, 4, weight = sqdist(G.nodes[3], G.nodes[4]))
G.add_edge(4, 7, weight = sqdist(G.nodes[4], G.nodes[7]))
G.add_edge(4, 12, weight = sqdist(G.nodes[4], G.nodes[12]))
G.add_edge(4, 13, weight = sqdist(G.nodes[4], G.nodes[13]))
G.add_edge(4, 5, weight = sqdist(G.nodes[4], G.nodes[5]))
G.add_edge(5, 12, weight = sqdist(G.nodes[5], G.nodes[12]))
G.add_edge(5, 13, weight = sqdist(G.nodes[5], G.nodes[13]))
G.add_edge(5, 6, weight = sqdist(G.nodes[5], G.nodes[6]))
G.add_edge(6, 13, weight = sqdist(G.nodes[6], G.nodes[13]))
G.add_edge(13, 12, weight = sqdist(G.nodes[13], G.nodes[12]))
G.add_edge(13, 15, weight = sqdist(G.nodes[13], G.nodes[15]))
G.add_edge(13, 14, weight = sqdist(G.nodes[13], G.nodes[14]))
G.add_edge(12, 7, weight = sqdist(G.nodes[12], G.nodes[7]))
G.add_edge(12, 11, weight = sqdist(G.nodes[12], G.nodes[11]))
# G.add_edge(12, 16, weight = sqdist(G.nodes[12], G.nodes[16]))
G.add_edge(12, 14, weight = sqdist(G.nodes[12], G.nodes[14]))
G.add_edge(11, 7, weight = sqdist(G.nodes[11], G.nodes[7]))
G.add_edge(11, 10, weight = sqdist(G.nodes[11], G.nodes[10]))
# G.add_edge(11, 15, weight = sqdist(G.nodes[11], G.nodes[15]))
G.add_edge(11, 16, weight = sqdist(G.nodes[11], G.nodes[16]))
G.add_edge(11, 17, weight = sqdist(G.nodes[11], G.nodes[17]))
G.add_edge(10, 7, weight = sqdist(G.nodes[10], G.nodes[7]))
G.add_edge(10, 16, weight = sqdist(G.nodes[10], G.nodes[16]))
G.add_edge(10, 17, weight = sqdist(G.nodes[10], G.nodes[17]))
# G.add_edge(10, 18, weight = sqdist(G.nodes[10], G.nodes[18]))
# G.add_edge(10, 9, weight = sqdist(G.nodes[10], G.nodes[9]))
# G.add_edge(9, 17, weight = sqdist(G.nodes[9], G.nodes[17]))
# G.add_edge(9, 18, weight = sqdist(G.nodes[9], G.nodes[18]))
# G.add_edge(9, 19, weight = sqdist(G.nodes[9], G.nodes[19]))
# G.add_edge(9, 8, weight = sqdist(G.nodes[9], G.nodes[8]))
# G.add_edge(9, 1, weight = sqdist(G.nodes[9], G.nodes[1]))
G.add_edge(8, 18, weight = sqdist(G.nodes[8], G.nodes[18]))
G.add_edge(8, 19, weight = sqdist(G.nodes[8], G.nodes[19]))
G.add_edge(14, 15, weight = sqdist(G.nodes[14], G.nodes[15]))
G.add_edge(14, 24, weight = sqdist(G.nodes[14], G.nodes[24]))
G.add_edge(14, 25, weight = sqdist(G.nodes[14], G.nodes[25]))
G.add_edge(15, 25, weight = sqdist(G.nodes[15], G.nodes[25]))
G.add_edge(15, 24, weight = sqdist(G.nodes[15], G.nodes[24]))
G.add_edge(15, 23, weight = sqdist(G.nodes[15], G.nodes[23]))
# G.add_edge(15, 16, weight = sqdist(G.nodes[15], G.nodes[16]))
G.add_edge(16, 17, weight = sqdist(G.nodes[16], G.nodes[17]))
G.add_edge(16, 22, weight = sqdist(G.nodes[16], G.nodes[22]))
G.add_edge(16, 23, weight = sqdist(G.nodes[16], G.nodes[23]))
G.add_edge(16, 24, weight = sqdist(G.nodes[16], G.nodes[24]))
G.add_edge(17, 23, weight = sqdist(G.nodes[17], G.nodes[23]))
G.add_edge(17, 22, weight = sqdist(G.nodes[17], G.nodes[22]))
G.add_edge(17, 21, weight = sqdist(G.nodes[17], G.nodes[21]))
G.add_edge(17, 18, weight = sqdist(G.nodes[17], G.nodes[18]))
G.add_edge(18, 22, weight = sqdist(G.nodes[18], G.nodes[22]))
G.add_edge(18, 21, weight = sqdist(G.nodes[18], G.nodes[21]))
G.add_edge(18, 20, weight = sqdist(G.nodes[18], G.nodes[20]))
G.add_edge(18, 19, weight = sqdist(G.nodes[18], G.nodes[19]))
G.add_edge(19, 21, weight = sqdist(G.nodes[19], G.nodes[21]))
G.add_edge(19, 20, weight = sqdist(G.nodes[19], G.nodes[20]))
G.add_edge(20, 21, weight = sqdist(G.nodes[20], G.nodes[21]))
G.add_edge(20, 30, weight = sqdist(G.nodes[20], G.nodes[30]))
G.add_edge(20, 31, weight = sqdist(G.nodes[20], G.nodes[31]))
G.add_edge(21, 31, weight = sqdist(G.nodes[21], G.nodes[31]))
G.add_edge(21, 30, weight = sqdist(G.nodes[21], G.nodes[30]))
G.add_edge(21, 29, weight = sqdist(G.nodes[21], G.nodes[29]))
G.add_edge(21, 22, weight = sqdist(G.nodes[21], G.nodes[22]))
G.add_edge(22, 30, weight = sqdist(G.nodes[22], G.nodes[30]))
G.add_edge(22, 29, weight = sqdist(G.nodes[22], G.nodes[29]))
G.add_edge(22, 28, weight = sqdist(G.nodes[22], G.nodes[28]))
G.add_edge(22, 23, weight = sqdist(G.nodes[22], G.nodes[23]))
G.add_edge(23, 29, weight = sqdist(G.nodes[23], G.nodes[29]))
G.add_edge(23, 28, weight = sqdist(G.nodes[23], G.nodes[28]))
G.add_edge(23, 27, weight = sqdist(G.nodes[23], G.nodes[27]))
G.add_edge(23, 24, weight = sqdist(G.nodes[23], G.nodes[24]))
G.add_edge(24, 28, weight = sqdist(G.nodes[24], G.nodes[28]))
G.add_edge(24, 27, weight = sqdist(G.nodes[24], G.nodes[27]))
G.add_edge(24, 26, weight = sqdist(G.nodes[24], G.nodes[26]))
G.add_edge(24, 25, weight = sqdist(G.nodes[24], G.nodes[25]))
G.add_edge(25, 26, weight = sqdist(G.nodes[25], G.nodes[26]))
G.add_edge(25, 27, weight = sqdist(G.nodes[25], G.nodes[27]))
G.add_edge(26, 37, weight = sqdist(G.nodes[26], G.nodes[37]))
G.add_edge(26, 36, weight = sqdist(G.nodes[26], G.nodes[36]))
G.add_edge(26, 27, weight = sqdist(G.nodes[26], G.nodes[27]))
G.add_edge(27, 37, weight = sqdist(G.nodes[27], G.nodes[37]))
G.add_edge(27, 36, weight = sqdist(G.nodes[27], G.nodes[36]))
G.add_edge(27, 35, weight = sqdist(G.nodes[27], G.nodes[35]))
G.add_edge(27, 28, weight = sqdist(G.nodes[27], G.nodes[28]))
G.add_edge(28, 36, weight = sqdist(G.nodes[28], G.nodes[36]))
G.add_edge(28, 35, weight = sqdist(G.nodes[28], G.nodes[35]))
G.add_edge(28, 34, weight = sqdist(G.nodes[28], G.nodes[34]))
G.add_edge(28, 29, weight = sqdist(G.nodes[28], G.nodes[29]))
G.add_edge(29, 35, weight = sqdist(G.nodes[29], G.nodes[35]))
G.add_edge(29, 34, weight = sqdist(G.nodes[29], G.nodes[34]))
G.add_edge(29, 33, weight = sqdist(G.nodes[29], G.nodes[33]))
G.add_edge(29, 30, weight = sqdist(G.nodes[29], G.nodes[30]))
G.add_edge(30, 34, weight = sqdist(G.nodes[30], G.nodes[34]))
G.add_edge(30, 33, weight = sqdist(G.nodes[30], G.nodes[33]))
# G.add_edge(30, 32, weight = sqdist(G.nodes[30], G.nodes[32]))
G.add_edge(30, 31, weight = sqdist(G.nodes[30], G.nodes[31]))
# G.add_edge(31, 33, weight = sqdist(G.nodes[31], G.nodes[33]))
G.add_edge(31, 32, weight = sqdist(G.nodes[31], G.nodes[32]))
G.add_edge(32, 43, weight = sqdist(G.nodes[32], G.nodes[43]))
G.add_edge(32, 42, weight = sqdist(G.nodes[32], G.nodes[42]))
G.add_edge(32, 33, weight = sqdist(G.nodes[32], G.nodes[33]))
G.add_edge(33, 43, weight = sqdist(G.nodes[33], G.nodes[43]))
G.add_edge(33, 42, weight = sqdist(G.nodes[33], G.nodes[42]))
# G.add_edge(33, 41, weight = sqdist(G.nodes[33], G.nodes[41]))
G.add_edge(33, 34, weight = sqdist(G.nodes[33], G.nodes[34]))
# G.add_edge(34, 42, weight = sqdist(G.nodes[34], G.nodes[42]))
# G.add_edge(34, 41, weight = sqdist(G.nodes[34], G.nodes[41]))
G.add_edge(34, 40, weight = sqdist(G.nodes[34], G.nodes[40]))
G.add_edge(34, 35, weight = sqdist(G.nodes[34], G.nodes[35]))
G.add_edge(35, 41, weight = sqdist(G.nodes[35], G.nodes[41]))
G.add_edge(35, 40, weight = sqdist(G.nodes[35], G.nodes[40]))
G.add_edge(35, 39, weight = sqdist(G.nodes[35], G.nodes[39]))
G.add_edge(35, 36, weight = sqdist(G.nodes[35], G.nodes[36]))
G.add_edge(36, 40, weight = sqdist(G.nodes[36], G.nodes[40]))
G.add_edge(36, 39, weight = sqdist(G.nodes[36], G.nodes[39]))
G.add_edge(36, 38, weight = sqdist(G.nodes[36], G.nodes[38]))
G.add_edge(36, 37, weight = sqdist(G.nodes[36], G.nodes[37]))
G.add_edge(37, 39, weight = sqdist(G.nodes[37], G.nodes[39]))
G.add_edge(37, 38, weight = sqdist(G.nodes[37], G.nodes[38]))
G.add_edge(38, 49, weight = sqdist(G.nodes[38], G.nodes[49]))
G.add_edge(38, 38, weight = sqdist(G.nodes[38], G.nodes[48]))
G.add_edge(38, 39, weight = sqdist(G.nodes[38], G.nodes[39]))
G.add_edge(39, 49, weight = sqdist(G.nodes[39], G.nodes[49]))
G.add_edge(39, 48, weight = sqdist(G.nodes[39], G.nodes[48]))
G.add_edge(39, 47, weight = sqdist(G.nodes[39], G.nodes[47]))
G.add_edge(39, 40, weight = sqdist(G.nodes[39], G.nodes[40]))
G.add_edge(40, 48, weight = sqdist(G.nodes[40], G.nodes[48]))
G.add_edge(40, 47, weight = sqdist(G.nodes[40], G.nodes[47]))
G.add_edge(40, 46, weight = sqdist(G.nodes[40], G.nodes[46]))
G.add_edge(40, 41, weight = sqdist(G.nodes[40], G.nodes[41]))
G.add_edge(41, 47, weight = sqdist(G.nodes[41], G.nodes[47]))
G.add_edge(41, 46, weight = sqdist(G.nodes[41], G.nodes[46]))
# G.add_edge(41, 45, weight = sqdist(G.nodes[41], G.nodes[45]))
G.add_edge(41, 42, weight = sqdist(G.nodes[41], G.nodes[42]))
# G.add_edge(42, 46, weight = sqdist(G.nodes[42], G.nodes[46]))
G.add_edge(42, 45, weight = sqdist(G.nodes[42], G.nodes[45]))
G.add_edge(42, 44, weight = sqdist(G.nodes[42], G.nodes[44]))
G.add_edge(42, 43, weight = sqdist(G.nodes[42], G.nodes[43]))
G.add_edge(43, 45, weight = sqdist(G.nodes[43], G.nodes[45]))
G.add_edge(43, 44, weight = sqdist(G.nodes[43], G.nodes[44]))
G.add_edge(44, 55, weight = sqdist(G.nodes[44], G.nodes[55]))
G.add_edge(44, 54, weight = sqdist(G.nodes[44], G.nodes[54]))
G.add_edge(44, 45, weight = sqdist(G.nodes[44], G.nodes[45]))
G.add_edge(44, 55, weight = sqdist(G.nodes[44], G.nodes[55]))
G.add_edge(45, 55, weight = sqdist(G.nodes[45], G.nodes[55]))
G.add_edge(45, 54, weight = sqdist(G.nodes[45], G.nodes[54]))
# G.add_edge(45, 53, weight = sqdist(G.nodes[45], G.nodes[53]))
# G.add_edge(45, 46, weight = sqdist(G.nodes[45], G.nodes[46]))
# G.add_edge(46, 54, weight = sqdist(G.nodes[46], G.nodes[54]))
G.add_edge(46, 53, weight = sqdist(G.nodes[46], G.nodes[53]))
G.add_edge(46, 52, weight = sqdist(G.nodes[46], G.nodes[52]))
G.add_edge(46, 47, weight = sqdist(G.nodes[46], G.nodes[47]))
G.add_edge(47, 53, weight = sqdist(G.nodes[47], G.nodes[53]))
G.add_edge(47, 52, weight = sqdist(G.nodes[47], G.nodes[52]))
# G.add_edge(47, 51, weight = sqdist(G.nodes[47], G.nodes[51]))
G.add_edge(47, 48, weight = sqdist(G.nodes[47], G.nodes[48]))
# G.add_edge(48, 52, weight = sqdist(G.nodes[48], G.nodes[52]))
G.add_edge(48, 51, weight = sqdist(G.nodes[48], G.nodes[51]))
G.add_edge(48, 50, weight = sqdist(G.nodes[48], G.nodes[50]))
G.add_edge(48, 49, weight = sqdist(G.nodes[48], G.nodes[49]))
G.add_edge(49, 51, weight = sqdist(G.nodes[49], G.nodes[51]))
G.add_edge(49, 50, weight = sqdist(G.nodes[49], G.nodes[50]))
G.add_edge(50, 61, weight = sqdist(G.nodes[50], G.nodes[61]))
G.add_edge(50, 60, weight = sqdist(G.nodes[50], G.nodes[60]))
G.add_edge(50, 51, weight = sqdist(G.nodes[50], G.nodes[51]))
G.add_edge(51, 61, weight = sqdist(G.nodes[51], G.nodes[61]))
G.add_edge(51, 60, weight = sqdist(G.nodes[51], G.nodes[60]))
# G.add_edge(51, 59, weight = sqdist(G.nodes[51], G.nodes[59]))
# G.add_edge(51, 52, weight = sqdist(G.nodes[51], G.nodes[52]))
# G.add_edge(52, 60, weight = sqdist(G.nodes[52], G.nodes[60]))
G.add_edge(52, 59, weight = sqdist(G.nodes[52], G.nodes[59]))
G.add_edge(52, 58, weight = sqdist(G.nodes[52], G.nodes[58]))
G.add_edge(52, 53, weight = sqdist(G.nodes[52], G.nodes[53]))
G.add_edge(53, 59, weight = sqdist(G.nodes[53], G.nodes[59]))
G.add_edge(53, 58, weight = sqdist(G.nodes[53], G.nodes[58]))
G.add_edge(53, 57, weight = sqdist(G.nodes[53], G.nodes[57]))
G.add_edge(53, 54, weight = sqdist(G.nodes[53], G.nodes[54]))
G.add_edge(54, 58, weight = sqdist(G.nodes[54], G.nodes[58]))
G.add_edge(54, 57, weight = sqdist(G.nodes[54], G.nodes[57]))
# G.add_edge(54, 56, weight = sqdist(G.nodes[54], G.nodes[56]))
G.add_edge(54, 55, weight = sqdist(G.nodes[54], G.nodes[55]))
G.add_edge(55, 57, weight = sqdist(G.nodes[55], G.nodes[57]))
# G.add_edge(55, 56, weight = sqdist(G.nodes[55], G.nodes[56]))
# G.add_edge(56, 67, weight = sqdist(G.nodes[56], G.nodes[67]))
# G.add_edge(56, 66, weight = sqdist(G.nodes[56], G.nodes[66]))
# G.add_edge(56, 57, weight = sqdist(G.nodes[56], G.nodes[57]))
G.add_edge(57, 67, weight = sqdist(G.nodes[57], G.nodes[67]))
G.add_edge(57, 66, weight = sqdist(G.nodes[57], G.nodes[66]))
G.add_edge(57, 65, weight = sqdist(G.nodes[57], G.nodes[65]))
G.add_edge(57, 58, weight = sqdist(G.nodes[57], G.nodes[58]))
G.add_edge(58, 66, weight = sqdist(G.nodes[58], G.nodes[66]))
G.add_edge(58, 65, weight = sqdist(G.nodes[58], G.nodes[65]))
G.add_edge(58, 64, weight = sqdist(G.nodes[58], G.nodes[64]))
G.add_edge(58, 59, weight = sqdist(G.nodes[58], G.nodes[59]))
G.add_edge(59, 65, weight = sqdist(G.nodes[59], G.nodes[65]))
G.add_edge(59, 64, weight = sqdist(G.nodes[59], G.nodes[64]))
G.add_edge(59, 63, weight = sqdist(G.nodes[59], G.nodes[63]))
G.add_edge(59, 60, weight = sqdist(G.nodes[59], G.nodes[60]))
G.add_edge(60, 64, weight = sqdist(G.nodes[60], G.nodes[64]))
G.add_edge(60, 63, weight = sqdist(G.nodes[60], G.nodes[63]))
G.add_edge(60, 62, weight = sqdist(G.nodes[60], G.nodes[62]))
G.add_edge(60, 61, weight = sqdist(G.nodes[60], G.nodes[61]))
G.add_edge(61, 63, weight = sqdist(G.nodes[61], G.nodes[63]))
G.add_edge(61, 62, weight = sqdist(G.nodes[61], G.nodes[62]))
G.add_edge(62, 63, weight = sqdist(G.nodes[62], G.nodes[63]))
G.add_edge(63, 64, weight = sqdist(G.nodes[63], G.nodes[64]))
G.add_edge(64, 65, weight = sqdist(G.nodes[64], G.nodes[65]))
G.add_edge(65, 66, weight = sqdist(G.nodes[65], G.nodes[66]))
G.add_edge(66, 67, weight = sqdist(G.nodes[66], G.nodes[67]))








print('test')

# CREATE PATH TO MOVE AROUND FIELD WITHOUT HITTING OBSTACLES
movingPath = nx.astar_path(G, 0, 62, heuristic=None, weight="weight")
secondSection = nx.astar_path(G, 62, 64, heuristic=None, weight="weight")
for node in secondSection: movingPath.append(node)
thirdSection = nx.astar_path(G, 64, 58, heuristic=None, weight="weight")
for node in thirdSection: movingPath.append(node)
fourthSection = nx.astar_path(G, 58, 54, heuristic=None, weight="weight")
for node in fourthSection: movingPath.append(node)
fifthSection = nx.astar_path(G, 54, 44, heuristic=None, weight="weight")
for node in fifthSection: movingPath.append(node)
sixthSection = nx.astar_path(G, 44, 15, heuristic=None, weight="weight")
for node in sixthSection: movingPath.append(node)
seventhSection = nx.astar_path(G, 15, 37, heuristic=None, weight="weight")
for node in seventhSection: movingPath.append(node)
eighthSection = nx.astar_path(G, 37, 3, heuristic=None, weight="weight")
for node in eighthSection: movingPath.append(node)
path = movingPath

movingPath2 = nx.astar_path(G, 0, 2, heuristic=None, weight="weight")
secondSection = nx.astar_path(G, 2, 10, heuristic=None, weight="weight")
for node in secondSection: movingPath2.append(node)
thirdSection = nx.astar_path(G, 10, 12, heuristic=None, weight="weight")
for node in thirdSection: movingPath2.append(node)
fourthSection = nx.astar_path(G, 12, 25, heuristic=None, weight="weight")
for node in fourthSection: movingPath2.append(node)
fifthSection = nx.astar_path(G, 25, 34, heuristic=None, weight="weight")
for node in fifthSection: movingPath2.append(node)
sixthSection = nx.astar_path(G, 34, 49, heuristic=None, weight="weight")
for node in sixthSection: movingPath2.append(node)
seventhSection = nx.astar_path(G, 49, 66, heuristic=None, weight="weight")
for node in seventhSection: movingPath2.append(node)
eighthSection = nx.astar_path(G, 66, 3, heuristic=None, weight="weight")
for node in eighthSection: movingPath2.append(node)

movingPath3 = nx.astar_path(G, 0, 32, heuristic=None, weight="weight")
secondSection = nx.astar_path(G, 32, 52, heuristic=None, weight="weight")
for node in secondSection: movingPath3.append(node)
thirdSection = nx.astar_path(G, 52, 16, heuristic=None, weight="weight")
for node in thirdSection: movingPath3.append(node)
fourthSection = nx.astar_path(G, 16, 26, heuristic=None, weight="weight")
for node in fourthSection: movingPath3.append(node)
fifthSection = nx.astar_path(G, 26, 30, heuristic=None, weight="weight")
for node in fifthSection: movingPath3.append(node)
sixthSection = nx.astar_path(G, 30, 50, heuristic=None, weight="weight")
for node in sixthSection: movingPath3.append(node)
seventhSection = nx.astar_path(G, 50, 55, heuristic=None, weight="weight")
for node in seventhSection: movingPath3.append(node)
eighthSection = nx.astar_path(G, 55, 3, heuristic=None, weight="weight")
for node in eighthSection: movingPath3.append(node)

thePaths = [movingPath, movingPath2, movingPath3]
path = movingPath
print(path)




# Camera setup
cap = cv.VideoCapture('http://192.168.0.209:3005/stream.mjpg')

if not cap.isOpened():
    print("Error opening video stream or file")
    exit()




# Move to duck
def duckSpeed(x, area, k_v, k_pr):
    desX = 320

    errd = 100

    errx = desX - x

    v = k_v * errd

    omega = k_pr * errx

    u = np.array([v-omega, v+omega])
    u[u > 1500] = 1500
    u[u < -1500] = -1500

    return u




if __name__ == "__main__":
    import socket
    ## getting the hostname by socket.gethostname() method
    hostname = socket.gethostname()
    ## getting the IP address using socket.gethostbyname() method
    ip_address = socket.gethostbyname(hostname)
    print(ip_address)
    
    clientAddress = "192.168.0.24"
    optitrackServerAddress = "192.168.0.04"
    robot_id = 309

    # This will create a new NatNet client
    streaming_client = NatNetClient()
    streaming_client.set_client_address(clientAddress)
    streaming_client.set_server_address(optitrackServerAddress)
    streaming_client.set_use_multicast(True)
    # Configure the streaming client to call our rigid body handler on the emulator to send data out.
    streaming_client.rigid_body_listener = receive_rigid_body_frame

    # Start up the streaming client now that the callbacks are set up.
    # This will run perpetually, and operate on a separate thread.
    is_running = streaming_client.run()

    # Point index
    p = 0
    p2 = 0

    # P-Controller gain
    k_v  = 1700
    k_pr = 200

    # Denotes if captured a duck, 0 if not and 1 if did
    captured_duck = 0

    # Denotes if currently capturing a duck, 0 if not 1 if is
    capturing_duck = 0

    t = 0.
    while is_running:
        try:
            if robot_id in positions:

                # Loop conditions
                if p > len(path) - 1 and captured_duck == 0:
                    print('path loop resetting')
                    if p2 >= 2:
                        p2 = 0
                    else:
                        p2 += 1
                    path = thePaths[p2]
                    command = 'CMD_MOTOR#-500#-500#-500#-500\n'
                    s.send(command.encode('utf-8'))
                    time.sleep(2)
                    p = 0
                print('Current Node:', path[p])

                # if p > len(path) - 1 and captured_duck == 1:
                #     print('return loop ended')
                #     captured_duck = 0
                #     command = 'CMD_MOTOR#-500#-500#-500#-500\n'
                #     s.send(command.encode('utf-8'))
                #     time.sleep(2)

                #     command = 'CMD_MOTOR#00#00#00#00\n'
                #     s.send(command.encode('utf-8'))

                #     # Find the nearest node
                #     nearest_node = 0
                #     for node in G:
                #         if sqdistOpti(G.nodes[node], positions[robot_id][0], positions[robot_id][1]) < sqdistOpti(G.nodes[nearest_node], positions[robot_id][0], positions[robot_id][1]):
                #             nearest_node = G.nodes[node]

                #     # Replace with path that was already in place to move around area
                #     p = 0
                #     path = movingPath




                # Get the camera capture
                print('CAMERA: capturing frame')
                ret, frame = cap.read()
                



                # It converts the BGR color space of image to HSV color space
                print('CAMERA: converting frame color')
                hsv = cv.cvtColor(frame, cv.COLOR_BGR2HSV)
                



                # Threshold of blue in HSV space
                print('CAMERA: creating yellow threshold')
                lower_yellow = np.array([20, 100, 100])
                upper_yellow = np.array([30, 255, 255])
                



                # preparing the mask to overlay
                print('CAMERA: preparing overlay')
                mask = cv.inRange(hsv, lower_yellow, upper_yellow)

                result = cv.bitwise_and(frame, frame, mask = mask)
                gray = cv.cvtColor(result, cv.COLOR_BGR2GRAY)
                gray = cv.medianBlur(gray, 5)
                    
                rows = gray.shape[0]
                
                params = cv.SimpleBlobDetector_Params()
                params.filterByArea = True
                params.minArea = 3000
                params.maxArea = 100000
                params.filterByColor = False
                params.filterByCircularity = True
                params.minCircularity = 0.5
                params.maxCircularity = 1
                params.filterByConvexity = False
                params.filterByInertia = False
                
                



                # Set up the detector with default parameters.
                print('CAMERA: creating simple blob detector')
                detector = cv.SimpleBlobDetector_create(params)
                



                # Detects duck and starts capturing process if one is detected
                keypoints = detector.detect(gray)

                # # If there is a duck in frame
                # if len(keypoints) >= 1 and captured_duck == 0:
                #     print('RETRIEVAL: duck detected, going to retrieval mode')
                #     command = 'CMD_MOTOR#00#00#00#00\n'
                #     s.send(command.encode('utf-8'))
                #     max = keypoints[0]
                #     for keypoint in keypoints:
                #         if keypoint.size > max.size:
                #             max = keypoint
                #     if(max.size > 120):
                #         print('RETRIEVAL: duck retrieved, going to safety area')
                #         command = 'CMD_MOTOR#500#500#500#500\n'
                #         s.send(command.encode('utf-8'))
                #         time.sleep(1)
                #         capturing_duck = 0
                #         captured_duck = 1

                #         print('RETRIEVAL: finding nearest node to robot')
                #         # Find the nearest node
                #         nearest_node = 0
                #         for node in G:
                #             if sqdistOpti(G.nodes[node], positions[robot_id][0], positions[robot_id][1]) < sqdistOpti(G.nodes[nearest_node], positions[robot_id][0], positions[robot_id][1]):
                #                 nearest_node = node

                #         print('RETRIEVAL: creating shortest path to drop off zone')
                #         # Path to get to the center of the circle
                #         p = 0
                #         path = nx.astar_path(G, nearest_node, 7, heuristic=None, weight="weight")
                #     else:
                #         print('RETRIEVAL: duck not captured yet, moving towards duck')
                #         capturing_duck = 1
                #         u = duckSpeed(max.pt[0], max.size, k_v, k_pr)
                #         command = 'CMD_MOTOR#%d#%d#%d#%d\n'%(u[0], u[0], u[1], u[1])
                #         s.send(command.encode('utf-8'))
                # if len(keypoints) < 1 and capturing_duck == 1:
                #     command = 'CMD_MOTOR#-300#-300#300#300\n'
                #     s.send(command.encode('utf-8'))




                # if capturing_duck == 0:
                #     if captured_duck == 1: print('RETRIEVAL: moving along path towards safe zone')
                    # else: print('MOVEMENT: moving along node path to find ducks')
                    # Rotation of the robot
                theta = rotations[robot_id] * pi/180



                # Goal distance
                x = G.nodes[path[p]]["x"] 
                y = G.nodes[path[p]]["y"]




                # P control for x
                errx = x - positions[robot_id][0]




                # P control for y
                erry = y - positions[robot_id][1]




                print('Robot Position: (%f,%f)'%(positions[robot_id][0],positions[robot_id][1]))
                print('Distance to Next Node: (%f,%f)'%(errx,erry))




                # P control for rotation
                alpha = atan2(erry, errx)
                errw  = degrees(atan2(sin(alpha-theta), cos(alpha-theta)))
                print(errw)
                omega = k_pr*errw



                # Velocity and Angular Velocity
                v = k_v*(sqrt(errx**2 + erry**2))
                u = np.array([v-omega, v+omega])
                # set bound of motor input
                u[u > 1500] = 1500
                u[u < -1500] = -1500




                # Send control input to the motors
                command = 'CMD_MOTOR#%d#%d#%d#%d\n'%(u[0], u[0], u[1], u[1])
                print(command)
                s.send(command.encode('utf-8'))



                # Move to next node
                if sqrt(errx**2 + erry**2) < 0.25:
                    p += 1
                
                print('FRAME: showing camera view')
                frame_with_keypoints = cv.drawKeypoints(result, keypoints, np.array([]), (0,0,255), cv.DRAW_MATCHES_FLAGS_DRAW_RICH_KEYPOINTS)
                cv.imshow("Keypoints", frame_with_keypoints)
                if cv.waitKey(1) & 0xFF == ord('q'):
                    break



                # Break between executions
                # time.sleep(.1)
                t += .02
                


        # Ctrl + C to kill program
        except KeyboardInterrupt:
            # STOP
            command = 'CMD_MOTOR#00#00#00#00\n'
            s.send(command.encode('utf-8'))

            # Close the connection
            s.shutdown(2)
            s.close()
            streaming_client.shutdown()
            cv.destroyAllWindows()
            print('cv.destroyAllWindows()')




    # STOP
    command = 'CMD_MOTOR#00#00#00#00\n'
    s.send(command.encode('utf-8'))




    # Close the connection
    s.shutdown(2)
    s.close()
    streaming_client.shutdown()
    cv.destroyAllWindows()
    print('cv.destroyAllWindows()')