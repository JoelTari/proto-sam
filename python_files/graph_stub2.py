#!/usr/bin/python3

import paho.mqtt.client as mqtt
import time
import json
import numpy as np
import copy
import math
import random as rd
import numpy.random as nprd

# Fake map for informel comms

# ----------------------------------------------------------------------------
#                           Globals
# ----------------------------------------------------------------------------
world = {
    "robots":
    [
        {
            "robot_id": "r1",
            "state": {"x": 38.6, "y": 19, "th": 55*math.pi/180},
            "sensor": {"range": 12, "angle_coverage": 0.75},
            'state_history':
            [
                [6.667185306549072,  12.38792896270752],
                [7.736385345458984,  12.450823783874512],
                [8.805584907531738,  12.513717651367188],
                [10.755302429199219,  12.38792896270752],
                [11.698714256286621,  12.073458671569824],
                [13.711325645446777,  12.073458671569824],
                [15.409466743469238,  12.010564804077148],
                [18.239702224731445,  12.073458671569824],
                [20.503889083862305,  12.325035095214844],
                [22.076242446899414,  12.450823783874512],
                [22.95676040649414,  12.639505386352539],
                [24.969371795654297,  12.639505386352539],
                [26.22725486755371,  12.828187942504883],
                [27.799606323242188,  13.268446922302246],
                [28.99459457397461,  13.70870590209961],
                [30.315370559692383,  14.274752616882324],
                [31.63614845275879,  14.840799331665039],
                [33.39718246459961,  15.532634735107422],
                [34.59217071533203,  16.224470138549805],
                [35.85005187988281,  17.04209327697754],
                [36.60478210449219,  17.67103385925293],
                [37.42240524291992,  18.1741886138916],
                [38.1771354675293,  18.67734146118164],
            ],
            'seq':0,
        },
        {
            "robot_id": "r2",
            "state": {"x": 27.3, "y": 48.3, "th": 0},
            "sensor": {"range": 12, "angle_coverage": 0.75},
            'state_history':
            [
                [7.232004165649414, 51.19482421875],
                [8.096797943115234, 51.03759002685547],
                [8.725739479064941, 50.88035202026367],
                [10.691180229187012, 50.09417724609375],
                [11.398738861083984, 49.85832595825195],
                [12.184915542602539, 49.701087951660156],
                [13.364179611206055, 49.62247085571289],
                [14.936532974243164, 49.2293815612793],
                [16.666120529174805, 48.914913177490234],
                [18.002620697021484, 48.83629608154297],
                [19.496355056762695, 48.67905807495117],
                [21.697649002075195, 48.52182388305664],
                [22.48382568359375, 48.36458969116211],
                [23.34861946105957, 48.20735549926758],
                [23.977561950683594, 48.12873458862305],
                [24.68511962890625, 48.05011749267578],
                [25.471296310424805, 48.05011749267578],
                [26.336090087890625, 48.20735549926758],
                [26.88641357421875, 48.20735549926758],
            ],
            'seq':0,
        },
        {
            "robot_id": "r3",
            "state": {"x": 70.7, "y": 29.9, "th": -175*math.pi/180},
            "sensor": {"range": 12, "angle_coverage": 0.5},
            'state_history':
            [
                [90.7566146850586, 42.404144287109375],
                [90.7566146850586, 41.71230697631836],
                [90.63082122802734, 41.02047348022461],
                [90.63082122802734, 40.39153289794922],
                [90.56793212890625, 39.76259231567383],
                [90.442138671875, 38.81917953491211],
                [90.12767028808594, 37.56129837036133],
                [90.12767028808594, 36.17762756347656],
                [90.12767028808594, 34.60527420043945],
                [89.75030517578125, 33.9134407043457],
                [87.48612213134766, 31.397674560546875],
                [86.98296356201172, 30.705839157104492],
                [86.79428100585938, 30.454261779785156],
                [86.22823333740234, 30.13979148864746],
                [85.5363998413086, 29.573745727539062],
                [84.08983612060547, 29.51085090637207],
                [82.45458984375, 29.38506317138672],
                [80.37908172607422, 29.38506317138672],
                [78.9325180053711, 29.51085090637207],
                [77.42305755615234, 29.38506317138672],
                [76.47965240478516, 29.447956085205078],
                [75.78781127929688, 29.51085090637207],
                [75.22176361083984, 29.573745727539062],
                [74.02677917480469, 29.573745727539062],
                [73.1462631225586, 29.699533462524414],
                [72.39153289794922, 29.699533462524414],
                [71.57390594482422, 29.762428283691406],
            ],
            'seq':0,
        },
    ],
    "landmarks":
    [
        {"landmark_id": "l1", "state": {"x": 9, "y": 46}},
        {"landmark_id": "l2", "state": {"x": 13, "y": 4}},
        {"landmark_id": "l3", "state": {"x": 13.5, "y": 55}},
        {"landmark_id": "l4", "state": {"x": 14, "y": 30}},
        {"landmark_id": "l5", "state": {"x": 27, "y": 31}},
        {"landmark_id": "l6", "state": {"x": 52.2, "y": 36.5}},
        {"landmark_id": "l7", "state": {"x": 45, "y": 10}},
        {"landmark_id": "l8", "state": {"x": 45, "y": 50}},
        {"landmark_id": "l9", "state": {"x": 61, "y": 11}},
        {"landmark_id": "l10", "state": {"x": 51.2, "y": 31}},
        {"landmark_id": "l11", "state": {"x": 67, "y": 27}},
        {"landmark_id": "l12", "state": {"x": 71, "y": 51}},
        {"landmark_id": "l13", "state": {"x": 77, "y": 16}},
        {"landmark_id": "l14", "state": {"x": 80, "y": 23}},
        {"landmark_id": "l15", "state": {"x": 84, "y": 35}},
        {"landmark_id": "l16", "state": {"x": 96, "y": 56}},
    ]
}

world2 = {
    "robots":
    [
        {
            "robot_id": "r1",
            "state": {"x": 42.76, "y": 29.57, "th": 15*math.pi/180},
            "sensor": {"range": 12, "angle_coverage": 0.75},
            'state_history':
            [
                [6.667185306549072,  12.38792896270752],
                [7.736385345458984,  12.450823783874512],
                [8.805584907531738,  12.513717651367188],
                [10.755302429199219,  12.38792896270752],
                [11.698714256286621,  12.073458671569824],
                [13.711325645446777,  12.073458671569824],
                [15.409466743469238,  12.010564804077148],
                [18.239702224731445,  12.073458671569824],
                [20.503889083862305,  12.325035095214844],
                [22.076242446899414,  12.450823783874512],
                [22.95676040649414,  12.639505386352539],
                [24.969371795654297,  12.639505386352539],
                [26.22725486755371,  12.828187942504883],
                [27.799606323242188,  13.268446922302246],
                [28.99459457397461,  13.70870590209961],
                [30.315370559692383,  14.274752616882324],
                [31.63614845275879,  14.840799331665039],
                [33.39718246459961,  15.532634735107422],
                [34.59217071533203,  16.224470138549805],
                [35.85005187988281,  17.04209327697754],
                [36.60478210449219,  17.67103385925293],
                [37.42240524291992,  18.1741886138916],
                [38.1771354675293,  18.67734146118164],
                [38.914913177490234, 19.433298110961914],
                [38.9935302734375, 20.062238693237305],
                [38.52182388305664, 20.691179275512695],
                [34.43370819091797, 22.027679443359375],
                [30.03112030029297, 22.892473220825195],
                [27.12226676940918, 25.25100326538086],
                [27.82982635498047, 28.15985679626465],
                [31.76070785522461, 28.78879737854004],
                [36.713619232177734, 29.2605037689209],
                [41.19482421875, 29.41773796081543],
            ],
            'seq':1,
        },
        {
            "robot_id": "r2",
            "state": {"x": 52.2, "y": 41.05, "th": -95*math.pi/180},
            "sensor": {"range": 12, "angle_coverage": 0.75},
            'state_history':
            [
                [7.232004165649414, 51.19482421875],
                [8.096797943115234, 51.03759002685547],
                [8.725739479064941, 50.88035202026367],
                [10.691180229187012, 50.09417724609375],
                [11.398738861083984, 49.85832595825195],
                [12.184915542602539, 49.701087951660156],
                [13.364179611206055, 49.62247085571289],
                [14.936532974243164, 49.2293815612793],
                [16.666120529174805, 48.914913177490234],
                [18.002620697021484, 48.83629608154297],
                [19.496355056762695, 48.67905807495117],
                [21.697649002075195, 48.52182388305664],
                [22.48382568359375, 48.36458969116211],
                [23.34861946105957, 48.20735549926758],
                [23.977561950683594, 48.12873458862305],
                [24.68511962890625, 48.05011749267578],
                [25.471296310424805, 48.05011749267578],
                [26.336090087890625, 48.20735549926758],
                [26.88641357421875, 48.20735549926758],
                [29.08770751953125, 48.36458969116211],
                [31.91794204711914, 48.05011749267578],
                [36.084678649902344, 47.57841110229492],
                [39.62247085571289, 47.65703201293945],
                [44.65399932861328, 48.12873458862305],
                [47.248382568359375, 49.30800247192383],
                [49.921382904052734, 51.03759002685547],
                [52.51576614379883, 50.40864944458008],
                [53.852264404296875, 48.05011749267578],
                [53.06608963012695, 44.11923599243164],
            ],
            'seq':1,
        },
        {
            "robot_id": "r3",
            "state": {"x": 56.83, "y": 30.28, "th": 135*math.pi/180},
            "sensor": {"range": 12, "angle_coverage": 0.5},
            'state_history':
            [
                [90.7566146850586, 42.404144287109375],
                [90.7566146850586, 41.71230697631836],
                [90.63082122802734, 41.02047348022461],
                [90.63082122802734, 40.39153289794922],
                [90.56793212890625, 39.76259231567383],
                [90.442138671875, 38.81917953491211],
                [90.12767028808594, 37.56129837036133],
                [90.12767028808594, 36.17762756347656],
                [90.12767028808594, 34.60527420043945],
                [89.75030517578125, 33.9134407043457],
                [87.48612213134766, 31.397674560546875],
                [86.98296356201172, 30.705839157104492],
                [86.79428100585938, 30.454261779785156],
                [86.22823333740234, 30.13979148864746],
                [85.5363998413086, 29.573745727539062],
                [84.08983612060547, 29.51085090637207],
                [82.45458984375, 29.38506317138672],
                [80.37908172607422, 29.38506317138672],
                [78.9325180053711, 29.51085090637207],
                [77.42305755615234, 29.38506317138672],
                [76.47965240478516, 29.447956085205078],
                [75.78781127929688, 29.51085090637207],
                [75.22176361083984, 29.573745727539062],
                [74.02677917480469, 29.573745727539062],
                [73.1462631225586, 29.699533462524414],
                [72.39153289794922, 29.699533462524414],
                [71.57390594482422, 29.762428283691406],
                [68.71099853515625, 29.732208251953125],
                [66.35247039794922, 29.496355056762695],
                [64.46564483642578, 29.103267669677734],
                [59.90582275390625, 28.631561279296875],
                [58.49070358276367, 28.94603157043457],
                [58.01900100708008, 29.339120864868164],
                [57.54729461669922, 29.574974060058594],
            ],
            'seq':1,
        },
    ],
}


# some mqtt related globals
broker = 'localhost'
request_ground_truth_topic = 'request_ground_truth'
ground_truth_topic = 'ground_truth'
request_estimation_graph_topic = 'request_estimation'
estimation_graph_topic = 'estimation'

# ----------------------------------------------------------------------------
#                           Some utility functions
# ----------------------------------------------------------------------------


def rSig(m=0.6, M=1.2):
    return rd.uniform(m, M)


def rSigMul(mul=1):
    return rd.uniform(0.6, 0.8)*mul


def rRot():
    return rd.uniform(0, math.pi/2)
# randomness at iteration


def rDSig(m=0.3, M=0.5):
    return rd.uniform(m, M)


def rDRot():
    return rd.uniform(-math.pi/3, math.pi/3)


def rXY(sig=1.5):
    return rd.normalvariate(0, sig)


def fd_landmark_by_id(ArrayOfLandmarks, lid):
    return next(item for item in ArrayOfLandmarks if item["landmark_id"] == lid)
    # return next(filter(lambda x: x['landmark_id'] == lid, ArrayOfLandmarks))


def getIdxM(margs, vid):
    return next(i for i, m in enumerate(margs) if m["var_id"] == vid)


def modify_covs(m):
    m['covariance']['sigma'][0] *= rDSig()
    m['covariance']['sigma'][1] *= rDSig()
    m['covariance']['rot'] += rDRot()
    return m


# ----------------------------------------------------------------------------
#                           Globals: Fake MultiMap
# ----------------------------------------------------------------------------


full_estimations1 = [
    {
        'header': {
            'robot_id': 'r1',
            'seq': 0,
        },
        'rt_estimate': {'type': 'relative',  # absolute or relative to last state (last pose mst exist)
                     "state": {"x": 40.3, "y": 17.4, "th": 47*math.pi/180},
                     'covariance': {'sigma': [2, 0.9], 'rot': 0.314}},
        'last_pose': {'last_pose_id': 'x4'},
        # TODO : plural graph with graph id for each graph and a header designating
        #        the main hypothesis
        # 'main_hypothesis_graph_id':'g1'
        'graph': {  # TODO: plural-> array, one header per element and the following encapsulated in 'data',
            #
            'marginals': [
                {'var_id': 'x0', 'mean': {'x': 6.5, 'y': 12},
                 'covariance': {'sigma': [rSig(0.2, 0.5), rSig(0.1, 0.3)], 'rot':rRot()}},
                {'var_id': 'x1', 'mean': {'x': 14, 'y': 10.6},
                 'covariance': {'sigma': [rSigMul(), rSigMul()], 'rot':rRot()}},
                {'var_id': 'x2', 'mean': {'x': 21.1, 'y': 10.5},
                 'covariance': {'sigma': [rSigMul(3/2), rSigMul(1)], 'rot':rRot()}},
                {'var_id': 'x3', 'mean': {'x': 27.7, 'y': 10.9},
                 'covariance': {'sigma': [rSigMul(2), rSigMul(3/2)], 'rot':rRot()}},
                {'var_id': 'x4', 'mean': {'x': 34.88, 'y': 13.8},
                 'covariance': {'sigma': [rSigMul(2.8), rSigMul(2.5)], 'rot':rRot()}},
                {'var_id': 'l2', 'mean': {
                    'x':
                        fd_landmark_by_id(world['landmarks'], 'l2')['state']['x']+rXY(), 'y':
                        fd_landmark_by_id(world['landmarks'], 'l2')[
                            'state']['y']+rXY()
                }, 'covariance': {'sigma': [rSigMul(), rSigMul()], 'rot':rRot()}},
                {'var_id': 'l7', 'mean': {
                    'x': 45.9, 'y': 8.75
                }, 'covariance': {'sigma': [rSigMul(2), rSigMul(1.5)], 'rot':rRot()}},
            ],

            'factors': [
                {'factor_id': 'f0',
                 'type': 'ini_position',
                 'vars_id': ['x0'],
                 },
                {'factor_id': 'f1',
                 'type': 'rb',
                 'vars_id': ['l2', 'x0'],
                 },
                {'factor_id': 'f2',
                 'type': 'odometry',
                 'vars_id': ['x1', 'x0'],
                 },
                {'factor_id': 'f3',
                 'type': 'odometry',
                 'vars_id': ['l2', 'x1'],
                 },
                {'factor_id': 'f4',
                 'type': 'odometry',
                 'vars_id': ['x2', 'l2'],
                 },
                {'factor_id': 'f5',
                 'type': 'odometry',
                 'vars_id': ['x1', 'x2'],
                 },
                {'factor_id': 'f6',
                 'type': 'odometry',
                 'vars_id': ['x2', 'x3'],
                 },
                {'factor_id': 'f7',
                 'type': 'odometry',
                 'vars_id': ['x4', 'x3'],
                 },
                {'factor_id': 'f8',
                 'type': 'odometry',
                 'vars_id': ['x4', 'l7'],
                 },
            ],
        }
    },
    # The second robot
    {
        'header': {
            'robot_id': 'r2',
            'seq': 0,
        },
        'rt_estimate': {'type': 'relative',  # absolute or relative to last state (last pose mst exist)
                     "state": {"x": 26.5, "y": 50.46, "th": -0.5},
                     'covariance': {'sigma': [2, 0.9], 'rot': 0.614}},
        'last_pose': {'last_pose_id': 'x2'},
        'graph': {
            'marginals': [
                {'var_id': 'x0', 'mean': {'x': 7.3, 'y': 51.6},
                 'covariance': {'sigma': [rSigMul(), rSigMul()], 'rot':rRot()}},
                {'var_id': 'x1', 'mean': {'x': 15, 'y': 50.5},
                 'covariance': {'sigma': [rSigMul(), rSigMul()], 'rot':rRot()}},
                {'var_id': 'x2', 'mean': {'x': 21.12, 'y': 50.1},
                 'covariance': {'sigma': [rSigMul(1.1), rSigMul(1.2)], 'rot':rRot()}},
                {'var_id': 'l1', 'mean': {
                    'x':
                        fd_landmark_by_id(world['landmarks'], 'l1')['state']['x']+rXY(), 'y':
                        fd_landmark_by_id(world['landmarks'], 'l1')[
                            'state']['y']+rXY()
                },
                    'covariance': {'sigma': [rSigMul(1.5), rSigMul(0.75)], 'rot':rRot()}},
                {'var_id': 'l3', 'mean': {
                    'x':
                        fd_landmark_by_id(world['landmarks'], 'l3')['state']['x']+rXY(), 'y':
                        fd_landmark_by_id(world['landmarks'], 'l3')[
                            'state']['y']+rXY()
                },
                    'covariance': {'sigma': [rSigMul(0.8), rSigMul(1)], 'rot':rRot()}},
            ],

            'factors': [
                {'factor_id': 'f0',
                 'type': 'ini_position',
                 'vars_id': ['x0'],
                 },
                {'factor_id': 'f1',
                 'type': 'range-bearing',
                 'vars_id': ['l1', 'x0'],
                 },
                {'factor_id': 'f2',
                 'type': 'range-bearing',
                 'vars_id': ['l3', 'x0'],
                 },
                {'factor_id': 'f3',
                 'type': 'odometry',
                 'vars_id': ['x0', 'x1'],
                 },
                {'factor_id': 'f4',
                 'type': 'range-bearing',
                 'vars_id': ['x1', 'l3'],
                 },
                {'factor_id': 'f5',
                 'type': 'odometry',
                 'vars_id': ['x1', 'x2'],
                 },
            ],
        }
    },
    # the third robot
    {
        'header': {
            'robot_id': 'r3',
            'seq': 0,
        },
        'rt_estimate': {'type': 'relative',  # absolute or relative to last state (last pose mst exist)
                     "state": {"x": 71.3, "y": 30.5, "th": 2.9},
                     'covariance': {'sigma': [2, 0.9], 'rot': 0.914}},
        'last_pose': {'last_pose_id': 'x2'},
        'last_pose': {'last_pose_id': 'x3'},
        'graph': {
            'marginals': [
                {'var_id': 'x0', 'mean': {'x': 90.8, 'y': 42.2},
                 'covariance': {'sigma': [rSigMul(1/2), rSigMul(1/2)], 'rot':rRot()}},
                {'var_id': 'x1', 'mean': {'x': 90.3, 'y': 33.8},
                 'covariance': {'sigma': [rSigMul(), rSigMul()], 'rot':rRot()}},
                {'var_id': 'x2', 'mean': {'x': 85.5, 'y': 29.2},
                 'covariance': {'sigma': [rSigMul(), rSigMul()], 'rot':rRot()}},
                {'var_id': 'x3', 'mean': {'x': 76.5, 'y': 29.1},
                 'covariance': {'sigma': [rSigMul(), rSigMul()], 'rot':rRot()}},
                {'var_id': 'l11', 'mean': {
                    'x':
                        fd_landmark_by_id(world['landmarks'], 'l11')['state']['x']+rXY(), 'y':
                        fd_landmark_by_id(world['landmarks'], 'l11')[
                            'state']['y']+rXY()
                }, 'covariance': {'sigma': [1, 1], 'rot':0}},
                {'var_id': 'l14', 'mean': {
                    'x':
                        fd_landmark_by_id(world['landmarks'], 'l14')['state']['x']+rXY(), 'y':
                        fd_landmark_by_id(world['landmarks'], 'l14')[
                            'state']['y']+rXY()
                }, 'covariance': {'sigma': [1.2/2, 1.2/2], 'rot':0}},
                {'var_id': 'l15', 'mean': {
                    'x':
                        fd_landmark_by_id(world['landmarks'], 'l15')['state']['x']+rXY(), 'y':
                        fd_landmark_by_id(world['landmarks'], 'l15')[
                            'state']['y']+rXY()
                }, 'covariance': {'sigma': [1.2/2, 1.2/2], 'rot':0}},
            ],

            'factors': [
                {'factor_id': 'f0',
                 'type': 'ini_position',
                 'vars_id': ['x0'],
                 },
                {'factor_id': 'f1',
                 'type': 'odometry',
                 'vars_id': ['x1', 'x0'],
                 },
                {'factor_id': 'f2',
                 'type': 'odometry',
                 'vars_id': ['x2', 'x1'],
                 },
                {'factor_id': 'f3',
                 'type': 'odometry',
                 'vars_id': ['x3', 'x2'],
                 },
                {'factor_id': 'f4',
                 'type': 'range-bearing',
                 'vars_id': ['x0', 'l15'],
                 },
                {'factor_id': 'f5',
                 'type': 'range-bearing',
                 'vars_id': ['x1', 'l15'],
                 },
                {'factor_id': 'f6',
                 'type': 'range-bearing',
                 'vars_id': ['x2', 'l14'],
                 },
                # {'factor_id': 'f7',
                #  'type': 'range-bearing',
                #  'vars_id': ['x2', 'l11'],
                #  },
                {'factor_id': 'f8',
                 'type': 'range-bearing',
                 'vars_id': ['x3', 'l11'],
                 },
                {'factor_id': 'f9',
                 'type': 'range-bearing',
                 'vars_id': ['x2', 'l15'],
                 },
            ],
        }
    }
]

full_estimations2 = copy.deepcopy(full_estimations1)
full_estimations2[0]['graph']['marginals'].extend(
    [
        {'var_id': 'x5', 'mean': {'x': 39.78, 'y': 17.07},
         'covariance': {'sigma': [3.3, 3], 'rot':rRot()}},
        {'var_id': 'x6', 'mean': {'x': 36.47, 'y': 20.21},
         'covariance': {'sigma': [3.4, 2.8], 'rot':rRot()}},
        {'var_id': 'x7', 'mean': {'x': 34.98, 'y': 24.62},
         'covariance': {'sigma': [3.6, 3], 'rot':rRot()}},
        {'var_id': 'x8', 'mean': {'x': 42, 'y': 23.2},
         'covariance': {'sigma': [4, 3.2], 'rot':rRot()}},
        {'var_id': 'x9', 'mean': {'x': 48.74, 'y': 21.4},
         'covariance': {'sigma': [4, 3.2], 'rot':rRot()}},
        {'var_id': 'l5', 'mean': {
            'x': 35.2, 'y': 29.34},
         'covariance': {'sigma': [4, 3], 'rot':rRot()}},
        {'var_id': 'l6', 'mean': {
            'x': 57.6, 'y': 26.7},
         'covariance': {'sigma': [4, 3], 'rot':rRot()}},
        {'var_id': 'l10', 'mean': {
            'x': 59.2, 'y': 19.35},
         'covariance': {'sigma': [3, 3], 'rot':rRot()}},

    ]
)
full_estimations2[0]['graph']['factors'].extend(
    [
        {'factor_id': 'f10a', 'type': 'odometry',
         'vars_id': ['x5', 'x4'], },
        {'factor_id': 'f10', 'type': 'odometry',
         'vars_id': ['x5', 'x6'], },
        {'factor_id': 'f11', 'type': 'odometry',
         'vars_id': ['x7', 'x6'], },
        {'factor_id': 'f12', 'type': 'odometry',
         'vars_id': ['x7', 'x8'], },
        {'factor_id': 'f13', 'type': 'odometry',
         'vars_id': ['x8', 'x9'], },
        {'factor_id': 'f14', 'type': 'range-bearing',
         'vars_id': ['x5', 'l7'], },
        {'factor_id': 'f15', 'type': 'range-bearing',
         'vars_id': ['x7', 'l5'], },
        {'factor_id': 'f16', 'type': 'range-bearing',
         'vars_id': ['x9', 'l10'], },
        {'factor_id': 'f17', 'type': 'range-bearing',
         'vars_id': ['x9', 'l6'], },
    ]
)
full_estimations2[0]['header']['seq'] += 1
full_estimations2[0]['rt_estimate']['state']['x'] = 51.17
full_estimations2[0]['rt_estimate']['state']['y'] = 20.84
full_estimations2[0]['rt_estimate']['state']['th'] = -20*math.pi/180
full_estimations2[0]['last_pose']['last_pose_id'] = 'x9'

# green
full_estimations2[1]['graph']['marginals'].extend(
    [
        {'var_id': 'x3', 'mean': {'x': 31.7, 'y': 50.4},
         'covariance': {'sigma': [1, 1], 'rot':0}},
        {'var_id': 'x4', 'mean': {'x': 42.7, 'y': 53.6},
         'covariance': {'sigma': [1, 2], 'rot':math.pi/6}},
        {'var_id': 'x5', 'mean': {'x': 48.9, 'y': 56.54},
         'covariance': {'sigma': [1, 3], 'rot':math.pi/6}},
        {'var_id': 'x6', 'mean': {'x': 49.68, 'y': 48.5},
         'covariance': {'sigma': [2, 4], 'rot':0}},
        {'var_id': 'l8', 'mean': {
            'x': 39.07, 'y': 55.44},
         'covariance': {'sigma': [1, 3], 'rot':math.pi/3}},
        {'var_id': 'l6', 'mean': {
            'x': 50.8, 'y': 40.8},
         'covariance': {'sigma': [3, 3], 'rot':0}},

    ]
)
full_estimations2[1]['graph']['factors'].extend(
    [
        {'factor_id': 'f6', 'type': 'odometry', 'vars_id': ['x3', 'x4'], },
        {'factor_id': 'f7', 'type': 'odometry', 'vars_id': ['x4', 'x5'], },
        {'factor_id': 'f8', 'type': 'range-bearing',
         'vars_id': ['x5', 'x6'], },
        {'factor_id': 'f9', 'type': 'range-bearing',
         'vars_id': ['x5', 'l6'], },
        {'factor_id': 'f10', 'type': 'range-bearing',
         'vars_id': ['x3', 'l8'], },
        {'factor_id': 'f11', 'type': 'odometry',
         'vars_id': ['x3', 'x2'], },
    ]
)
full_estimations2[1]['header']['seq'] += 1
full_estimations2[1]['rt_estimate']['state']['x'] = 50.86
full_estimations2[1]['rt_estimate']['state']['y'] = 44.8
full_estimations2[1]['rt_estimate']['state']['th'] = -95*math.pi/180
full_estimations2[1]['rt_estimate']['covariance']['rot'] = 0
full_estimations2[1]['last_pose']['last_pose_id'] = 'x6'

# blue
full_estimations2[2]['graph']['marginals'].extend(
    [
        {'var_id': 'x4', 'mean': {'x': 67.92, 'y': 31},
         'covariance': {'sigma': [1, 1], 'rot':0}},
        {'var_id': 'x5', 'mean': {'x': 60.1, 'y': 30.75},
         'covariance': {'sigma': [1, 1], 'rot':0}},
        {'var_id': 'l10', 'mean': {
            'x': 52.42, 'y': 33.2},
         'covariance': {'sigma': [1, 1], 'rot':0}},
        {'var_id': 'l6', 'mean': {
            'x': 55.2, 'y': 41.68},
         'covariance': {'sigma': [1, 1], 'rot':0}},

    ]
)
full_estimations2[2]['graph']['factors'].extend(
    [
        {'factor_id': 'f10', 'type': 'odometry',
         'vars_id': ['x3', 'x4'], },
        {'factor_id': 'f11', 'type': 'odometry',
         'vars_id': ['x4', 'x5'], },
        {'factor_id': 'f14', 'type': 'range-bearing',
         'vars_id': ['x3', 'l11'], },
        {'factor_id': 'f15', 'type': 'range-bearing',
         'vars_id': ['x5', 'l6'], },
        {'factor_id': 'f16', 'type': 'range-bearing',
         'vars_id': ['x5', 'l10'], },
    ]
)
full_estimations2[2]['header']['seq'] += 1
full_estimations2[2]['rt_estimate']['state']['x'] = 58.17
full_estimations2[2]['rt_estimate']['state']['y'] = 32.84
full_estimations2[2]['rt_estimate']['state']['th'] = 135*math.pi/180
full_estimations2[2]['last_pose']['last_pose_id'] = 'x5'


full_estimations3 = copy.deepcopy(full_estimations2)

common_l6 = {'var_id': 'l6', 'mean': {'x': 52.6, 'y': 37},
             'covariance': {'sigma': [1, 1], 'rot': 0}}
common_l10 = {'var_id': 'l10', 'mean': {
    'x': 50, 'y': 31},
    'covariance': {'sigma': [1, 1], 'rot': 0}}

# red
# fix odom of red
i = getIdxM(full_estimations3[0]['graph']['marginals'], "x1")
full_estimations3[0]['graph']['marginals'][i]['mean']['x'] = 14.2
full_estimations3[0]['graph']['marginals'][i]['mean']['y'] = 12.12
i = getIdxM(full_estimations3[0]['graph']['marginals'], "x2")
full_estimations3[0]['graph']['marginals'][i]['mean']['x'] = 20.99
full_estimations3[0]['graph']['marginals'][i]['mean']['y'] = 12.35
i = getIdxM(full_estimations3[0]['graph']['marginals'], "x3")
full_estimations3[0]['graph']['marginals'][i]['mean']['x'] = 28.61
full_estimations3[0]['graph']['marginals'][i]['mean']['y'] = 13.53
i = getIdxM(full_estimations3[0]['graph']['marginals'], "x4")
full_estimations3[0]['graph']['marginals'][i]['mean']['x'] = 34.04
full_estimations3[0]['graph']['marginals'][i]['mean']['y'] = 15.81
i = getIdxM(full_estimations3[0]['graph']['marginals'], "x5")
full_estimations3[0]['graph']['marginals'][i]['mean']['x'] = 39.15
full_estimations3[0]['graph']['marginals'][i]['mean']['y'] = 19.9
i = getIdxM(full_estimations3[0]['graph']['marginals'], "x6")
full_estimations3[0]['graph']['marginals'][i]['mean']['x'] = 32.23
full_estimations3[0]['graph']['marginals'][i]['mean']['y'] = 22.68
i = getIdxM(full_estimations3[0]['graph']['marginals'], "x7")
full_estimations3[0]['graph']['marginals'][i]['mean']['x'] = 27.59
full_estimations3[0]['graph']['marginals'][i]['mean']['y'] = 27.37
i = getIdxM(full_estimations3[0]['graph']['marginals'], "x8")
full_estimations3[0]['graph']['marginals'][i]['mean']['x'] = 33.8
full_estimations3[0]['graph']['marginals'][i]['mean']['y'] = 28.94
i = getIdxM(full_estimations3[0]['graph']['marginals'], "x9")
full_estimations3[0]['graph']['marginals'][i]['mean']['x'] = 38.91
full_estimations3[0]['graph']['marginals'][i]['mean']['y'] = 29.10
# fix landmarks of red
i = getIdxM(full_estimations3[0]['graph']['marginals'], "l2")
full_estimations3[0]['graph']['marginals'][i]['mean']['x'] = fd_landmark_by_id(
    world['landmarks'], 'l2')['state']['x']+rXY(0.5)
full_estimations3[0]['graph']['marginals'][i]['mean']['y'] = fd_landmark_by_id(
    world['landmarks'], 'l2')['state']['y']+rXY(0.5)
i = getIdxM(full_estimations3[0]['graph']['marginals'], "l7")
full_estimations3[0]['graph']['marginals'][i]['mean']['x'] = fd_landmark_by_id(
    world['landmarks'], 'l7')['state']['x']+rXY(0.5)
full_estimations3[0]['graph']['marginals'][i]['mean']['y'] = fd_landmark_by_id(
    world['landmarks'], 'l7')['state']['y']+rXY(0.5)
i = getIdxM(full_estimations3[0]['graph']['marginals'], "l5")
full_estimations3[0]['graph']['marginals'][i]['mean']['x'] = fd_landmark_by_id(
    world['landmarks'], 'l5')['state']['x']+rXY(0.5)
full_estimations3[0]['graph']['marginals'][i]['mean']['y'] = fd_landmark_by_id(
    world['landmarks'], 'l5')['state']['y']+rXY(0.5)
# diminish all cov of red
full_estimations3[0]['graph']['marginals'] = [modify_covs(
    m) for m in full_estimations3[0]['graph']['marginals']]
# fix common landmarks of red
i = getIdxM(full_estimations3[0]['graph']['marginals'], "l6")
full_estimations3[0]['graph']['marginals'][i] = common_l6
i = getIdxM(full_estimations3[0]['graph']['marginals'], "l10")
full_estimations3[0]['graph']['marginals'][i] = common_l10
# fix last pose of red
full_estimations3[0]['rt_estimate']['state']['x'] = 42.45
full_estimations3[0]['rt_estimate']['state']['y'] = 30.675
full_estimations3[0]['rt_estimate']['state']['th'] = 25*math.pi/180.0

# green
# fix green odom
i = getIdxM(full_estimations3[1]['graph']['marginals'], "x1")
full_estimations3[1]['graph']['marginals'][i]['mean']['x'] = 15
full_estimations3[1]['graph']['marginals'][i]['mean']['y'] = 49.12
i = getIdxM(full_estimations3[1]['graph']['marginals'], "x2")
full_estimations3[1]['graph']['marginals'][i]['mean']['x'] = 21.99
full_estimations3[1]['graph']['marginals'][i]['mean']['y'] = 49.35
i = getIdxM(full_estimations3[1]['graph']['marginals'], "x3")
full_estimations3[1]['graph']['marginals'][i]['mean']['x'] = 31.61
full_estimations3[1]['graph']['marginals'][i]['mean']['y'] = 47.53
i = getIdxM(full_estimations3[1]['graph']['marginals'], "x4")
full_estimations3[1]['graph']['marginals'][i]['mean']['x'] = 48.04
full_estimations3[1]['graph']['marginals'][i]['mean']['y'] = 49.81
i = getIdxM(full_estimations3[1]['graph']['marginals'], "x5")
full_estimations3[1]['graph']['marginals'][i]['mean']['x'] = 53.15
full_estimations3[1]['graph']['marginals'][i]['mean']['y'] = 50.9
i = getIdxM(full_estimations3[1]['graph']['marginals'], "x6")
full_estimations3[1]['graph']['marginals'][i]['mean']['x'] = 53.23
full_estimations3[1]['graph']['marginals'][i]['mean']['y'] = 44.68
# fix landmarks of green
i = getIdxM(full_estimations3[1]['graph']['marginals'], "l8")
full_estimations3[1]['graph']['marginals'][i]['mean']['x'] = fd_landmark_by_id(
    world['landmarks'], 'l8')['state']['x']+rXY(0.5)
full_estimations3[1]['graph']['marginals'][i]['mean']['y'] = fd_landmark_by_id(
    world['landmarks'], 'l8')['state']['y']+rXY(0.5)
# diminish all cov of green
full_estimations3[1]['graph']['marginals'] = [modify_covs(
    m) for m in full_estimations3[1]['graph']['marginals']]
# fix common landmarks of green
i = getIdxM(full_estimations3[1]['graph']['marginals'], "l6")
full_estimations3[1]['graph']['marginals'][i] = common_l6
# fix last pose of green
full_estimations3[1]['rt_estimate']['state']['x'] = 52
full_estimations3[1]['rt_estimate']['state']['y'] = 41
full_estimations3[1]['rt_estimate']['state']['th'] = -95*math.pi/180

# blue
# fix blue odom
i = getIdxM(full_estimations3[2]['graph']['marginals'], "x4")
full_estimations3[2]['graph']['marginals'][i]['mean']['x'] = 68
full_estimations3[2]['graph']['marginals'][i]['mean']['y'] = 29.5
i = getIdxM(full_estimations3[2]['graph']['marginals'], "x5")
full_estimations3[2]['graph']['marginals'][i]['mean']['x'] = 60.3
full_estimations3[2]['graph']['marginals'][i]['mean']['y'] = 28.7
# fix landmarks of blue
i = getIdxM(full_estimations3[2]['graph']['marginals'], "l14")
full_estimations3[2]['graph']['marginals'][i]['mean']['x'] = fd_landmark_by_id(
    world['landmarks'], 'l14')['state']['x']+rXY(0.5)
full_estimations3[2]['graph']['marginals'][i]['mean']['y'] = fd_landmark_by_id(
    world['landmarks'], 'l14')['state']['y']+rXY(0.5)
# diminish all cov of blue
full_estimations3[2]['graph']['marginals'] = [modify_covs(
    m) for m in full_estimations3[2]['graph']['marginals']]
# fix common landmarks of blue
i = getIdxM(full_estimations3[2]['graph']['marginals'], "l6")
full_estimations3[2]['graph']['marginals'][i] = common_l6
i = getIdxM(full_estimations3[2]['graph']['marginals'], "l10")
full_estimations3[2]['graph']['marginals'][i] = common_l10
# fix last pose of blue
full_estimations3[2]['rt_estimate']['state']['x'] = 56.7
full_estimations3[2]['rt_estimate']['state']['y'] = 30.6

# ----------------------------------------------------------------------------
#         mqtt layer callbacks overrides (not yet users callbacks)
# ----------------------------------------------------------------------------


def on_log(client, userdata, level, buf):
    print('log: ' + buf)


def on_connect(client, userdata, flags, rc):
    if rc == 0:
        print('connected')
    else:
        print('connection error. code :', rc)
    # do the subscriptions here
    # client.subscribe(cmd_topic)
    # client.subscribe(request_position_ini_topic)
    client.subscribe(request_ground_truth_topic)
    client.subscribe(request_estimation_graph_topic)


def on_disconnect(client, userdata, flags, rc=0):
    print('Disconnected result code : ' + str(rc))


def on_message(client, userdata, message):
    print("Received message '" + str(message.payload) + "' on topic '"
          + message.topic + "' with QoS " + str(message.qos) + "'\n")

    # decode payload as string
    msg = message.payload.decode('utf-8')
    # depending on the topic, disptach to the user defined functions
    if message.topic == request_estimation_graph_topic:
        if msg == '1':
            client.publish(estimation_graph_topic,
                           json.dumps(full_estimations1))
        elif msg == '2':
            client.publish(estimation_graph_topic,
                           json.dumps(full_estimations2))
        elif msg == '3':
            client.publish(estimation_graph_topic,
                           json.dumps(full_estimations3))
    elif message.topic == request_ground_truth_topic:
        if msg == '2':
            client.publish(ground_truth_topic, json.dumps(world2))
        else:
            client.publish(ground_truth_topic, json.dumps(world))

    # if firstTime:
    #      # publish
    # print('publishing initial pose for robots')
# # TODO: make it accessible on demand
    #  request_position_ini_topic
    #  position_ini_topic


# print('\n')


# -----------------------------------------------------------------------------
#                           Main
# -----------------------------------------------------------------------------
if __name__ == '__main__':
    # -------------------------------------------------------------------------
    #                       Print Simulation Data
    # -------------------------------------------------------------------------
    print(f"Simulation World :\n${world}")

    # -------------------------------------------------------------------------
    #                       MQTT connection
    # -------------------------------------------------------------------------

    mqttc = mqtt.Client('simulation-py')
    print("Connecting to broker : ", broker)

    # the on_connect method will set the subscriber
    mqttc.on_connect = on_connect
    # mqttc.on_log=on_log
    mqttc.on_message = on_message

    # connect to the broker
    mqttc.connect(broker)

    # block the code here, process the messages for the subscribed topics
    print('looping')
    mqttc.loop_forever()
    print('disconnecting')
    mqttc.disconnect()
