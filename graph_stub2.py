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
            "state": {"x": 38.6, "y": 19, "th": 55},
            "sensor": {"range": 12, "angle_coverage": 0.75}
        },
        {
            "robot_id": "r2",
            "state": {"x": 27.3, "y": 48.3, "th": 0},
            "sensor": {"range": 12, "angle_coverage": 0.75}
        },
        {
            "robot_id": "r3",
            "state": {"x": 70.7, "y": 29.9, "th": -175},
            "sensor": {"range": 12, "angle_coverage": 0.5}
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


# some mqtt related globals
broker = 'localhost'
request_ground_truth_topic = 'request_ground_truth'
ground_truth_topic = 'ground_truth'
request_estimation_graph_topic = 'request_estimation'
estimation_graph_topic = 'estimation'

# ----------------------------------------------------------------------------
#                           Some utility functions
# ----------------------------------------------------------------------------


def rSig():
    return rd.uniform(1, 5)


def rRot():
    return rd.uniform(0, math.pi/2)
# randomness at iteration


def rDSig():
    return rd.uniform(0.3, 0.8)


def rDRot():
    return rd.uniform(-math.pi/3, math.pi/3)


def rXY(sig=1.5):
    return rd.normalvariate(0, sig)


def fd_landmark_by_id(ArrayOfLandmarks, lid):
    return next(item for item in ArrayOfLandmarks if item["landmark_id"] == lid)
    # return next(filter(lambda x: x['landmark_id'] == lid, ArrayOfLandmarks))

# ----------------------------------------------------------------------------
#                           Globals: Fake MultiMap
# ----------------------------------------------------------------------------

full_estimations1 = [
    {
        'header': {
            'robot_id': 'r1',
            "state": {"x": 40.3, "y": 17.4, "th": 47},
            'seq': 0,
        },
        'last_pose':{'last_pose_id':'x4'},
        # TODO : plural graph with graph id for each graph and a header designating
        #        the main hypothesis
        # 'main_hypothesis_graph_id':'g1'
        'graph': { # TODO: plural-> array, one header per element and the following encapsulated in 'data', 
                   #                                                
            'marginals': [
                {'var_id': 'x0', 'mean': {'x': 6.5, 'y': 12},
                 'covariance': {'sigma': [1, 1], 'rot':0}},
                {'var_id': 'x1', 'mean': {'x': 14, 'y': 10.6},
                 'covariance': {'sigma': [1, 1], 'rot':0}},
                {'var_id': 'x2', 'mean': {'x': 21.1, 'y': 10.5},
                 'covariance': {'sigma': [1, 1], 'rot':0}},
                {'var_id': 'x3', 'mean': {'x': 27.7, 'y': 10.9},
                 'covariance': {'sigma': [1, 1], 'rot':0}},
                {'var_id': 'x4', 'mean': {'x': 34.88, 'y': 13.8},
                 'covariance': {'sigma': [1, 1], 'rot':0}},
                {'var_id': 'l2', 'mean': {
                    'x':
                        fd_landmark_by_id(world['landmarks'], 'l2')['state']['x']+rXY(), 'y':
                        fd_landmark_by_id(world['landmarks'], 'l2')[
                            'state']['y']+rXY()
                }, 'covariance': {'sigma': [1, 1], 'rot':0}},
                {'var_id': 'l7', 'mean': {
                    'x':
                        fd_landmark_by_id(world['landmarks'], 'l7')['state']['x']+rXY(), 'y':
                        fd_landmark_by_id(world['landmarks'], 'l7')[
                            'state']['y']+rXY()
                }, 'covariance': {'sigma': [1, 1], 'rot':0}},
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
    }
    , 
    # The second robot
    {
        'header': {
            'robot_id': 'r2',
            "state": {"x": 26.5, "y": 50.46, "th": -4},
            'seq': 0,
        },
        'last_pose':{'last_pose_id':'x2'},
        'graph': {
            'marginals': [
                {'var_id': 'x0', 'mean': {'x': 7.3, 'y': 51.6},
                 'covariance': {'sigma': [1, 1], 'rot':0}},
                {'var_id': 'x1', 'mean': {'x': 15, 'y': 50.5},
                 'covariance': {'sigma': [1, 1], 'rot':0}},
                {'var_id': 'x2', 'mean': {'x': 21.12, 'y': 50.1},
                 'covariance': {'sigma': [1, 1], 'rot':0}},
                {'var_id': 'l1', 'mean': {
                    'x':
                        fd_landmark_by_id(world['landmarks'], 'l1')['state']['x']+rXY(), 'y':
                        fd_landmark_by_id(world['landmarks'], 'l1')[
                            'state']['y']+rXY()
                }, 'covariance': {'sigma': [1, 1], 'rot':0}},
                {'var_id': 'l3', 'mean': {
                    'x':
                        fd_landmark_by_id(world['landmarks'], 'l3')['state']['x']+rXY(), 'y':
                        fd_landmark_by_id(world['landmarks'], 'l3')[
                            'state']['y']+rXY()
                }, 'covariance': {'sigma': [1, 1], 'rot':0}},
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
    }
    ,
    # the third robot
    {
        'header': {
            'robot_id': 'r3',
            "state": {"x": 71.3, "y": 30.5, "th": 175},
            'seq': 0,
        },
        'last_pose':{'last_pose_id':'x3'},
        'graph': {
            'marginals': [
                {'var_id': 'x0', 'mean': {'x': 90.8, 'y': 42.2},
                 'covariance': {'sigma': [1, 1], 'rot':0}},
                {'var_id': 'x1', 'mean': {'x': 90.3, 'y': 33.8},
                 'covariance': {'sigma': [1, 1], 'rot':0}},
                {'var_id': 'x2', 'mean': {'x': 85.5, 'y': 29.2},
                 'covariance': {'sigma': [1, 1], 'rot':0}},
                {'var_id': 'x3', 'mean': {'x': 76.5, 'y': 29.1},
                 'covariance': {'sigma': [1, 1], 'rot':0}},
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
                }, 'covariance': {'sigma': [1, 1], 'rot':0}},
                {'var_id': 'l15', 'mean': {
                    'x':
                        fd_landmark_by_id(world['landmarks'], 'l15')['state']['x']+rXY(), 'y':
                        fd_landmark_by_id(world['landmarks'], 'l15')[
                            'state']['y']+rXY()
                }, 'covariance': {'sigma': [1, 1], 'rot':0}},
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


full_estimations2 = {}

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
    elif message.topic == request_ground_truth_topic:
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
