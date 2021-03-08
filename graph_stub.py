#!/usr/bin/python3

import paho.mqtt.client as mqtt
import time
import json
import numpy as np
import copy
import math
import random as rd
import numpy.random as nprd


request_estimation_graph_topic = 'request_estimation_graph'
estimation_graph_topic = 'estimation_graph'

# the stub is a 5 vars sparsely connected graph
# if the problem is EKF, there would only be

# some randomness
def rSig():
    return rd.uniform(2,10)
def rRot():
    return rd.uniform(0,math.pi/2)
# randomness at iteration
def rDSig():
    return rd.uniform(0,3)
def rDRot():
    return rd.uniform(-math.pi/12,math.pi/12)
def rDxy():
    return rd.normalvariate(0,4)

# TODO: had RMSE, hypothesis_id, robot_it (perhaps separately in a header)
full_estimation = {
    'marginals': [
        {'var_id': 'l1', 'mean': {'x': 20, 'y': 37}, 'covariance': {'sigma': [rSig(), rSig()], 'rot':rRot()}},
        {'var_id': 'l2', 'mean': {'x': 25, 'y': 47}, 'covariance': {'sigma': [rSig(), rSig()], 'rot':rRot()}},
        {'var_id': 'l3', 'mean': {'x': 30, 'y': 32}, 'covariance': {'sigma': [rSig(), rSig()], 'rot':rRot()}},
        {'var_id': 'l4', 'mean': {'x': 55, 'y': 49}, 'covariance': {'sigma': [rSig(), rSig()], 'rot':rRot()}},
        {'var_id': 'l5', 'mean': {'x': 75, 'y': 25}, 'covariance': {'sigma': [rSig(), rSig()], 'rot':rRot()}},
    ],
    'mean': [],
    'covariance': [],
    'information': [],
    'sqrtroot': [],
    'factors': [
        {'factor_id': 'f1',
            'type': 'odometry',
            'vars_id':['l1','l2'],
            },
        {'factor_id': 'f2',
            'type': 'odometry',
            'vars_id':['l3','l1'],
         },
        {'factor_id': 'f3',
            'type': 'odometry',
            'vars_id':['l3','l2'],
            },
        {'factor_id': 'f5',
            'type': 'odometry',
            'vars_id':['l4','l5'],
            },
        {'factor_id': 'f4',
            'type': 'odometry',
            'vars_id':['l4','l3'],
            },
    ],
    'variable_ordering': ['l2', 'l1', 'l4', 'l3', 'l5']
}


# TODO: just do deepcopies and modify or else its a pain
# Change l4.y
# some random increments to everything else
full_estimation1 = copy.deepcopy(full_estimation)
full_estimation1 = {
    'marginals': [
        {'var_id': 'l1', 'mean': {'x': 20, 'y': 37}, 'covariance': {'sigma': [1, 0.3], 'rot':0.5}},
        {'var_id': 'l2', 'mean': {'x': 25, 'y': 47}, 'covariance': {'sigma': [1, 0.3], 'rot':0.5}},
        {'var_id': 'l3', 'mean': {'x': 30, 'y': 32}, 'covariance': {'sigma': [1, 0.3], 'rot':0.5}},
        {'var_id': 'l4', 'mean': {'x': 55, 'y': 14}, 'covariance': {'sigma': [1, 0.3], 'rot':0.5}},
        {'var_id': 'l5', 'mean': {'x': 75, 'y': 25}, 'covariance': {'sigma': [1, 0.3], 'rot':0.5}},
    ],
    'mean': [],
    'covariance': [],
    'information': [],
    'sqrtroot': [],
    'factors': [
        {'factor_id': 'f1',
            'type': 'odometry',
            'vars_id':['l1','l2'],
            },
        {'factor_id': 'f2',
            'type': 'odometry',
            'vars_id':['l3','l1'],
         },
        {'factor_id': 'f3',
            'type': 'odometry',
            'vars_id':['l3','l2'],
            },
        {'factor_id': 'f5',
            'type': 'odometry',
            'vars_id':['l4','l5'],
            },
        {'factor_id': 'f4',
            'type': 'odometry',
            'vars_id':['l4','l3'],
            },
    ],
    'variable_ordering': ['l2', 'l1', 'l4', 'l3', 'l5']
}

# add new var l6 and some random change to everything else
full_estimation2 = {
    'marginals': [
        {'var_id': 'l1', 'mean': {'x': 20, 'y': 37}, 'covariance': {'sigma': [1, 0.3], 'rot':0.5}},
        {'var_id': 'l2', 'mean': {'x': 25, 'y': 47}, 'covariance': {'sigma': [1, 0.3], 'rot':0.5}},
        {'var_id': 'l3', 'mean': {'x': 30, 'y': 32}, 'covariance': {'sigma': [1, 0.3], 'rot':0.5}},
        {'var_id': 'l4', 'mean': {'x': 55, 'y': 49}, 'covariance': {'sigma': [1, 0.3], 'rot':0.5}},
        {'var_id': 'l5', 'mean': {'x': 75, 'y': 25}, 'covariance': {'sigma': [1, 0.3], 'rot':0.5}},
        {'var_id': 'l6', 'mean': {'x': 26, 'y': 9}, 'covariance': {'sigma': [1, 0.3], 'rot':0.5}},
    ],
    'mean': [],
    'covariance': [],
    'information': [],
    'sqrtroot': [],
    'factors': [
        {'factor_id': 'f1',
            'type': 'odometry',
            'vars_id':['l1','l2'],
            },
        {'factor_id': 'f2',
            'type': 'odometry',
            'vars_id':['l3','l1'],
         },
        {'factor_id': 'f3',
            'type': 'odometry',
            'vars_id':['l3','l2'],
            },
        {'factor_id': 'f5',
            'type': 'odometry',
            'vars_id':['l4','l5'],
            },
        {'factor_id': 'f4',
            'type': 'odometry',
            'vars_id':['l4','l3'],
            },
        {'factor_id': 'f6',
         'type': 'odometry',
            'vars_id':['l6','l1'],
         },
        {'factor_id': 'f7',
         'type': 'odometry',
            'vars_id':['l6','l3'],
         },
    ],
    'variable_ordering': ['l2', 'l1', 'l4', 'l3', 'l5']
}
# the factor contains: its id, the type of factor,
# most fields are optional, and depend on the problem (EKF,SAM,etc...)
# map (maximum a posteriori) and mean have the same data but structured
# differently (mean is a raw vector)

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
    client.subscribe(request_estimation_graph_topic)


def on_disconnect(client, userdata, flags, rc=0):
    print('Disconnected result code : ' + str(rc))


def on_message(client, userdata, message):
    print("Received message '" + str(message.payload) + "' on topic '"
          + message.topic + "' with QoS " + str(message.qos) + "'\n")

    # depending on the topic, disptach to the user defined functions
    if message.topic == request_estimation_graph_topic:
        # the graph change arbitrarily depending on the request
        msg = message.payload.decode('utf-8')
        print('Received (decoded as utf8 string) :  ' + msg)
        if msg == '1':
            client.publish(estimation_graph_topic,
                           json.dumps(full_estimation1))
        elif msg == '2':
            client.publish(estimation_graph_topic,
                           json.dumps(full_estimation2))
        else:
            client.publish(estimation_graph_topic, json.dumps(full_estimation))

    else:
        raise RuntimeError


# -----------------------------------------------------------------------------
#                           Main
# -----------------------------------------------------------------------------
if __name__ == '__main__':
    print(f"Graph Stub program")

    # -------------------------------------------------------------------------
    #                       MQTT connection
    # -------------------------------------------------------------------------
    broker = 'localhost'
    mqttc = mqtt.Client('graph_stub')
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
