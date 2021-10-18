#!/usr/bin/python3

import paho.mqtt.client as mqtt
import time
import json
import numpy as np
import copy
import math
import random as rd
import numpy.random as nprd


request_estimation_graph_topic = 'request_estimation'
estimation_graph_topic = 'estimation'

# the stub is a 5 vars sparsely connected graph
# if the problem is EKF, there would only be

# some randomness
def rSig():
    return rd.uniform(1,5)
def rRot():
    return rd.uniform(0,math.pi/2)
# randomness at iteration
def rDSig():
    return rd.uniform(0.3,0.8)
def rDRot():
    return rd.uniform(-math.pi/3,math.pi/3)
def rDxy():
    return rd.normalvariate(0,2)

def increment_things_in_marginal(m) -> dict:
    m['mean']['x']+= rDxy()
    m['mean']['y']+= rDxy()
    m['covariance']['sigma'][0]*=rDSig()
    m['covariance']['sigma'][1]*=rDSig()
    m['covariance']['rot']+=rDRot()
    return m


# TODO: had RMSE, hypothesis_id, robot_it (perhaps separately in a header)
graph = {
    'marginals': [
        {'var_id': 'l1', 'mean': {'x': 20, 'y': 37}, 'covariance': {'sigma': [rSig(), rSig()], 'rot':rRot()}},
        {'var_id': 'l2', 'mean': {'x': 25, 'y': 47}, 'covariance': {'sigma': [rSig(), rSig()], 'rot':rRot()}},
        {'var_id': 'l3', 'mean': {'x': 30, 'y': 32}, 'covariance': {'sigma': [rSig(), rSig()], 'rot':rRot()}},
        {'var_id': 'l4', 'mean': {'x': 55, 'y': 49}, 'covariance': {'sigma': [rSig(), rSig()], 'rot':rRot()}},
        {'var_id': 'l5', 'mean': {'x': 75, 'y': 25}, 'covariance': {'sigma': [rSig(), rSig()], 'rot':rRot()}},
        {'var_id': 'x0', 'mean': {'x': 75, 'y': 9}, 'covariance': {'sigma': [rSig(), rSig()], 'rot':rRot()}},
    ],
    'mean': [],
    'covariance': [],
    'information': [],
    'sqrtroot': [],
    'factors': [
        {'factor_id': 'fp0',
            'type': 'ini_position',
            'vars_id':['x0'],
            },
        {'factor_id': 'f0',
            'type': 'ini_position',
            'vars_id':['l3'],
            },
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
full_estimation={
        'header': {
            'robot_id': 'r4',
            "state": {"x": 87.3, "y": 9, "th": 0},
            'seq': 0,
        },
        'last_pose':{'last_pose_id':'x0'},
        'graph': graph
        }


# Change l4.y
# some random increments to everything else
graph1 = copy.deepcopy(graph)

graph1['marginals'] = [increment_things_in_marginal(copy.deepcopy(m)) for m in graph['marginals']]
graph1['marginals'][3]['mean']['y']=14
graph1['factors'].pop(4)
graph1['factors'].extend(
        [
            {'factor_id': 'f00',
                'type': 'odometry',
                'vars_id':['x0'],
                },
            {'factor_id': 'f00bis',
                'type': 'odometry',
                'vars_id':['x0'],
                },
            ]
        )
full_estimation1= \
{
    'header': 
    {
        'robot_id': 'r4',
        "state": {"x": 82.3, "y": 12, "th": 10},
        'seq': 0,
    },
    'last_pose':{'last_pose_id':'x0'},
    'graph': graph1
}



# add new var l6 and some random change to everything else
graph2 = copy.deepcopy(graph1)
graph2['marginals'] = [increment_things_in_marginal(copy.deepcopy(m)) for m in graph1['marginals']]
graph2['marginals'].append(
        {'var_id': 'l6', 'mean': {'x': 26, 'y': 9}, 'covariance': {'sigma': [rSig(), rSig()], 'rot':rRot()}},
        )
graph2['marginals'][3]['mean']['y']=31
graph2['factors'].extend(
        [
            {'factor_id': 'f6',
                'type': 'odometry',
                'vars_id':['l6','l1'],
                },
            {'factor_id': 'f7',
                'type': 'odometry',
                'vars_id':['l6','l3'],
                },
            {'factor_id': 'f8',
                'type': 'range-bearing',
                'vars_id':['l5','x0'],
                },
            ]
        )
graph2['variable_ordering'].append('l6') # extend and append are different
full_estimation2={
        'header': {
            'robot_id': 'r4',
            "state": {"x": 85.3, "y": 10, "th": -10},
            'seq': 0,
        },
        'last_pose':{'last_pose_id':'x0'},
        'graph': graph2
        }


# trifactor for the fun
graph3 = copy.deepcopy(graph2)
graph3['marginals'] = [increment_things_in_marginal(copy.deepcopy(m)) for m in graph2['marginals']]
graph3['factors'].extend(
        [
            {'factor_id': 'f9',
                'type': 'smart',
                'vars_id':['l4','l5','x0'],
                },
            ]
        )
full_estimation3={
        'header': {
            'robot_id': 'r4',
            "state": {"x": 88.3, "y": 4, "th": -30},
            'seq': 0,
        },
        'last_pose':{'last_pose_id':'x0'},
        'graph': graph3
        }

# One last 
graph4 = copy.deepcopy(graph3)
graph4['marginals'] = [increment_things_in_marginal(copy.deepcopy(m)) for m in graph3['marginals']]
graph4['factors'].pop()
full_estimation4={
        'header': {
            'robot_id': 'r4',
            "state": {"x": 84.3, "y": 17, "th": 70},
            'seq': 0,
        },
        'last_pose':{'last_pose_id':'x0'},
        'graph': graph4
        }

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
        elif msg == '3':
            client.publish(estimation_graph_topic,
                           json.dumps(full_estimation3))
        elif msg == '4':
            client.publish(estimation_graph_topic,
                           json.dumps(full_estimation4))
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
