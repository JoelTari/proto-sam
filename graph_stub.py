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

full_estimation = {
    'marginals': [
        {'var_id': 'l1', 'mean': {'x': 20, 'y': 37}, 'covariance':[0,1,0,.3]},
        {'var_id': 'l2', 'mean': {'x': 25, 'y': 47}, 'covariance':[0,1,0,.3]},
        {'var_id': 'l3', 'mean': {'x': 30, 'y': 32}, 'covariance':[0,1,0,.3]},
        {'var_id': 'l4', 'mean': {'x': 55, 'y': 49}, 'covariance':[0,1,0,.3]},
        {'var_id': 'l5', 'mean': {'x': 75, 'y': 25}, 'covariance':[0,1,0,.3]},
    ],
    'mean': [],
    'covariance': [],
    'information': [],
    'sqrtroot': [],
    'factors': [
        {'factor_id': 'f1',
            'type': 'odometry',
            'vars_id': ['l1', 'l2']},
        {'factor_id': 'f2',
            'type': 'odometry',
            'vars_id': ['l3', 'l1']},
        {'factor_id': 'f3',
            'type': 'odometry',
            'vars_id': ['l3', 'l2']},
        {'factor_id': 'f5',
            'type': 'odometry',
            'vars_id': ['l4', 'l5']},
        {'factor_id': 'f4',
            'type': 'odometry',
            'vars_id': ['l3', 'l4']},
    ],
    'variable_ordering': ['l2', 'l1', 'l4', 'l3', 'l5']
}

full_estimation1 = {
    'map': [
        {'var_id': 'l1', 'state': {'x': 20, 'y': 37}},
        {'var_id': 'l2', 'state': {'x': 25, 'y': 47}},
        {'var_id': 'l3', 'state': {'x': 30, 'y': 32}},
        {'var_id': 'l4', 'state': {'x': 55, 'y': 14}},
        {'var_id': 'l5', 'state': {'x': 75, 'y': 25}},
    ],
    'mean': [],
    'covariance': [],
    'information': [],
    'sqrtroot': [],
    'factors': [
        {'factor_id': 'f1',
            'type': 'odometry',
            'vars_id': ['l1', 'l2']},
        {'factor_id': 'f2',
            'type': 'odometry',
            'vars_id': ['l3', 'l1']},
        {'factor_id': 'f3',
            'type': 'odometry',
            'vars_id': ['l3', 'l2']},
        {'factor_id': 'f5',
            'type': 'odometry',
            'vars_id': ['l4', 'l5']},
        {'factor_id': 'f4',
            'type': 'odometry',
            'vars_id': ['l3', 'l4']},
    ],
    'variable_ordering': ['l2', 'l1', 'l4', 'l3', 'l5']
}

full_estimation2 = {
    'map': [
        {'var_id': 'l1', 'state': {'x': 20, 'y': 37}},
        {'var_id': 'l2', 'state': {'x': 25, 'y': 47}},
        {'var_id': 'l3', 'state': {'x': 30, 'y': 32}},
        {'var_id': 'l4', 'state': {'x': 55, 'y': 49}},
        {'var_id': 'l5', 'state': {'x': 75, 'y': 25}},
        {'var_id': 'l6', 'state': {'x': 26, 'y': 9}}
    ],
    'mean': [],
    'covariance': [],
    'information': [],
    'sqrtroot': [],
    'factors': [
        {'factor_id': 'f1',
            'type': 'odometry',
            'vars_id': ['l1', 'l2']},
        {'factor_id': 'f2',
            'type': 'odometry',
            'vars_id': ['l3', 'l1']},
        {'factor_id': 'f3',
            'type': 'odometry',
            'vars_id': ['l3', 'l2']},
        {'factor_id': 'f5',
            'type': 'odometry',
            'vars_id': ['l4', 'l5']},
        {'factor_id': 'f4',
            'type': 'odometry',
            'vars_id': ['l3', 'l4']},
        {'factor_id': 'f6',
         'type': 'odometry',
         'vars_id': ['l6', 'l1']},
        {'factor_id': 'f7',
         'type': 'odometry',
         'vars_id': ['l6', 'l3']},
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
