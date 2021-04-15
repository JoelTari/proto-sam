#!/usr/bin/python3

import paho.mqtt.client as mqtt
import time
import json
import numpy as np
import copy
import math
import random as rd
import numpy.random as nprd
import sys
import time

last_pose_idx = 0
last_pose_state = {'x': 0, 'y': 0, 'th': 0}
factor_idx = 0
last_cov = np.zeros([3, 3])

if __name__ == '__main__':
    # Robot name
    robot_id = sys.argv[1]
    broker = 'localhost'

    #inputs (subs)
    position_ini_topic = '/'.join([robot_id, 'position_ini'])
    measures_pose_topic_in = '/'.join([robot_id, 'measures_pose'])
    request_graph_topic_in = '/'.join([robot_id, 'request_graphs'])
    #outputs (pubs)
    updated_reference_pose_topic_out = '/'.join(
        [robot_id, 'update_reference_pose'])
    request_position_ini_topic = '/'.join([robot_id, 'request_position_ini'])
    odom_topic_out = '/'.join([robot_id, 'odom'])
    graphs_topic_out = '/'.join([robot_id, 'graphs'])

    graphs = {
        'header': {
            'robot_id': robot_id,
            'seq': 0,
        }
        , 'marginals': [], 'factors': []}

    def ecpi(a):
        return math.atan2(math.sin(a), math.cos(a))

    def stringify_pose_id(count):
        return f'X{count}'

    def stringify_factor_id(count):
        return f'f{count}'

    def getVisualFromCovMatrix(cov: np.ndarray) -> dict:
        eVa, eVec = np.linalg.eig(cov)
        R = eVec
        angle = math.atan2(R[1, 0], R[0, 0])
        sigmax = math.sqrt(eVa[0])
        sigmay = math.sqrt(eVa[1])
        return {'sigma': [sigmax, sigmay], 'rot': angle}

    def getRotationMatrix(angle, size = 2):
        c= math.cos(angle)
        s= math.sin(angle)
        if (size==2):
            return np.array([[c,-s],[s,c]])
        elif (size ==3):
            return np.array([[c,-s,0],[s,c,0],[0,0,1]])


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
        client.subscribe(position_ini_topic)
        client.subscribe(measures_pose_topic_in)
        client.subscribe(request_graph_topic_in)
        time.sleep(3) # TODO: find another way
        client.publish(request_position_ini_topic)

    def on_disconnect(client, userdata, flags, rc=0):
        print('Disconnected result code : ' + str(rc))

    def on_message(client, userdata, message):
        # print("Received message '" + str(message.payload) + "' on topic '"
        #       + message.topic + "' with QoS " + str(message.qos) + "'\n")
        global last_pose_state
        global last_pose_idx     # ugly
        global factor_idx
        global last_cov

        # decode payload as string, and transform as JSON
        msg = json.loads(message.payload.decode('utf-8'))
        # depending on the topic, disptach to the user defined functions
        if (message.topic == measures_pose_topic_in):
            # update the last_pose_state and increment index, and publish
            relative_pose = msg['odom']['state']
            # the relative pose should be anchored to the last pose
            xref = last_pose_state['x']
            yref = last_pose_state['y']
            thref = last_pose_state['th']
            x = relative_pose['x']
            y = relative_pose['y']
            th = relative_pose['th']
            new_reference_state = {'robot_id': msg['robot_id'], 'state': {}}
            new_reference_state['state']['x'] = x * \
                math.cos(thref) - y*math.sin(thref) + xref
            new_reference_state['state']['y'] = x * \
                math.sin(thref) + y*math.cos(thref) + yref
            new_reference_state['state']['th'] = ecpi(th+thref)
            last_pose_state = new_reference_state['state']
            client.publish(updated_reference_pose_topic_out,
                           json.dumps(new_reference_state))
            node_previous_id = stringify_pose_id(last_pose_idx-1)
            node_id = stringify_pose_id(last_pose_idx)
            factor_id = stringify_factor_id(factor_idx)
            rot_mat_relative = getRotationMatrix(thref,size=3)
            cov =  rot_mat_relative@np.array(msg['odom']['covariance']).reshape(3, 3)@rot_mat_relative.T + last_cov
            # cov =  np.array(msg['odom']['covariance']).reshape(3, 3) + last_cov
            visual_cov = getVisualFromCovMatrix(cov[0:2,0:2])
            # visual_cov['rot'] = ecpi(visual_cov['rot'] + thref)
            graphs['marginals'].append(
                {'var_id': node_id, 'mean': last_pose_state,
                    'covariance': visual_cov}  # TODO: raw cov, do the visual conversion in dataviz manager
            )
            graphs['factors'].append(
                {'factor_id': factor_id,
                 'type': 'odometry',
                 'vars_id': [node_id, node_previous_id],
                 }
            )
            last_pose_idx += 1
            last_cov = cov
            factor_idx += 1
            # CONTINUE HERE (landmarks)
            for measure in msg['measures']:
                if measure['type'] == 'range-bearing':
                    # if exists already this landmark, remove
                    lid=measure['landmark_id']
                    try:
                        graphs['marginals'].remove(
                                next(marg for marg in graphs['marginals'] if marg['var_id'] == lid)
                                )
                    except: 
                        StopIteration
                    # append new marginal
                    r=measure['vect'][0]
                    a=measure['vect'][1]
                    mean={}
                    mean['x'] = last_pose_state['x'] + r*math.cos(a+last_pose_state['th'])
                    mean['y'] = last_pose_state['y'] + r*math.sin(a+last_pose_state['th'])
                    graphs['marginals'].append(
                            {'var_id':measure['landmark_id'],'mean':mean,
                                'covariance':getVisualFromCovMatrix(np.array(measure['covariance']).reshape(2,2))}
                            )
                    # TODO : covariance proper
                    graphs['factors'].append(
                            {'factor_id': factor_id,
                             'type': measure['type'],
                             'vars_id': [node_id, lid],
                             }
                            )
                    factor_idx+=1

                elif measure['type'] == 'range-AA':
                    lid=measure['landmark_id']
                    # if exists already this landmark, remove
                    try:
                        graphs['marginals'].remove(
                                next(marg for marg in graphs['marginals'] if marg['var_id'] == lid)
                                )
                    except: 
                        StopIteration
                    # append new marginal
                    dx=measure['vect'][0]
                    dy=measure['vect'][1]
                    mean={}
                    mean['x'] = last_pose_state['x'] + dx
                    mean['y'] = last_pose_state['y'] + dy
                    graphs['marginals'].append(
                            {'var_id':measure['landmark_id'],'mean':mean,
                                'covariance':getVisualFromCovMatrix(cov[0:2,0:2] +np.array(measure['covariance']).reshape(2,2))}
                            )
                    # TODO : covariance proper
                    graphs['factors'].append(
                            {'factor_id': factor_id,
                             'type': measure['type'],
                             'vars_id': [node_id, lid],
                             }
                            )
                    factor_idx+=1

                else:
                    raise NotImplementedError

            client.publish(graphs_topic_out, json.dumps(graphs))

        elif (message.topic == position_ini_topic):
            print(
                f'\n[Back-end MiniSAM::{robot_id}] R {position_ini_topic} :\n {msg}')
            last_pose_state = msg['state']
            # TODO: give an initial covariance
            # send it already to dataviz manager
            new_reference_state = {
                'robot_id': robot_id, 'state': last_pose_state}
            client.publish(updated_reference_pose_topic_out,
                           json.dumps(new_reference_state))
            node_id = stringify_pose_id(last_pose_idx)
            factor_id = stringify_factor_id(factor_idx)
            graphs['marginals'].append(
                {'var_id': stringify_pose_id(last_pose_idx), 'mean': last_pose_state,
                    'covariance': {'sigma': [0.1, 0.1], 'rot': 0}}  # TODO: raw cov, do the visual conversion in dataviz manager
            )
            graphs['factors'].append(
                {'factor_id': factor_id,
                 'type': 'ini_position',
                 'vars_id': [node_id],
                 }
            )
            # increment the count of factor and node to avoid name collision
            last_pose_idx += 1
            factor_idx += 1
            # publish the graph
            client.publish(graphs_topic_out, json.dumps(graphs))

        elif (message.topic == request_graph_topic_in):
            print(f'\n[Back-end MiniSAM::{robot_id}] R {request_graph_topic_in} \n')
            if not graphs:
                client.publish(graphs_topic_out, json.dumps(graphs))

    print("This is the SLAM back-end for : ", robot_id)

    # -------------------------------------------------------------------------
    #                       MQTT connection
    # -------------------------------------------------------------------------

    mqttc = mqtt.Client('slam-back-end-minisam-py_' + robot_id)
    print("Connecting to broker : ", broker)

    # the on_connect method will set the subscriber
    mqttc.on_connect = on_connect
    # mqttc.on_log=on_log
    mqttc.on_message = on_message

    # connect to the broker
    mqttc.connect(broker)

    # block the code here, process the messages for the subscribed topics

    mqttc.loop_forever()
    print('disconnecting')
    mqttc.disconnect()
