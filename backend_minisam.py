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
last_pose_state = {'x':0,'y':0,'th':0}
factor_idx = 0
last_cov = np.zeros([3,3])

if __name__ == '__main__':
    # Robot name
    robot_id = sys.argv[1]
    broker = 'localhost'

    #inputs (subs)
    position_ini_topic = '/'.join([robot_id, 'position_ini'])
    measures_pose_topic_in ='/'.join([robot_id,'measures_pose']) 
    #outputs (pubs)
    updated_reference_pose_topic_out = '/'.join([robot_id,'update_reference_pose'])
    request_position_ini_topic ='/'.join([robot_id, 'request_position_ini']) 
    odom_topic_out ='/'.join([robot_id,'odom']) 
    graphs_topic_out = '/'.join([robot_id,'graphs']) 


    graphs={'marginals':[],'factors':[]}

    def ecpi(a):
        return math.atan2(math.sin(a),math.cos(a))

    def stringify_pose_id(count):
        return f'X{count}'

    def stringify_factor_id(count):
        return f'f{count}'

    def getVisualFromCovMatrix(cov : np.ndarray) -> dict:
        eVa,eVec = np.linalg.eig(cov)
        R = eVec
        angle = math.atan2(R[1,0],R[0,0])
        sigmax=math.sqrt(eVa[0])
        sigmay=math.sqrt(eVa[1])
        return { 'sigma': [sigmax, sigmay], 'rot': angle }


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
        time.sleep(2)
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
            x=relative_pose['x']
            y=relative_pose['y']
            th=relative_pose['th']
            new_reference_state = {'robot_id':msg['robot_id'], 'state':{}}
            new_reference_state['state']['x'] = x*math.cos(thref) - y*math.sin(thref)+ xref
            new_reference_state['state']['y'] = x*math.sin(thref) + y*math.cos(thref)+ yref
            new_reference_state['state']['th'] = ecpi(th+thref)
            last_pose_state=new_reference_state['state']
            client.publish(updated_reference_pose_topic_out,json.dumps(new_reference_state))
            # CONTINUE HERE: graph next pose
            node_previous_id = stringify_pose_id(last_pose_idx-1)
            node_id = stringify_pose_id(last_pose_idx)
            factor_id = stringify_factor_id(factor_idx)
            cov = np.array(msg['odom']['covariance']).reshape(3,3) + last_cov
            graphs['marginals'].append(
                {'var_id': node_id, 'mean': last_pose_state,
                    'covariance': getVisualFromCovMatrix(cov)} #TODO: raw cov, do the visual conversion in dataviz manager
                    )
            graphs['factors'].append(
                {'factor_id': factor_id,
                 'type': 'odometry',
                 'vars_id': [node_id,node_previous_id],
                 }
                    )
            last_pose_idx+=1
            factor_idx+=1
            client.publish(graphs_topic_out,json.dumps(graphs))
            last_cov = cov


        elif (message.topic == position_ini_topic):
            print(f'\n[Back-end MiniSAM::{robot_id}] R {position_ini_topic} :\n {msg}')
            last_pose_state = msg['state']
            # TODO: give an initial covariance
            # send it already to dataviz manager
            new_reference_state = {'robot_id':robot_id, 'state':last_pose_state}
            client.publish(updated_reference_pose_topic_out,json.dumps(new_reference_state))
            # TODO : graph first pose x0 and unary factor f0
            node_id = stringify_pose_id(last_pose_idx)
            factor_id = stringify_factor_id(factor_idx)
            graphs['marginals'].append(
                {'var_id': stringify_pose_id(last_pose_idx), 'mean': last_pose_state,
                    'covariance': {'sigma': [2,2], 'rot':0}} #TODO: raw cov, do the visual conversion in dataviz manager
                    )
            graphs['factors'].append(
                {'factor_id': factor_id,
                 'type': 'ini_position',
                 'vars_id': [node_id],
                 }
                    )
            # increment the count of factor and node to avoid name collision 
            last_pose_idx+=1
            factor_idx+=1
            # publish the graph
            client.publish(graphs_topic_out,json.dumps(graphs))


    print("This is the SLAM back-end for : ", robot_id)

    # -------------------------------------------------------------------------
    #                       MQTT connection
    # -------------------------------------------------------------------------

    mqttc = mqtt.Client('slam-back-end-minisam-py_'+ robot_id)
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
