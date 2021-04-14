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

reference_pose = {'x':0,'y':0,'th':0}

if __name__ == '__main__':
    # Robot name
    robot_id = sys.argv[1]
    # robot_id = "r2"
    broker = 'localhost'

    #inputs (subs)
    position_ini_topic = '/'.join([robot_id, 'position_ini'])
    odom_topic_in ='/'.join([robot_id,'relative_odom']) 
    updated_reference_pose_topic_in = '/'.join([robot_id,'update_reference_pose'])
    # request_odom_topic_in = '/'.join([robot_id,'request_odom'])
    #outputs (pubs)
    request_position_ini_topic ='/'.join([robot_id, 'request_position_ini']) 
    odom_topic_out ='/'.join([robot_id,'odom']) 
    # TODO: graph

    nd_reference_pose = np.zeros([3,1])
    nd_reference_cov =  np.zeros([3,3])

    def ecpi(a):
        return math.atan2(math.sin(a),math.cos(a))


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
        # client.subscribe(position_ini_topic)
        client.subscribe(odom_topic_in)
        client.subscribe(updated_reference_pose_topic_in)
        # client.subscribe(request_odom_topic_in)
        # TODO graph topic
        # time.sleep(2)
        # client.publish(request_position_ini_topic)


    def on_disconnect(client, userdata, flags, rc=0):
        print('Disconnected result code : ' + str(rc))


    def on_message(client, userdata, message):
        # print("Received message '" + str(message.payload) + "' on topic '"
        #       + message.topic + "' with QoS " + str(message.qos) + "'\n")

        global reference_pose
        # decode payload as string, and transform as JSON
        msg = json.loads(message.payload.decode('utf-8'))
        # depending on the topic, disptach to the user defined functions
        if (message.topic == odom_topic_in):
            odom = copy.deepcopy(msg)
            print(f'\n[SlamDataVizManager::{robot_id}] R {odom_topic_in} :\n {odom}')
            x = msg['state']['x']
            y = msg['state']['y']
            th = msg['state']['th']
            # TODO: reference_pose should be the last graph pose
            xref = reference_pose['x']
            yref = reference_pose['y']
            thref = reference_pose['th']
            # co, si = math.cos(thref), math.sin(thref)
            # R = np.array([[co, -(si),xref],[co,si,yref],[0,0,1]])
            odom['state']['x'] = x*math.cos(thref) - y*math.sin(thref)+ xref
            odom['state']['y'] = x*math.sin(thref) + y*math.cos(thref)+ yref
            odom['state']['th'] = ecpi(odom['state']['th']+thref)
            # visual cov
            try:
                odom['visual_covariance']=getVisualFromCovMatrix(np.array(odom['covariance']).reshape(3,3)[0:2,0:2])
                odom['visual_covariance']['rot'] = ecpi(odom['visual_covariance']['rot']+thref)
            except ValueError:
                pass
            client.publish(odom_topic_out, json.dumps(odom) )
            # last_odom_sent = odom
            # first_time_odom = True
            # reset the aggregate (TODO)
            # reset_aggr()
        # elif (message.topic == position_ini_topic):
        #     print(f'\n[SlamDataVizManager::{robot_id}] R {position_ini_topic} :\n {msg}')
        #     reference_pose = msg['state']
        #     # TODO : remove once back end is integrated
        elif (message.topic == updated_reference_pose_topic_in):
            print(f'\n[SlamDataVizManager::{robot_id}] R {updated_reference_pose_topic_in} :\n {msg}')
            reference_pose = msg['state']
        # elif (message.topic == request_odom_topic_in):
        #     print(f'\n[SlamDataVizManager::{robot_id}] R {request_odom_topic_in}')
        #     if (first_time_odom):



        else:
            raise NotImplementedError


    def getVisualFromCovMatrix(cov : np.ndarray) -> dict:
        eVa,eVec = np.linalg.eig(cov)
        R = eVec
        angle = math.atan2(R[1,0],R[0,0])
        sigmax=math.sqrt(eVa[0])
        sigmay=math.sqrt(eVa[1])
        return { 'sigma': [sigmax, sigmay], 'rot': angle }

    print("This is the SLAM data visualization manager for : ", robot_id)

    # -------------------------------------------------------------------------
    #                       MQTT connection
    # -------------------------------------------------------------------------

    mqttc = mqtt.Client('slam-data-viz-py_'+ robot_id)
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
