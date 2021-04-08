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
    #outputs (pubs)
    request_position_ini_topic ='/'.join([robot_id, 'request_position_ini']) 
    odom_topic_out ='/'.join([robot_id,'odom']) 
    # TODO: graph

    nd_reference_pose = np.zeros([3,1])
    nd_reference_cov =  np.zeros([3,3])

# aggr_state
    
# # return zero_state,zero_cov
#     def reset_aggr():
#         return np.zeros([3,1]),np.zeros([3,3])

    def ecpi(a):
        return math.atan2(math.sin(a),math.cos(a))

#     def compute_helper_odom_DD(cmd_dd: np.ndarray
#                             , cmd_dd_cov: np.ndarray
#                             , current_th
#                             , dt = 1) -> np.ndarray:
#         th=current_th
#         v = cmd_dd[0,0]
#         w = cmd_dd[1,0]
#         sth = math.sin(th)
#         sth_n = math.sin(th+w*dt)          # sin(theta_new)
#         cth = math.cos(th)
#         cth_n = math.cos(th+w*dt)
#         if (math.fabs(w) > 1e-4 and math.fabs(v/w) < 1e+4):
#             # V
#             V = np.array([[(sth_n-sth)/w, v*(sth-sth_n)/w**2+ v*cth_n*dt/w]
#                         , [(cth-cth_n)/w,-v*(cth-cth_n)/w**2+ v*sth_n*dt/w]
#                         , [        0     ,       dt                       ]])
#             # G
#             G = np.array([[ 1 , 0 , v/w*(cth_n - cth)]
#                          ,[ 0 , 1 , v/w*(sth_n - sth)]
#                          ,[ 0 , 0 ,         1        ]])
#             # dstate_vec (the additive state)
#             dstate_vec = np.array([[v/w*(sth_n - sth)
#                                    ,v/w*(cth - cth_n)
#                                    ,    w*dt]]).T
#         else : # euler
#             # V
#             V = np.array([[cth*dt, 0]
#                          ,[sth*dt, 0]
#                          ,[ 0    ,dt]])
#             # G
#             G = np.array([[ 1 , 0 , -v*sth*dt]
#                          ,[ 0 , 1 ,  v*cth*dt]
#                          ,[ 0 , 0 ,     1    ]])
#             # dstate_vec (the additive state)
#             dstate_vec = np.array([[v*cth*dt,v*sth*dt,w*dt]]).T

#         # TODO: this belong to the simu
#         # M = np.array([[alpha1*v**2,0],[0,alpha2*w**2]]) 
#         M = cmd_dd_cov
#         return G , V@M@V.T, dstate_vec

#     def process_cmd_feedback_AA(cmd_AA_vec: np.ndarray,cmd_AA_cov : np.ndarray):
#         # Update step of the odom covariance
#         global g_aggr_cov
#         global g_aggr_state
#         g_aggr_cov[0:2,0:2] += cmd_AA_cov
#         g_aggr_state[0:2] += cmd_AA_vec

#     def process_cmd_feedback_DD(cmd_DD_vec: np.ndarray,cmd_DD_cov : np.ndarray):

#         global g_aggr_state
#         # compute the helper that will help us compute the new covariance state
#         current_th = g_aggr_state[2,0]
#         G,R, dstate_vec = compute_helper_odom_DD(cmd_DD_vec,cmd_DD_cov,current_th)

#         # Update step of the odom covariance
#         global g_aggr_cov
#         g_aggr_cov = G@g_aggr_cov@G.T + R
#         g_aggr_state += dstate_vec
#         g_aggr_state[2,0] = ecpi(g_aggr_state[2,0])

#         # Pb, TODO : comment le back end va s'en servir de ce facteur (re-lineariser)
#         #           (ignorer pb pour l'instant)




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
        client.subscribe(odom_topic_in)
        # TODO graph topic
        time.sleep(1)
        client.publish(request_position_ini_topic)


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
            xref = reference_pose['x']
            yref = reference_pose['y']
            thref = reference_pose['th']
            # co, si = math.cos(thref), math.sin(thref)
            # R = np.array([[co, -(si),xref],[co,si,yref],[0,0,1]])
            odom['state']['x'] = x*math.cos(thref) - y*math.sin(thref)+ xref
            odom['state']['y'] = x*math.sin(thref) + y*math.cos(thref)+ yref
            odom['state']['th'] = ecpi(odom['state']['th']+thref)
            # visual cov
            odom['visual_covariance']=getVisualFromCovMatrix(np.array(odom['covariance']).reshape(3,3)[0:2,0:2])
            odom['visual_covariance']['rot'] = ecpi(odom['visual_covariance']['rot']+thref)
            client.publish(odom_topic_out, json.dumps(odom) )
            # reset the aggregate (TODO)
            # reset_aggr()
        elif (message.topic == position_ini_topic):
            print(f'\n[SlamDataVizManager::{robot_id}] R {position_ini_topic} :\n {msg}')
            reference_pose = msg['state']
            # TODO : remove once back end is integrated

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
