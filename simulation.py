#!/usr/bin/python3

import paho.mqtt.client as mqtt
import time
import json
import numpy as np
import copy
import math
import random as rd
import numpy.random as nprd

# TODO introduce global flags such as type of commands, AA etc..
# TODO dynamic change of std-dev noise

# ----------------------------------------------------------------------------
#                           Globals
# ----------------------------------------------------------------------------
world = {
    "robots":
    [
        {
            "robot_id": "r1",
            "state": {"x": 5, "y": 6.1, "th": 30},
            "sensor": {"range": 18, "angle-coverage": 0.75}
        },
        {
            "robot_id": "r2",
            "state": {"x": 53, "y": 16.1, "th": -150},
            "sensor": {"range": 18, "angle-coverage": 0.75}
        },
        {
            "robot_id": "r3",
            "state": {"x": 25, "y": 56.1, "th": 100},
            "sensor": {"range": 18, "angle-coverage": 0.75}
        },
    ],
    "landmarks":
    [
        {"landmark_id": "l1", "state": {"x": 9, "y": 46}},
        {"landmark_id": "l2", "state": {"x": 13, "y": 4}},
        {"landmark_id": "l3", "state": {"x": 13.5, "y": 55}},
        {"landmark_id": "l4", "state": {"x": 14, "y": 30}},
        {"landmark_id": "l5", "state": {"x": 27, "y": 31}},
        {"landmark_id": "l6", "state": {"x": 21, "y": 34}},
        {"landmark_id": "l7", "state": {"x": 45, "y": 10}},
        {"landmark_id": "l8", "state": {"x": 45, "y": 50}},
        {"landmark_id": "l9", "state": {"x": 61, "y": 11}},
        {"landmark_id": "l10", "state": {"x": 63, "y": 46}},
        {"landmark_id": "l11", "state": {"x": 67, "y": 27}},
        {"landmark_id": "l12", "state": {"x": 71, "y": 51}},
        {"landmark_id": "l13", "state": {"x": 77, "y": 16}},
        {"landmark_id": "l14", "state": {"x": 80, "y": 23}},
        {"landmark_id": "l15", "state": {"x": 84, "y": 35}},
        {"landmark_id": "l16", "state": {"x": 96, "y": 56}},
    ]
}
# store positions ini of each agent
robots_ini_poses = [{"robot_id": r["robot_id"],
                     "position_ini":r["state"]} for r in world['robots']]
print(robots_ini_poses)


# last referenced pose to decide when to generate a new odometry measurement
last_pose = copy.deepcopy(world['robots'])

new_pose_distance_threshold = 7.2

sensor_range = 15

# seeding randomness (for the numpy stuff)
# https://numpy.org/doc/stable/reference/random/index.html#random-quick-start
rng = np.random.default_rng()

# noise cmd (diag)
# 0 -> no noise
cmd_std_dev_ratio_y = 0.5/100
cmd_std_dev_ratio_x = 1.0/100

# meas noise (diag)
# 0 -> no noise
# lets say a std dev of 3cm for each measured unit on X
# lets say a std dev of 2cm for each measured meter on Y
measure_std_dev_ratio_x = 3.0/100
measure_std_dev_ratio_y = 2.0/100

# the cumulative odom store the effect of the accumulation of cmd topics.
# Once the distance threshold is reached, this object is consummed and reset
# in order to construct the odometry measurement.
# It evolved like the variance of a random walk.
cumulative_odom_cov = np.zeros([2, 2])

# some mqtt related globals
broker = 'localhost'

cmd_topic = 'cmd'
cmd_feedback_topic = 'cmd_feedback'
measures_topic = 'measures'
request_ground_truth_topic = 'request_ground_truth'
ground_truth_topic = 'ground_truth'
request_position_ini_topic = 'request_position_ini'
position_ini_topic = 'position_ini'

# init flag
firstTime = True

# ----------------------------------------------------------------------------
#                           math utility
# ----------------------------------------------------------------------------


def sq_dist(p1: dict, p2: dict) -> float:
    return (p1['x']-p2['x'])**2+(p1['y']-p2['y'])**2


def sqrt_dist(p1: dict, p2: dict) -> float:
    return math.sqrt(sq_dist(p1, p2))


def dx(p_ref: dict, p_target: dict) -> float:
    return p_target['x']-p_ref['x']


def dy(p_ref: dict, p_target: dict) -> float:
    return p_target['y']-p_ref['y']


def nosify_cmd(cmd: list):
    exact_cmd = np.array([cmd]).T
    print(f'exact_cmd is of shape {exact_cmd.shape}')
    # compute the covariance that will generate the noise
    cov = generate_covariance_noise(
        exact_cmd, [cmd_std_dev_ratio_x, cmd_std_dev_ratio_y])
    # generate the random measure with additive noise
    return exact_cmd + rng.multivariate_normal(np.zeros(2), cov).reshape(2, 1), cov


def measure_robot_landmark(robotstate: dict, landmark: dict) -> dict:
    xl = landmark['state']['x']
    yl = landmark['state']['y']
    xr = robotstate['x']
    yr = robotstate['y']
    thr = robotstate['th']

    # an exact measurement would be
    exact_measure = np.array([[xl-xr, yl-yr]]).T

    # compute the covariance that will generate the noise
    cov = generate_covariance_noise(
        exact_measure, [measure_std_dev_ratio_x, measure_std_dev_ratio_y])

    # generate the random measure with additive noise
    vect = exact_measure + \
        rng.multivariate_normal(np.zeros(2), cov).reshape(2, 1)
    # return the measure and its covariance confidence
    # Important: the covariance returned corresponds to the real noise,
    #            which might be different than in reality
    # lastly, the np objects are serialized
    return {'landmark_id': landmark['landmark_id'], 'vect': vect.reshape(2,).tolist(), 'covariance': cov.reshape(4,).tolist()}

# TODO : range-bearing sensor mes
#       bearing only sensor mes


def generate_odom_measurement(current_pose: dict, last_pose: dict):
    global cumulative_odom_cov
    # exact odometry measurement
    exact_odom_mes = np.array(
        [[dx(last_pose, current_pose), dy(last_pose, current_pose)]])

    # TODO change when its 3x3 with theta
    # generate the noisy odometry according to the cumulative noise
    vect_odom_mes = exact_odom_mes + \
        rng.multivariate_normal(np.zeros(2), cumulative_odom_cov)

    full_odom_mes = {'type': 'AAOdom',        # AA means axis-aligned
                     'vect': vect_odom_mes.reshape(2,).tolist(),
                     'covariance': cumulative_odom_cov.reshape(4,).tolist()
                     }

    # reset cumulative_cov value until next time
    cumulative_odom_cov = np.zeros([2, 2])

    return full_odom_mes


def in_sensor_coverage(sensorPos: dict, target: dict, sensor_info: dict) -> bool:
    # TODO: add angle coverage
    isInRange = sqrt_dist(sensorPos, target) < sensor_info['range']
    isAngleCovered = True
    return isInRange and isAngleCovered


def generate_covariance_noise(exact_vect: np.ndarray, std_dev_stats: list, isNoiseAxisAligned=False, maxCovarianceSkew=math.pi/8) -> np.ndarray:
    # axis aligned covariance I want for odom measurement
    cov_AA = np.square(np.diag(std_dev_stats)
                       @ np.diag(exact_vect.reshape(2,).tolist()))

    if not isNoiseAxisAligned:
        # a rotational angle is defined randomly to skew the axis aligned covariance
        rot_angle = rd.uniform(-maxCovarianceSkew, maxCovarianceSkew)
        rot_mat = np.array([[math.cos(rot_angle), - math.sin(rot_angle)],
                            [math.sin(rot_angle), math.cos(rot_angle)]])
        cov = rot_mat @ cov_AA @ rot_mat.T
    else:
        cov = cov_AA
    return cov


def integrate_cumulative_odometry(cmd_cov: np.ndarray) -> None:
    global cumulative_odom_cov
    cumulative_odom_cov += cmd_cov
    # TODO if its not linear, use recursive bayesian filter
    #       In linear X,Y this is easy because the cmd and the position are on the same space
    #       But in NL, the command is v,w and the pos is 3d (x y theta)
    #       In that case, apply the recursive linearized filter, which the update part of the traditional EKF SLAM in 2.5d

# ----------------------------------------------------------------------------
#                        User defined callback funtions
# ----------------------------------------------------------------------------


def cmd_vel_callback(client, msg):
    print('cmd')
    received_cmd = json.loads(msg)
    # 1 noisify the order and update cumulative odom cov
    noisy_cmd, cov_cmd = nosify_cmd(received_cmd['cmd_vel'])
    print('cov')
    print(cov_cmd)
    integrate_cumulative_odometry(cov_cmd)
    # TODO publish cmd feedback not at every cmd, device some shenaningans (but lets keep it 1 thread)
    client.publish(cmd_feedback_topic, json.dumps(
        {'robot_id': received_cmd['robot_id'], 'type': 'AAcmd', 'cmd': noisy_cmd.reshape(2,).tolist(), 'cmd_cov': cov_cmd.reshape(4,).tolist()}))
    # 2 update robot position
    # retrieve a pointer to the robot in the structure
    robot_gidx = (i for i, d in enumerate(
        world['robots']) if d['robot_id'] == received_cmd['robot_id'])
    # stopit error if this robot_id doesnt exist
    robot_idx = next(robot_gidx)
    robot = world['robots'][robot_idx]
    # retrieve a pointer to the robot position & make a copy
    robotPos = robot['state']
    cur_state = copy.copy(robotPos)
    # use np arrays for operations
    # TODO: dont forget theta
    npar_cur_state = np.array([[robotPos['x'], robotPos['y']]]).T
    npar_new_state = npar_cur_state + noisy_cmd
    print(noisy_cmd)
    # back to dictionaries to save in the world structure
    robotPos['x'] = npar_new_state[0, 0]
    robotPos['y'] = npar_new_state[1, 0]
    print(f"new state   {npar_new_state}")
    # # some trace
    # print('Current state of robot ' + order['id'])
    # print(cur_state)
    # print('New state :')
    # print(world['robots'][idx]['state'])

    # 3 publish ground truth
    client.publish(ground_truth_topic, json.dumps(world))
    # 4 check if new odom measurement should be generated
    # 4.1 if no, return
    global last_pose
    robot_last_pose_node = last_pose[robot_idx]['state']
    if sqrt_dist(robotPos, robot_last_pose_node) < new_pose_distance_threshold:
        return

        # 4.2 if yes, generate sensor suite
    else:
        # 4.2.1 generate odom measure
        odom_mes = generate_odom_measurement(
            robotPos, robot_last_pose_node)
        # 4.2.2 generate robot to landmarks measure
        #       np arrays are not json serializable, so I to convert them to list first
        #       with the np::tolist() method
        sensor_info = robot['sensor']
        # comprehension list
        gen_landmarks_measurements = (
            measure_robot_landmark(robotPos, l)
            for l in world['landmarks']
            if in_sensor_coverage(robotPos, l['state'], sensor_info)
        )
        # cast the measures generator as a list
        landmarks_mes = list(gen_landmarks_measurements)
        # 4.2.3 save the new robot position as the last pose node
        robot_last_pose_node = robotPos  # TODO check that its a proper value affectation
        last_pose[robot_idx]['state'] = copy.deepcopy(robotPos)
        # 4.2.4 publish the measurements in the same package
        mes_payload = {"robot_id": robot_idx,
                       "odom": odom_mes,
                       "landmarks": landmarks_mes
                       }
        client.publish(measures_topic, json.dumps(mes_payload))


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
    client.subscribe(cmd_topic)
    client.subscribe(request_position_ini_topic)
    client.subscribe(request_ground_truth_topic)


def on_disconnect(client, userdata, flags, rc=0):
    print('Disconnected result code : ' + str(rc))


def on_message(client, userdata, message):
    print("Received message '" + str(message.payload) + "' on topic '"
          + message.topic + "' with QoS " + str(message.qos) + "'\n")

    # decode payload as string
    msg = message.payload.decode('utf-8')
    # depending on the topic, disptach to the user defined functions
    if message.topic == cmd_topic:
        cmd_vel_callback(client, msg)
    elif message.topic == request_ground_truth_topic:
        client.publish(ground_truth_topic, json.dumps(world))
    elif message.topic == request_position_ini_topic:
        client.publish(position_ini_topic, json.dumps(robots_ini_poses))

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
