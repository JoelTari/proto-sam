# Mqtt message structures and topics

## TLDR: topic names

- `cmd`
- `cmd_feedback`
- `ground_truth`
- `measures`
- `request_position_ini`
- `position_ini`

## Command: JS to Simu/Robot

Topic name: `cmd`

```json
{
  "robot_id":"r1",
  "type":"AA",       // or DD, ACKER
  "cmd_vel":{"x": dx, "y":dy } # depend on type, could be v,w
}
```
Other types for `cmd_vel`: `"cmd_vel":{"linear": dlinear, "angular": dangular }`

## Command: Simu/Robot to Estimation

Topic name: `cmd_feedback`

```json
{
  "robot_id":"r1",
  "type":"AAcmd",
  "cmd_vel":[dx,dy] ,# depend on type, could be v,w
  "cmd_cov": cov   # listed array of the cov matrix
}
```

## World: simu <-> JS

Topic request: `request_ground_truth`

Topic response: `ground_truth`

```json
{
  "robots": [
    {
      "robot_id": "r1",
      "state": { "x": 5, "y": 10, "th": 0 },
      "sensor": { "range": 18, "angle-coverage": 0.75 }
    }
  ],
  "landmarks": [
    { "id": "l1", "state": { "x": 6, "y": 2 } },
    { "id": "l2", "state": { "x": 35, "y": 10 } }
  ]
}
```

## Measure package: Simu/RobotFrontEnd to Estimation

Topic names: `measures`

```json
{
    "robot_id":"r1",
    "odom":{
      "type": "AAOdom",
      "vect": vect_odom_mes, # serialized vector
      "cov": covariance, # serialized matrix
      },
      "landmarks":[
      {
          "landmark_id": "l2",
          "vect": vect_mes, # serialized vector
          "cov": covariance, # serialized matrix
        },
      {
          "landmark_id": "l5",
          "vect": vect_mes, # serialized vector
          "cov": covariance, # serialized matrix
        },
        ]
  }
```

## Position Initial: Simu <-> Estimation

Request position topic name: `request_position_ini`

Answer position topic name: `position_ini`

```json
# the request
{ "robot_id": 'r2'}
# the response
{
  "robot_id":'r2',
  "type":"AAposition",  # means 2d position, could be SO2position to account for orientation
  "vect_position": vect,  # serialized
  "covariance": cov
}
```

## Estimation resut:  JS <- Estimation (on request)

Request estimation result topic name: `request_estimation_graph`

Answer estimation result topic name:  `estimation_graph`

```python
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
            'vars_id': ['l1', 'l2']},
        {'factor_id': 'f2',
            'type': 'odometry',
            'vars':
            [
                { 'var_id': 'l3', 'mean': {'x': 30, 'y': 32} },
                { 'var_id': 'l1', 'mean': {'x': 20, 'y': 37} },
            ]
         },
        {'factor_id': 'f3',
            'type': 'odometry',
            'vars':
            [
                { 'var_id': 'l3', 'mean': {'x': 30, 'y': 32} },
                { 'var_id': 'l2', 'mean': {'x': 25, 'y': 47} },
            ]
            },
        {'factor_id': 'f5',
            'type': 'odometry',
            'vars':
            [
                { 'var_id': 'l4', 'mean': {'x': 55, 'y': 49} },
                { 'var_id': 'l5', 'mean': {'x': 75, 'y': 25} },
            ]
            },
        {'factor_id': 'f4',
            'type': 'odometry',
            'vars':
            [
                { 'var_id': 'l4', 'mean': {'x': 55, 'y': 49} },
                { 'var_id': 'l3', 'mean': {'x': 30, 'y': 32} },
            ]
            },
        {'factor_id': 'f6',
         'type': 'odometry',
            'vars':
            [
                { 'var_id': 'l6', 'mean': {'x': 26, 'y': 9} },
                { 'var_id': 'l1', 'mean': {'x': 20, 'y': 37} },
            ]
         },
        {'factor_id': 'f7',
         'type': 'odometry',
            'vars':
            [
                { 'var_id': 'l6', 'mean': {'x': 26, 'y': 9} },
                { 'var_id': 'l3', 'mean': {'x': 30, 'y': 32} },
            ]
         },
    ],
    'variable_ordering': ['l2', 'l1', 'l4', 'l3', 'l5']
}
}
```

