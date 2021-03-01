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
  "type":"AAcmd",
  "cmd_vel":[dx,dy] # depend on type, could be v,w
}
```

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
