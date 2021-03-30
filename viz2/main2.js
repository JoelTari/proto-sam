// DOM related
const elBody = d3.select("body");
const elCanvas = d3.select("svg.canvas");
// Define the div for the tooltip
const elDivTooltip = d3
  .select("body")
  .append("div")
  .classed("tooltip", true)
  .style("opacity", 0);

const aratio = 0.6;

// initially no robot is selected
GlobalUI = {
  selected_robot_id: "",
};
/******************************************************************************
 *                           SVG Group binding to d3
 *****************************************************************************/
const elMainGroup = d3.select(".main_group");
const elAgents = elMainGroup
  .append("g")
  .classed("agents", true);
const elLandmarks = elMainGroup
  .append("g")
  .classed("landmarks", true);
// dynamic
let elsLandmark = elLandmarks.selectAll(".landmark");

/******************************************************************************
 *                            MQTT events
 *****************************************************************************/

let client = mqtt.connect("ws://localhost:9001");

client.on("connect", function () {
  console.log("[mqtt] Connected to broker !");

  client.subscribe("ground_truth", function (err) {
    if (!err) {
      console.log("[mqtt] subscribed to the topic >> ground_truth");
    }
  });

  client.subscribe("estimation", function (err) {
    if (!err) {
      console.log("[mqtt] subscribed to the topic >> estimation_graph");
    }
  });
});

client.on("message", function (topic, message) {

  if (topic == "ground_truth") { 
    // This topic should be called in fact metaInfo or something when no ground truth is available
    // create the AgentTeam instanciate the robot objects
    // create a GeUpPa for the landmarks
    const msg = JSON.parse(message.toString());
    // draw landmarks first, I dont do a group for each individual
    // landmark  (no updates as they are supposed fixed)
    if (msg.landmarks != null) {
      elsLandmark = elsLandmark
        .data(msg.landmarks, (d) => d.landmark_id)
        .join(
          (enter) =>
            enter
              .append("path")
              // size is the area, for a cross: area= desired_tot_length**2 *5
              .attr("d", `${d3.symbol(d3.symbolCross, 1.1 * 1.1)()}`)
              .classed("landmark", true)
              .attr("transform", (d) => `translate(${d.state.x},${d.state.y})`)
              .on("mouseover", function (e, d) {
                // TODO: use the UI event function
                //       like others mouseover func do
                // note: use d3.pointer(e) to get pointer coord wrt the target element
                elDivTooltip
                  .style("left", `${e.pageX}px`)
                  .style("top", `${e.pageY - 6}px`)
                  .style("visibility", "visible")
                  // .transition()
                  // .duration(200)
                  .style("opacity", 0.9);
                elDivTooltip.html(`${d.landmark_id}`);
                d3.select(this).style("cursor", "pointer");
              })
              .on("mouseout", mouseout_mg())
          // ,(update) => update // the default if not specified
        );
    }


    // Merge UI and ground_truth
    msg.robots.forEach(
      (r) => (r.isSelected = r.robot_id === GlobalUI.selected_robot_id)
    );

    // TODO continue here: create the AgentTeam and its AgentViz
    msg.robots.forEach(
      r => 
      {
        AgentTeam[r.robot_id] = new AgentViz(r, client, elAgents)
      }
    )


  } else if (topic.split('/').length == 2) {
     const msg = JSON.parse(message.toString());
     const [agent_id, topic_suffix] = topic.split('/')
     AgentTeam.checkSubscriptions(agent_id,topic_suffix,msg)
  }
  

});

/******************************************************************************
 *                            Class AgentViz
 *****************************************************************************/
// AgentTeam: define an object to hold the instances of AgentViz
// AgentTeam['r1']= new AgentViz()...
const AgentTeam = {
  checkSubscriptions : function(agent_id, topic_suffix,msg){
    this[agent_id].mqttProcessTopicSuffix(topic_suffix,msg)
  }
};

class AgentViz{

  constructor(robot_data, mqttc, container){
    this.id = robot_data.robot_id;
    this.d3container = container; // d3 selection inside which this agent will append stuff
    this.mqttc = mqttc;
    this.sensor_svg_path = this.sensorVisual(robot_data.sensor)
    this.history_odom = []   // successive poses of the odometry (from the last graph pose): emptied when a new pose is created on the graph
    this.history_graph = []  // succesives poses of the graph (x0 to x{last_pose})
    this.history_true = []   // successives true poses
    this.max_history_elements = 500 // TODO: apply (maybe not the same size-reducing rules for the 3)...
                                    //      ex: history_true: there could be a min eucl. distance btw elements to reduce size
                                    //          history_odom: remove every other element each time the threshold is reached
    const self = this;
    this.constructD3(self,robot_data);
    this.constructMqtt();
    this.addGroundTruthData(robot_data.state) 
  }
  constructD3 = function(self,robot_data){
    // d3 truth
    this.d3truth = 
      this.d3container
        .append("g")
        .classed("agent_true_group", true)
        .classed("selected", robot_data.isSelected)
        .attr("id", robot_data.robot_id)
        .each(function () {
          d3.select(this)
            .append("g")
            .attr("transform", () => `translate(${robot_data.state.x},${robot_data.state.y})`)
            .append("g")
            .attr("transform", `rotate(${(robot_data.state.th * 180) / Math.PI})`)
            .call(function (g) {
              // adding all display components
              // 1. the sensor
              if (robot_data.sensor != null)
                g.append("path").classed("sensor", true).attr("d", self.sensor_svg_path);
              // 2. the robot
              g.append("polygon")
                .attr("points", "0,-1 0,1 3,0") // TODO: append a <g> first
                .on("mouseover", mouseover_mg(`${robot_data.robot_id}`))
                .on("mouseout", mouseout_mg());
              g.append("line")
                .attr("x1", 0)
                .attr("y1", 0)
                .attr("x2", 1)
                .attr("y2", 0);
            });
          // the state history
          if (self.state_history > 1) {
            d3.select(this)
              .append("polyline")
              .classed("state_history", true)
              // .attr("id", (d) => d.seq)
              // the points of the polyline are the history + the rt state
              .attr(
                "points",
                d.state_history.map((e) => e.join(",")).join(" ") +
                  ` ${robot_data.state.x},${robot_data.state.y}`
              );
          }
        })
        .transition()
        .duration(500)
        .style("opacity", 1)
        .selection()
        .on("click", function () {
          // for next joining of data
          GlobalUI.selected_robot_id = d3.select(this).attr("id");
          // for current data session: put others to false, and the selected to true
          self.d3container.selectAll('.agent_true_group').classed("selected", false);
          d3.select(this).classed("selected", true);
        });
  }
  constructMqtt = function(){
    // do the subscriptions
    this.mqttc.subscribe(`${self.id}/odom`
      , err => {if(!err) console.log(`[mqtt] subscribed to the topic >> ${this.id}/odom`)})
    this.mqttc.subscribe(`${this.id}/graphs`
      , err => {if(!err) console.log(`[mqtt] subscribed to the topic >> ${this.id}/graphs`)})
    this.mqttc.subscribe(`${this.id}/measures`
      , err => {if(!err) console.log(`[mqtt] subscribed to the topic >> ${this.id}/measures`)})
    this.mqttc.subscribe(`${this.id}/ground_truth`
      , err => {if(!err) console.log(`[mqtt] subscribed to the topic >> ${this.id}/ground_truth`)})
    // publishers topic name
    // id/request_ground_truth (expected id/ground_truth back)
    // id/cmd (expected id/odom back)
    this.request_ground_truth_topic = `${this.id}/request_ground_truth`
    this.cmd_topic = `${this.id}/cmd`
  }

  // add ground_truth data
  addGroundTruthData = function(state){
    this.current_true_state = state;
    this.history_true.push(state)

  }

  // function that given the data angle cover and range, draws the string data for the
  // svg-path element
  sensorVisual = function (sensor) {
    const rx = sensor.range;
    const ry = rx;
    if (sensor.angle_coverage > 1 || sensor.angle_coverage < 0)
      console.error("Sensor angle_coverage out of bound");
    // def px py as the starting point along the sensor range arc
    const px = Math.cos(Math.PI * sensor.angle_coverage) * sensor.range;
    const py = Math.sin(Math.PI * sensor.angle_coverage) * sensor.range;
    const sweep = true * (sensor.angle_coverage > 0.5);
    return `M0,0 l${px},${py} A ${rx} ${ry} 0 ${sweep} 0 ${px} ${-py}`;
  }

  // define the callbacks
  odomCallback = function (data){
    console.log('Receive some odom response :' + this.id )
  }
  graphsCallback = function (data){
    console.log('Receive some graph return :' + this.id )
  }
  measuresCallback = function (data){
    console.log('Receive some measure :' + this.id )
  }
  groundTruthCallback = function (data){
    console.log('Receive some GT info :' + this.id )
  }
  // define an entry point for the topic suffix (that will dispatch to appropriate callback)
  mqttProcessTopicSuffix = function(suffix_topic_name, data){
    switch(suffix_topic_name){
      case `odom`:
        this.odomCallback(data);
        break;
      case `graphs`:
        this.graphsCallback(data);
        break;
      case `measures`:
        this.measuresCallback(data);
        break;
      case `ground_truth`:
        this.groundTruthCallback(data);
        break;
      default:
        console.error('unknown suffix topic name : ' + suffix_topic_name)
        break;
    }
  }
  // general update patterns (are used in the callbacks)

};


/******************************************************************************
 *                            UI Events
 *****************************************************************************/

// mouse over-out
function mouseover_mg(text_str) {
  return function (e, d) {
    elDivTooltip
      .style("left", `${e.pageX}px`)
      .style("top", `${e.pageY - 6}px`)
      .style("visibility", "visible")
      // .transition()
      // .duration(200)
      .style("opacity", 0.9);
    elDivTooltip.html(text_str);
    d3.select(this).style("cursor", "pointer");
  };
}
function mouseout_mg() {
  return function (e, d) {
    d3.select(this).style("cursor", "default");
    elDivTooltip
      // .transition()
      // .duration(300)
      .style("opacity", 0)
      .style("visibility", "hidden");
  };
}

// TODO: put this var in globalUI
let keyPressedBuffer = {
  ArrowUp: false,
  ArrowDown: false,
  ArrowRight: false,
  ArrowLeft: false,
};

elBody.on("keydown", (e) => {
  // discard unmanaged keys
  if (
    e.key != "ArrowUp" &&
    e.key != "ArrowDown" &&
    e.key != "ArrowRight" &&
    e.key != "ArrowLeft"
  ) {
    return;
  }

  if (!keyPressedBuffer[e.key]) keyPressedBuffer[e.key] = true;

  inputCmdModel = "DD"; // TODO: centralize in globalUI
  const cmdObj = inputToMove(inputCmdModel);

  client.publish(
    `${GlobalUI.selected_robot_id}/cmd`,
    JSON.stringify({
      robot_id: GlobalUI.selected_robot_id,
      type: inputCmdModel,
      cmd_vel: cmdObj,
    })
  );
});

elCanvas.on("click", (e) => console.log(d3.pointer(e, elMainGroup.node())));

elBody.on("keyup", (e) => (keyPressedBuffer[e.key] = false));

function getTransform_gg(d3_single_selected) {
  // only works on double group descendant framework
  // (first descendant is translation, second is rotation)
  curx = d3_single_selected.selectChild("g").node().transform.baseVal[0].matrix
    .e;
  cury = d3_single_selected.selectChild("g").node().transform.baseVal[0].matrix
    .f;
  curth = d3_single_selected.selectChild("g").selectChild("g").node().transform
    .baseVal[0].angle;
  return [curx, cury, curth];
}

function applyRelativeMove_gg(d3_single_selected, dmove) {
  const [curx, cury, curth] = getTransform_gg(d3_single_selected);
  const [dX, dY, dth] = dmove;
  applyMove_gg(d3_single_selected, [curx + dX, cury + dY, curth + dth]);
}

function applyMove_gg(d3_single_selected, pose) {
  [x, y, th] = pose;
  // I left the transitions related lines commented for posterity
  d3_single_selected
    .selectChild("g")
    .transition("tra")
    .duration(60)
    .attr("transform", "translate(" + x + "," + y + ")")
    .selection()
    .selectChild("g")
    .transition("rot")
    .duration(60)
    .attr("transform", `rotate(${(th * 180) / Math.PI})`);
}

/******************************************************************************
 *                           KeyPresses Helper
 *****************************************************************************/

function inputToMove(model) {
  // get key or combination from global buffer
  up = keyPressedBuffer["ArrowUp"];
  down = keyPressedBuffer["ArrowDown"];
  left = keyPressedBuffer["ArrowLeft"];
  right = keyPressedBuffer["ArrowRight"];

  console.log("Moving using model : " + model); // TODO globalUI
  const speed = 0.5; // TODO: globalUI

  if (model === "AA") {
    dx = 0;
    dy = 0;

    if ((up && down) || (right && left)) {
      // if contradictory order(s)
      // nothing to do
    } else {
      dx += speed * right;
      dx -= speed * left;
      dy += speed * up;
      dy -= speed * down;
    }
    return { x: dx, y: dy };
  } else if (model === "DD") {
    dlinear = 0;
    dangular = 0;
    if ((up && down) || (right && left)) {
      // if contradictory order(s)
      // nothing to do
    } else {
      // TODO: decouple speeds
      dangular -= (speed / 10.0) * right; // rads
      dangular += (speed / 10.0) * left;
      dlinear += speed * up;
      dlinear -= speed * down;
    }
    return { linear: dlinear, angular: dangular };
  } else {
    console.error("Unknown model for sending cmd", model);
  }
}

/******************************************************************************
 *                            HELPER
 *****************************************************************************/
function arraytised(obj_or_array) {
  if (obj_or_array[Symbol.iterator] == null) {
    console.warn("received data is not an array: attempting to arraytised");
    return [obj_or_array];
  }
  // if already an array, all is gud
  else return obj_or_array;
}

function ecpi(a) {
  return Math.atan2(Math.sin(a), Math.cos(a));
}

function indexOfMax(arr) {
  if (arr.length === 0) {
    return -1;
  }

  var max = arr[0];
  var maxIndex = 0;

  for (var i = 1; i < arr.length; i++) {
    if (arr[i] > max) {
      maxIndex = i;
      max = arr[i];
    }
  }

  return maxIndex;
}

function getRandomInt(max) {
  return Math.floor(Math.random() * Math.floor(max));
}

/******************************************************************************
 *                            FOR TESTING PURPOSE 2
 *****************************************************************************/
// sending estimation_graph request_position_ini
client.publish("request_ground_truth", " ");
