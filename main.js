// DOM related
const body = d3.select("body");
const canvas = d3.select(".canvas");
const canvas_mg = d3.select(".main_group");

const aratio = 0.6;
let value = 250;
// draw
const vgGrobot1 = canvas_mg.append("g").classed("agent", true);

d3.selectAll(".agent")
  .data([{ x: 66, y: 10 }])
  .attr("transform", (d) => "translate(" + d["x"] + "," + d["y"] + ")");

vgGrobot1.append("circle").attr("r", 4);
// .attr("cx", 96)
// .attr("cy", 5)

// ellipse is kept in commentary as a template
// const vgcov = canvas_mg
//   .append("ellipse")
//   .attr('transform', ' translate(50,40) rotate(0)')
//   .attr("rx", 20)
//   .attr("ry", 12);
// .attr("cx", 50)
// .attr("cy", 40)

console.log(`svg height: ${canvas.attr("height")}`);

/******************************************************************************
 *                           SVG Group binding to d3
 *****************************************************************************/
const main_group = d3.select('.main_group')
const agents_true_group = d3.select('.agents_true_group')
const landmarks_true_group = d3.select('.landmarks_true_group')
const agents_graphs_group = d3.select('.agents_graphs_group')
const agents_estimated_group = d3.select('.agents_estimated_group')
// The d_ means a dynamic group as opposed to the groups declared above
//  (I just made that up). Since the names would be only 1-letter appart from
//  their const counter part otherwise, better be safe than sorry with a hard
//  to debug vicious bug.
//  As per the d3 pattern, these selections are empty at this stage and will
//  received elements during the d3 update pattern (in the mqtt callback)
let d_agent_true_group = agents_true_group.selectAll('.agent_true_group')
let d_landmark_true_group = landmarks_true_group.selectAll('.landmark_true_group')
let d_agent_estimated_group = agents_estimated_group.selectAll('.agent_estimated_group')
let d_agent_true_group = agents_true_group.selectAll('.agent_true_group')
let d_agent_graph_group = agents_graphs_group.selectAll('.agent_graph_group')
// For now assume only one hypothesis. TODO: revisit



/******************************************************************************
 *                            FOR TESTING PURPOSE
 *****************************************************************************/
// bound new data, but doesnt 'recompute' the properties that depends on data
setTimeout((_) => d3.selectAll(".agent").data([{ x: 26, y: 10 }]), 1000);
// call a property, as the data refered is already stored (from the 1s timeout)
setTimeout(
  (_) =>
    d3
      .selectAll(".agent")
      .attr("transform", (d) => "translate(" + d["x"] + "," + d["y"] + ")"),
  1500
);
// if we want to change as soon as we update the data, we just call both in a chain
setTimeout(
  (_) =>
    d3
      .selectAll(".agent")
      .data([{ x: 86, y: 50 }])
      .attr("transform", (d) => "translate(" + d["x"] + "," + d["y"] + ")"),
  2000
);
// d3.select("body").transition()
//     .duration(1000)
//     .style("background-color", "red");

// d3.selectAll('.agent')
//   .transition()
//   .duration()

/******************************************************************************
 *                            MQTT events
 *****************************************************************************/

// mqtt log then join with d3
// from the front end I can only access directly an mqtt broker
var client = mqtt.connect("ws://localhost:9001");

client.on("connect", function () {
  console.log("[mqtt] Connected to broker !");

  client.subscribe("ground_truth", function (err) {
    if (!err) {
      console.log("[mqtt] subscribed to the topic >> ground_truth");
    }
  });

  client.subscribe("estimation_graph", function (err) {
    if (!err) {
      console.log("[mqtt] subscribed to the topic >> estimation_graph");
    }
  });
});

// its a global callback fo rall mqtt subs it seems...
client.on("message", function (topic, message) {
  // message is Buffer
  // console.log("$  " + message.toString() + "  << from topic >>  " + topic);

  if (topic == "ground_truth") {
    const msg = JSON.parse(message.toString());
    canvas_mg
      .selectAll(".landmark")
      .data(msg.landmarks)
      .enter()
      .append("circle")
      .attr("cx", (d) => d.state.x)
      .attr("cy", (d) => d.state.y)
      .attr("r", 0.42)
      .classed("landmark", true);
  } else if (topic == "estimation_graph") {
    console.log(`Estimation Graph received : `);
    console.log(JSON.parse(message.toString()));
    // TODO : continue
  }
});

client.publish("presence", "Hello mqtt from JS script");
client.publish("request_ground_truth", " ");

/******************************************************************************
 *                            UI Events
 *****************************************************************************/

let keyPressedBuffer = {
  ArrowUp: false,
  ArrowDown: false,
  ArrowRight: false,
  ArrowLeft: false,
};

// d3 selected robot
let selectedRobot = vgGrobot1;

body.on("keydown", (e) => {
  if (!keyPressedBuffer[e.key]) keyPressedBuffer[e.key] = true;
  const [steerX, steerY] = inputToSteerXY();
  // console.log(steerX, steerY);
  // current state
  curx = selectedRobot.node().transform.baseVal[0].matrix.e;
  cury = selectedRobot.node().transform.baseVal[0].matrix.f;
  curth = selectedRobot.node().transform.baseVal[0].angle;
  // always put translate before rotate
  selectedRobot.attr(
    "transform",
    d3.zoomIdentity.translate(curx + steerX, cury + steerY).toString() +
      " rotate(" +
      curth +
      ")"
  );
  // TODO: send cmd through client
});

body.on("keyup", (e) => (keyPressedBuffer[e.key] = false));

canvas.on("click", () => {
  const msg = {
    header: "some header",
    message: "canvas clicked",
  };
  client.publish("requestChan", JSON.stringify(msg));
  console.log("canvas clicked!");
});

/******************************************************************************
 *                            FOR TESTING PURPOSE 2
 *****************************************************************************/
// sending estimation_graph request_position_ini
setTimeout((_) => client.publish("request_estimation_graph", " "), 2500);
setTimeout((_) => client.publish("request_estimation_graph", "1"), 3500);
setTimeout((_) => client.publish("request_estimation_graph", "2"), 4500);

/******************************************************************************
 *                           KeyPresses Helper
 *****************************************************************************/

function inputToSteerXY() {
  // get key or combination from global buffer
  up = keyPressedBuffer["ArrowUp"];
  down = keyPressedBuffer["ArrowDown"];
  left = keyPressedBuffer["ArrowLeft"];
  right = keyPressedBuffer["ArrowRight"];

  steerX = 0;
  steerY = 0;

  const speed = 0.25;

  if ((up && down) || (right && left)) {
    // if contradictory order(s)
    // nothing to do
  } else {
    steerX += speed * right;
    steerX -= speed * left;
    steerY += speed * up;
    steerY -= speed * down;
  }
  // TODO: split between order reading and state change effect
  return [steerX, steerY];
}

/******************************************************************************
 *                            ALTERNATE MQTT LIB
 *****************************************************************************/

// I prefer the API above for simplicity
// but the few tests I did with paho were ok
// so it is left here for reference
// /********************************************************************
//  *                       PAHO
//  *                       */

// // Create a client instance
// client = new Paho.MQTT.Client('localhost', 9001, "clientId");

// // set callback handlers
// client.onConnectionLost = onConnectionLost;
// client.onMessageArrived = onMessageArrived;

// // connect the client
// client.connect({onSuccess:onConnect});

// // called when the client connects
// function onConnect() {
//   // Once a connection has been made, make a subscription and send a message.
//   console.log("onConnect");
//   client.subscribe("World");
//   message = new Paho.MQTT.Message("Hello");
//   message.destinationName = "World";
//   client.send(message);
// }

// // called when the client loses its connection
// function onConnectionLost(responseObject) {
//   if (responseObject.errorCode !== 0) {
//     console.log("onConnectionLost:"+responseObject.errorMessage);
//   }
// }

// // called when a message arrives
// function onMessageArrived(message) {
//   console.log("onMessageArrived:"+message.payloadString);
// }
//

/******************************************************************************
 *                            HELPER
 *****************************************************************************/
function getRandomInt(max) {
  return Math.floor(Math.random() * Math.floor(max));
}

// Not an in-place method
function randomArraySplice(my_array) {
  return [...my_array].splice(Math.round(Math.random() * my_array.length));
}

// some derpery to prove concept
const poc_data = [2, 14, 26, 35, 44, 52, 68, 81];

let derp = d3.select(".main_group").selectAll("rect");

// call as :       derp = derpery(derp)
// Note: enter uses transition.selection() while update uses call() wrapper on the
//  transition. Afaik there is no difference.
function derpery(my_sel) {
  return my_sel
    .data(randomArraySplice(poc_data), (d) => d)
    .join(
      (enter) =>
        enter
          .append("rect")
          .attr("stroke", "black")
          .attr("stroke-width", 0.1)
          .attr("width", 8)
          .attr("x", (d) => d)
          .transition()
          .duration(750)
          .attr("opacity", 1)
          .attr("fill", "salmon")
          .attr("height", (_) => Math.random() * 55 + 2)
          .transition()
          .duration(750)
          .attr("fill", "lightblue")
          .selection(),
      (update) =>
        update.call((u) =>
          u
            .transition()
            .duration(750)
            .ease(d3.easeCubic)
            .attr("fill", "khaki")
            .attr("height", (_) => Math.random() * 55 + 2)
            .transition()
            .duration(750)
            .attr("fill", "lightblue")
        ),
      (exit) =>
        exit.call((ex_sel) =>
          ex_sel
            .transition()
            .duration(500)
            .attr("fill", "indigo")
            .transition()
            .duration(700)
            .ease(d3.easeCubicIn)
            .attr("opacity", 0)
            .attr("height", 0)
            .remove()
        )
    );
  // .transition()
  // .delay(2250)
  // .duration(750)
  // .attr("fill", "lightblue")
  // .selection();
}
