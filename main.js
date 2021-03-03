// DOM related
const body = d3.select("body");
const canvas = d3.select(".canvas");
const canvas_mg = d3.select(".main_group");

const aratio = 0.6;
let value = 250;
// draw
const vgGrobot1 = canvas_mg
  .append("g")
  .classed("agent", true);

d3.selectAll('.agent')
  .data([{'x':66,'y':10}])
  .attr("transform", (d) => "translate("+ d['x'] +','+ d['y']+")")

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
 *                            FOR TESTING PURPOSE
 *****************************************************************************/
// bound new data, but doesnt 'recompute' the properties that depends on data
setTimeout(_=> 
d3.selectAll('.agent')
  .data([{'x':26,'y':10}])
,1000)
// call a property, as the data refered is already stored (from the 1s timeout) 
setTimeout(_=> 
d3.selectAll('.agent')
  .attr("transform", (d) => "translate("+ d['x'] +','+ d['y']+")")
,1500)
// if we want to change as soon as we update the data, we just call both in a chain
setTimeout(_=> 
d3.selectAll('.agent')
  .data([{'x':86,'y':50}])
  .attr("transform", (d) => "translate("+ d['x'] +','+ d['y']+")")
,2000)
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

  if (topic == "ground_truth"){
    const msg = JSON.parse(message.toString());
    canvas_mg.selectAll('.landmark')
        .data(msg.landmarks)
        .enter()
        .append('circle')
        .attr('cx',d => d.state.x)
        .attr('cy',d => d.state.y)
        .attr('r',0.42)
        .classed('landmark',true)
  }
  else if (topic == "estimation_graph"){
    console.log(`Estimation Graph received : `)
    console.log(JSON.parse(message.toString()))
    // TODO : continue
  }
});


client.publish("presence", "Hello mqtt from JS script");
client.publish('request_ground_truth'," ");

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
    d3.zoomIdentity.translate(curx + steerX, cury + steerY).toString()
    + ' rotate('+ curth +')' 
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
setTimeout(_ => client.publish('request_estimation_graph',' '),2500)
setTimeout(_ => client.publish('request_estimation_graph','1'),3500)
setTimeout(_ => client.publish('request_estimation_graph','2'),4500)

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
