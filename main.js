// DOM related
const body = d3.select("body");
const canvas = d3.select(".canvas");
const canvas_mg = d3.select(".main_group");

const aratio = 0.6;
let value = 250;
// draw
const vgGrobot1 = canvas_mg
  .append("g")
  .attr("transform", "translate(96,5)")
  .classed("nono", true);

vgGrobot1.append("circle").attr("r", 4);
// .attr("cx", 96)
// .attr("cy", 5)

const vgcov = canvas_mg
  .append("ellipse")
  .attr("cx", 50)
  .attr("cy", 40)
  .attr("rx", 20)
  .attr("ry", 15);

// canvas
//   .append("rect")
//   .attr("width", 30)
//   .attr("height", value)
//   .attr("class", "area")
//   .attr("x", 50)
//   .attr("y", 10);

console.log(`svg height: ${canvas.attr("height")}`);

/******************************************************************************
 *                            MQTT events
 *****************************************************************************/

// mqtt log then join with d3
// from the front end I can only access directly an mqtt broker
var client = mqtt.connect("ws://localhost:9001");

client.on("connect", function () {
  console.log("connected !");
  client.subscribe("presence", function (err) {
    if (!err) {
      console.log("[mqtt] subscribed to the topic >> presence");
    }
  });
  client.subscribe("atopic", function (err) {
    if (!err) {
      console.log("[mqtt] subscribed to the topic >> atopic");
    }
  });
});

// its a global callback fo rall mqtt subs it seems...
client.on("message", function (topic, message) {
  // message is Buffer
  console.log("$  " + message.toString() + "  << from topic >>  " + topic);

  // this is where I diverge from the example and update d3 stuff
  if (topic == "atopic") {
    const msg = JSON.parse(message.toString());
    d3.select("rect").attr("height", msg.value);
  }
});

client.publish("presence", "Hello mqtt from JS script");

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
  const [steerX, steerY] = inputToSteerXY(selectedRobot);
  // console.log(steerX, steerY);
  // current state
  curx = selectedRobot.node().transform.baseVal[0].matrix.e;
  cury = selectedRobot.node().transform.baseVal[0].matrix.f;
  curth = selectedRobot.node().transform.baseVal[0].angle;
  // always put rotate before translate
  selectedRobot.attr(
    "transform",
    d3.zoomIdentity.translate(curx + steerX, cury + steerY).toString()
  );
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
 *                           KeyPresses Helper
 *****************************************************************************/

function inputToSteerXY(sel_robot) {
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
