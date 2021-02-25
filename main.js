// DOM related
const canvas = d3.select(".canvas");
const svg = canvas.append("svg").attr("width", 600).attr("height", 600);

// define scales here normally

const rect = d3.selectAll("rect");

let value = 250;
// draw
svg
  .append("rect")
  .attr("width", 30)
  .attr("height", value)
  .attr("class", "area")
  .attr("x", 50)
  .attr("y", 10);

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

canvas.on("click", () => {
  const msg = {
    header: "some header",
    message: "canvas clicked",
  };
  client.publish("requestChan", JSON.stringify(msg));
  console.log("canvas clicked!");
});

document.addEventListener("click", () => console.log("document clicked!"));
// // this method ends the client
//  client.end();

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
