// DOM related
const body = d3.select("body");
const canvas = d3.select(".canvas");
const canvas_mg = d3.select(".main_group");

const aratio = 0.6;

GlobalUI = {
  selected_robot_id: "r1",
};

/******************************************************************************
 *                           SVG Group binding to d3
 *****************************************************************************/
const main_group = d3.select(".main_group");
const agents_true_group = d3.select(".agents_true_group");
const landmarks_true_group = d3.select(".landmarks_true_group");
const agents_graphs_group = d3.select(".agents_graphs_group");
const agents_estimated_group = d3.select(".agents_estimated_group");
// The d_ means a dynamic group as opposed to the groups declared above
//  (I just made that up). Since the names would be only 1-letter appart from
//  their const counter part otherwise, better be safe than sorry with a hard
//  to debug vicious bug.
//  As per the d3 pattern, these selections are empty at this stage and will
//  received elements during the d3 update pattern (in the mqtt callback)
//  those SelectAll can only be filled with join(), append() doesnt seem to work
let d_agent_true_group = agents_true_group.selectAll(".agent_true_group");
let d_landmark_true_group = landmarks_true_group.selectAll(
  ".landmark_true_group"
);
let d_agent_estimated_group = agents_estimated_group.selectAll(
  ".agent_estimated_group"
);
let d_agent_graph_group = agents_graphs_group.selectAll(".agent_graph_group");
// For now assume only one hypothesis. TODO: revisit

/******************************************************************************
 *                            FOR TESTING PURPOSE
 *****************************************************************************/
const graph_test_group = canvas_mg
  .append("g")
  .classed("graph_test_group", true);
let d_edges_group = graph_test_group
  .append("g")
  .classed("edges_graph_test_group", true)
  .selectAll(".edge");
let d_vertices_group = graph_test_group
  .append("g")
  .classed("vertices_graph_test_group", true)
  .selectAll(".vertex");

// // draw
// const vgGrobot1 = canvas_mg.append("g").classed("agent", true);
// d3.selectAll(".agent")
//   .data([{ x: 66, y: 10 }])
//   .attr("transform", (d) => "translate(" + d["x"] + "," + d["y"] + ")");
// vgGrobot1.append("circle").attr("r", 4);
// // .attr("cx", 96)
// // .attr("cy", 5)

// // ellipse is kept in commentary as a template
// // const vgcov = canvas_mg
// //   .append("ellipse")
// //   .attr('transform', ' translate(50,40) rotate(0)')
// //   .attr("rx", 20)
// //   .attr("ry", 12);
// // .attr("cx", 50)
// // .attr("cy", 40)

// console.log(`svg height: ${canvas.attr("height")}`);
// TODO: construct a fake json object here, with 1 and 2 robots
//        ( alike the ground_truth structure that comes through mqtt)

// d_agent_true_group = d_agent_true_group
//   .data([12, 24, 33, 54])
//   .join("g")
//   .transition().duration(1000)
//   .attr('opacity', 1)
//   .selection()
//   .attr("transform", d =>  "translate("+ d +",30) rotate(30)")
//   .classed("agent_true_group", true)
//   .call((g) =>
//     g
//       .append("polygon")
//       .attr("points", "0,-1 0,1 3,0") // TODO: append a <g> first
//       .attr("fill", "linen")
//       .attr("stroke", "black")
//       .attr("stroke-width", 0.1)
//   );

// // bound new data, but doesnt 'recompute' the properties that depends on data
// setTimeout((_) => d3.selectAll(".agent").data([{ x: 26, y: 10 }]), 1000);
// // call a property, as the data refered is already stored (from the 1s timeout)
// setTimeout(
//   (_) =>
//     d3
//       .selectAll(".agent")
//       .attr("transform", (d) => "translate(" + d["x"] + "," + d["y"] + ")"),
//   1500
// );
// // if we want to change as soon as we update the data, we just call both in a chain
// setTimeout(
//   (_) =>
//     d3
//       .selectAll(".agent")
//       .data([{ x: 86, y: 50 }])
//       .attr("transform", (d) => "translate(" + d["x"] + "," + d["y"] + ")"),
//   2000
// );
// // d3.select("body").transition()
// //     .duration(1000)
// //     .style("background-color", "red");

// // d3.selectAll('.agent')
// //   .transition()
// //   .duration()

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
    // draw landmarks first, I dont do a group for each individual
    // landmark  (no updates as they are supposed fixed)
    d_landmark_true_group = d_landmark_true_group
      .data(msg.landmarks, (d) => d.landmark_id)
      .join(
        (enter) =>
          enter
            .append("circle")
            .attr("cx", (d) => d.state.x)
            .attr("cy", (d) => d.state.y)
            .attr("r", 1)
            .classed("landmark_true_group", true)
            .transition()
            .duration(1000)
            .attr("opacity", 1)
            .attr("r", 0.4)
            .selection(),
        (update) => update
      );

    // d_agent_true_group = d_agent_true_group
    //   .data(msg.robots, (d) => d.rodot_id)
    //   .join("g")
    //   .classed("agent_true_group", true)
    //   .attr("id", (d) => d.robot_id)
    //   .on("click", function (e, d) {
    //     console.log(d3.select(this).selectChild("polygon"));
    //     // console.log(d3.select(this).selectChildren('polygon'))
    //     // I want to add the 'selected' class to the clicked agent
    //     // but this should be exclusive, so first remove the 'selected'
    //     // name from the class list of everybody
    //     d_agent_true_group.classed("selected", false);
    //     // now set the concerned entity to 'selected' in the classlist
    //     d3.select(this).classed("selected", true);
    //     // maybe I just need to do that to control it
    //     //  (but there is also the styling to consider anyway)
    //     selectedRobot = d3.select(this);
    //   })
    //   .each(function (d) {
    //     d3.select(this)
    //       .append("g")
    //       .attr(
    //         "transform",
    //         (d) => "translate(" + d.state.x + "," + d.state.y + ")"
    //       )
    //       .append("g")
    //       .attr("transform", "rotate(" + d.state.th + ")")
    //       .call(function (g) {
    //         // adding all display components
    //         g.append("polygon")
    //           .attr("points", "0,-1 0,1 3,0") // TODO: append a <g> first
    //           .attr("fill", "linen")
    //           .attr("stroke", "black")
    //           .attr("stroke-width", 0.1);
    //         g.append("line")
    //           .attr("points", "0,0 0,1")
    //           .attr("stroke", "black")
    //           .attr("stroke-width", 0.1);
    //       });
    //   })
    //   .transition()
    //   .duration(500)
    //   .attr("opacity", 1)
    //   .selection();

    // ugly, since updates remove my active/selected that is added outside of d3
    // I got the impression that the general update pattern deletes any excess property added
    // Im kinda joining data from MQTT and the UI
    ////////////////////////////////////////////////////// msg.active = "r2"
    msg.robots.forEach(
      (r) => (r.isSelected = r.robot_id === GlobalUI.selected_robot_id)
    );
    // msg.robots.forEach(r=> console.log(r))

    d_agent_true_group = d_agent_true_group
      .data(msg.robots, (d) => d.robot_id)
      .join(
        (enter) =>
          enter
            .append("g")
            .classed("agent_true_group", true)
            .classed("selected", (d) => d.isSelected)
            .attr("id", (d) => d.robot_id)
            .each(function (d) {
              d3.select(this)
                .append("g")
                .attr(
                  "transform",
                  (d) => "translate(" + d.state.x + "," + d.state.y + ")"
                )
                .append("g")
                .attr("transform", "rotate(" + d.state.th + ")")
                .call(function (g) {
                  // adding all display components
                  // 1. the sensor
                  if (d.sensor != null)
                    g.append("path")
                      .classed("sensor", true)
                      .attr("d", function (d) {
                        rx = d.sensor.range;
                        ry = rx;
                        if (
                          d.sensor.angle_coverage > 1 ||
                          d.sensor.angle_coverage < 0
                        )
                          console.error("Sensor angle_coverage out of bound");
                        // def px py as the starting point along the sensor range arc
                        px =
                          Math.cos(Math.PI * d.sensor.angle_coverage) *
                          d.sensor.range;
                        py =
                          Math.sin(Math.PI * d.sensor.angle_coverage) *
                          d.sensor.range;
                        return (
                          "M0,0 l" +
                          px +
                          "," +
                          py +
                          " A " +
                          rx +
                          " " +
                          ry +
                          " 0 " +
                          true * (d.sensor.angle_coverage > 0.5) +
                          " 0 " +
                          px +
                          " " +
                          -py
                        );
                      });
                  // 2. the robot
                  g.append("polygon")
                    // .classed('agent_representation',true)
                    .attr("points", "0,-1 0,1 3,0"); // TODO: append a <g> first
                  // .attr("fill", "linen")
                  // .attr("stroke", "black")
                  // .attr("stroke-width", 0.1);
                  g.append("line")
                    // .classed('agent_representation',true)
                    .attr("x1", 0)
                    .attr("y1", 0)
                    .attr("x2", 1)
                    .attr("y2", 0);
                  // .attr("stroke-width", 0.3)
                  // .attr("stroke", (d) => {
                  //   if (d.robot_id === "r1") return "red";
                  //   else if (d.robot_id === "r2") return "blue";
                  //   else if (d.robot_id === "r3") return "green";
                  // });
                });
            })
            .transition()
            .duration(500)
            .attr("opacity", 1)
            .selection(),
        (update) =>
          update
            .classed("selected", (d) => d.isSelected)
            .each(function (d) {
              applyMove_gg(d3.select(this), [d.state.x, d.state.y, d.state.th]);
              // applyMove_gg(d3.select(this), [10, 30, 30]);
            })
      )
      .on("click", function (e, d) {
        // for next joining of data
        GlobalUI.selected_robot_id = d3.select(this).attr("id");
        // for current data session: put others to false, and the selected to true
        d_agent_true_group.classed("selected", false);
        d3.select(this).classed("selected", true);
      });
  } else if (topic == "estimation_graph") {
    console.log(`Estimation Graph received : `);
    console.log(JSON.parse(message.toString()));

    estimation_data = JSON.parse(message.toString());

    // let d_edges_group = graph_test_group.select('.edges')
    // filled by the factors
    // let d_vertices_group = graph_test_group.select('.vertices')
    // filled by the marginals

    // same transition object must applies to edges and vertices for consitent
    // graph motion
    const t_graph_motion = d3
      .transition()
      .duration(1000)
      .ease(d3.easeCubicInOut);

    // the edges (soon to be factors)
    // I need to use additional info from the marginal part
    d_edges_group = d_edges_group
      .data(estimation_data.factors, (d) => d.factor_id)
      .join(
        (enter) =>
          enter
            .append("g")
            .classed("edge", true)
            .attr("id", (d) => d.factor_id)
            .each(function (d) {
              d3.select(this)
                .append("g")
                .attr("transform", "translate(0,0)")
                .append("g")
                .attr("transform", "rotate(0)")
                .call(function (g) {
                  g.append("line") // TODO: replace if different than 2 vars per factor
                    .attr("x1", (d) => d.vars[0].mean.x)
                    .attr("y1", (d) => d.vars[0].mean.y)
                    .attr("x2", (d) => d.vars[1].mean.x)
                    .attr("y2", (d) => d.vars[1].mean.y);
                  g.append("circle")
                    .attr("r", 0.3)
                    .attr("fill", "black")
                    .attr(
                      "cx",
                      (d) => (d.vars[0].mean.x + d.vars[1].mean.x) / 2
                    )
                    .attr(
                      "cy",
                      (d) => (d.vars[0].mean.y + d.vars[1].mean.y) / 2
                    );
                });
            }),
        (update) =>
          update.each(function (d) {
            d3.select(this)
              .selectChild("g")
              .selectChild("g")
              .selectChild("line")
              .transition(t_graph_motion)
              .attr("x1", (d) => d.vars[0].mean.x)
              .attr("y1", (d) => d.vars[0].mean.y)
              .attr("x2", (d) => d.vars[1].mean.x)
              .attr("y2", (d) => d.vars[1].mean.y)
              .selection();
            d3.select(this)
              .selectChild("g")
              .select("circle")
              .transition(t_graph_motion)
              .attr("cx", (d) => (d.vars[0].mean.x + d.vars[1].mean.x) / 2)
              .attr("cy", (d) => (d.vars[0].mean.y + d.vars[1].mean.y) / 2)
              .selection();
          })
      );
    // the vertices
    d_vertices_group = d_vertices_group
      .data(estimation_data.marginals, (d) => d.var_id)
      .join(
        (enter) =>
          enter
            .append("g")
            .classed("vertex", true)
            .attr("id", (d) => d.var_id)
            .each(function (d) {
              d3.select(this)
                .append("g")
                .attr(
                  "transform",
                  "translate(" + d.mean.x + "," + d.mean.y + ")"
                )
                .append("g")
                .attr("transform", "rotate(0)")
                .call(function (g) {
                  g.append("circle").attr("r", 1);
                  g.append("text")
                    .text((d) => d.var_id)
                    .attr("font-size", 1)
                    .attr("stroke-width", "0.1px")
                    .attr("text-anchor", "middle")
                    .attr("alignment-baseline", "central");
                });
            }),
        (update) =>
          update.each(function (d) {
            d3.select(this)
              .selectChild("g")
              .transition(t_graph_motion)
              .attr("transform", "translate(" + d.mean.x + "," + d.mean.y + ")")
              .selection();
          })
      );
  }
});

/******************************************************************************
 *                            UI Events
 *****************************************************************************/

// TODO: put this var in globalUI
let keyPressedBuffer = {
  ArrowUp: false,
  ArrowDown: false,
  ArrowRight: false,
  ArrowLeft: false,
};

body.on("keydown", (e) => {
  if (!keyPressedBuffer[e.key]) keyPressedBuffer[e.key] = true;

  inputCmdModel = "AA"; // TODO: centralize in globalUI
  const cmdObj = inputToMove(inputCmdModel);

  client.publish(
    "cmd",
    JSON.stringify({
      robot_id: GlobalUI.selected_robot_id,
      type: inputCmdModel,
      cmd_vel: cmdObj,
    })
  );
});

body.on("keyup", (e) => (keyPressedBuffer[e.key] = false));

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
    // .transition('rot').duration(60)
    .attr("transform", "rotate(" + th + ")");
}

/******************************************************************************
 *                            FOR TESTING PURPOSE 2
 *****************************************************************************/
// sending estimation_graph request_position_ini
client.publish("request_ground_truth", " ");
setTimeout((_) => client.publish("request_estimation_graph", " "), 2500);
setTimeout((_) => client.publish("request_estimation_graph", "1"), 3500);
setTimeout((_) => client.publish("request_estimation_graph", "2"), 4500);

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
      dangular -= (speed / 5.0) * right; // rads
      dangular += (speed / 5.0) * left;
      dlinear += speed * up;
      dlinear -= speed * down;
    }
    return { linear: dlinear, angular: dangular };
  } else {
    console.error("Unknown model for sending cmd", model);
  }
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
