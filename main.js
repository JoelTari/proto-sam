// DOM related
const body = d3.select("body");
const canvas = d3.select("svg.canvas");
// Define the div for the tooltip
const div_tooltip = d3
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
const main_group = d3.select(".main_group");
const agents_true_group = main_group
  .append("g")
  .classed("agents_true_group", true);
const landmarks_true_group = main_group
  .append("g")
  .classed("landmarks_true_group", true);

// The d_ means a dynamic group as opposed to the groups declared above
//  (I just made that up). Since the names would be only 1-letter appart from
//  their const counter part otherwise, better be safe than sorry with a hard
//  to debug vicious bug.
//  As per the d3 pattern, these selections are empty at this stage and will
//  received elements during the d3 update pattern (in the mqtt callback)
//  those SelectAll can only be filled with join() once data() is bound
let d_agent_true_group = agents_true_group.selectAll(".agent_true_group");
let d_landmark_true_group = landmarks_true_group.selectAll(
  ".landmark_true_group"
);
let d_robots_estimates_group = main_group
  .append("g")
  .classed("robots_estimates_group", true) // robots & estimates plural
  .selectAll("g.robot_estimates"); // estimates plural (sev. hypothesis), robot sing

/******************************************************************************
 *                            FOR TESTING PURPOSE
 *****************************************************************************/

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

  client.subscribe("estimation", function (err) {
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
    if (msg.landmarks != null) {
      d_landmark_true_group = d_landmark_true_group
        .data(msg.landmarks, (d) => d.landmark_id)
        .join(
          (enter) =>
            enter
              .append("path")
              // size is the area, for a cross: area= desired_tot_length**2 *5
              .attr("d", `${d3.symbol(d3.symbolCross, 1.6 * 1.6)()}`)
              .classed("landmark_true_group", true)
              .attr("transform", (d) => `translate(${d.state.x},${d.state.y})`)
              .on("mouseover", function (e, d) {
                // TODO: use the UI event function
                //       like others mouseover func do
                // note: use d3.pointer(e) to get pointer coord wrt the target element
                div_tooltip
                  .style("left", `${e.pageX}px`)
                  .style("top", `${e.pageY - 6}px`)
                  .style("visibility", "visible")
                  // .transition()
                  // .duration(200)
                  .style("opacity", 0.9);
                div_tooltip.html(`${d.landmark_id}`);
                d3.select(this).style("cursor", "pointer");
              })
              .on("mouseout", mouseout_mg())
          // ,(update) => update // the default if not specified
        );
    }

    // ugly, since updates remove my active/selected that is added outside of d3
    // I got the impression that the general update pattern deletes any excess property added
    // Im kinda joining data from MQTT and the UI
    ////////////////////////////////////////////////////// msg.active = "r2"
    msg.robots.forEach(
      (r) => (r.isSelected = r.robot_id === GlobalUI.selected_robot_id)
    );

    d_agent_true_group = d_agent_true_group
      .data(msg.robots, (d) => d.robot_id)
      .join(join_agent_truth_enter, join_agent_truth_update)
      .on("click", function (e, d) {
        // for next joining of data
        GlobalUI.selected_robot_id = d3.select(this).attr("id");
        // for current data session: put others to false, and the selected to true
        d_agent_true_group.classed("selected", false);
        d3.select(this).classed("selected", true);
      });
  } else if (topic == "estimation") {
    console.log(`Estimation received`);
    // console.log(JSON.parse(message.toString()));

    // convert the string in json
    estimation = JSON.parse(message.toString());

    // if I received an object instead of an array of a single object
    // transform into an array, but give a warning
    estimation = arraytised(estimation);

    // massage data
    estimation.forEach((an_agent_estimation) => {
      estimation_data_massage(an_agent_estimation.graph);
      estimation_query_last_pose(an_agent_estimation);
    });
    // console.log(estimation)

    d_robots_estimates_group = d_robots_estimates_group
      .data(estimation, (d) => d.header.robot_id)
      .join(
        join_enter_robot_estimates,
        join_update_robot_estimates,
        (exit) => exit // NO REMOVE AT THIS LEVEL !!
      )
      // branch off: factor_graph and RT_estimate
      .each(function (_, _, _) {
        // the factor graph first. TODO : hypothesis will be considered first
        d3.select(this)
          .select("g.factor_graph")
          .attr("id", (d) => d.header.robot_id)
          // the factor graph branches off as edges and factors
          .each(function (_, _, _) {
            // d,i,n are the arguments.
            // no need to pass d, as the data binds at the upper level data bounds
            // are available through the data function
            d3.select(this)
              .select("g.factors_group")
              .selectAll(".factor")
              .data(
                function (upper_level_data) {
                  return upper_level_data.graph.factors;
                },
                (d) => d.factor_id
              )
              .join(join_enter_factor, join_update_factor, join_exit_factor);

            d3.select(this)
              .select("g.vertices_group")
              .selectAll(".vertex")
              .data(
                function (upper_level_data) {
                  return upper_level_data.graph.marginals;
                },
                (d) => d.var_id
              )
              .join(join_enter_vertex, join_update_vertex); // TODO: exit vertex
          });

        // now the RT_estimate, that comprises the rt_ghost & the line to link
        // to last pose of the graph
        // d3.select(this).select("g.rt_estimate");
      });
  }
});

/******************************************************************************
 *                          UPDATE PATTERN ROUTINES
 *                  enter,update,exit of the various d3 selections
 *****************************************************************************/

function join_enter_robot_estimates(enter) {
  return (
    enter
      .append("g")
      .classed("robot_estimates", true)
      .attr("id", (d) => d.header.robot_id)
      // prepare underlayers the fg group and the rt_estimate group
      .each(function (d, _, _) {
        // for each robot
        // prepare fg group
        d3.select(this)
          .append("g")
          .classed("factor_graph", true)
          // prepare underlayers: factors_group and vertices_group
          .each(function (_, _, _) {
            // for each graph
            d3.select(this).append("g").classed("factors_group", true);
            d3.select(this).append("g").classed("vertices_group", true);
          });
        // prepare and fill in rt_estimate_group
        d3.select(this)
          .append("g")
          .classed("rt_estimate", true)
          .attr("id", `${d.header.robot_id}`)
          .call(function (g_rt_estimate) {
            const t_ghost_entry = d3.transition("ghost_entry").duration(400);
            const t_ghost_entry2 = d3
              .transition("ghost_entry_last_pose_line")
              .duration(400);
            // the ghost
            g_rt_estimate
              .append("g")
              .classed("rt_ghost", true)
              .on("mouseover", mouseover_mg(`${d.header.robot_id}`))
              .on("mouseout", mouseout_mg())
              // .attr('id',`${d.header.robot_id}`)
              .append("g")
              .attr(
                "transform",
                (local_d) =>
                  "translate(" +
                  local_d.header.state.x +
                  "," +
                  local_d.header.state.y +
                  ")"
              )
              .append("g")
              .attr(
                "transform",
                (local_d) => "rotate(" + local_d.header.state.th + ")"
              )
              .call(function (g_rt_ghost) {
                // 1. the robot
                g_rt_ghost
                  .append("polygon")
                  .attr("points", "0,-1 0,1 3,0")
                  .style("stroke-opacity", 0)
                  .transition(t_ghost_entry)
                  .style("stroke-opacity");

                g_rt_ghost
                  .append("line")
                  .attr("x1", 0)
                  .attr("y1", 0)
                  .attr("x2", 1)
                  .attr("y2", 0)
                  .style("stroke-opacity", 0)
                  .transition(t_ghost_entry)
                  .style("stroke-opacity");
              });
            // the line to the last pose
            g_rt_estimate
              .append("line")
              .classed("rt_line_to_last_pose", true)
              // .attr('id',`${d.header.robot_id}`)
              .attr("x1", d.header.state.x)
              .attr("y1", d.header.state.y)
              .attr("x2", d.last_pose.state.x)
              .attr("y2", d.last_pose.state.y)
              .style("stroke-opacity", 0)
              .transition(t_ghost_entry2)
              .style("stroke-opacity");
          });
      })
  );
}

function join_update_robot_estimates(update) {
  const t_graph_motion = d3
    .transition("m1")
    .duration(1000)
    .ease(d3.easeCubicInOut);
  const t_graph_motion2 = d3
    .transition("m2")
    .duration(1000)
    .ease(d3.easeCubicInOut);

  return update.each(function (_, _, _) {
    d3.select(this)
      .select("g.rt_estimate")
      .call(function (g_rt_estimate) {
        // update the rt ghost dasharray-ed drawing
        g_rt_estimate
          .select("g.rt_ghost")
          .select("g")
          .transition(t_graph_motion)
          .attr(
            "transform",
            (local_d) =>
              "translate(" +
              local_d.header.state.x +
              "," +
              local_d.header.state.y +
              ")"
          )
          .selection()
          .select("g")
          .transition(t_graph_motion2)
          .attr(
            "transform",
            (local_d) => "rotate(" + local_d.header.state.th + ")"
          );
        // .selection();
        // update the line to the last pose
        g_rt_estimate
          .select("line.rt_line_to_last_pose")
          .transition(t_graph_motion)
          .attr("x1", (d) => d.header.state.x)
          .attr("y1", (d) => d.header.state.y)
          .attr("x2", (d) => d.last_pose.state.x)
          .attr("y2", (d) => d.last_pose.state.y);
      });
  });
}

function join_enter_factor(enter) {
  // TODO:
  // Imho best way to avoid to define those transitions everywhere is to
  // transform those functions in classes of which the transitions are members
  const t_factor_entry = d3.transition().duration(400);
  const t_graph_motion = d3.transition().duration(1000).ease(d3.easeCubicInOut);

  return enter
    .append("g")
    .classed("factor", true)
    .attr("id", (d) => d.factor_id)
    .each(function (d) {
      d3.select(this)
        .append("g")
        .attr("transform", "translate(0,0)")
        .append("g")
        .attr("transform", "rotate(0)")
        .style("opacity", 0)
        .transition(t_factor_entry) // ugly (im interest in the child opacity not this node) but necessary to run concurrent transitions on the line (which doesnt work if I place it below)
        .style("opacity")
        .selection()
        .call(function (g) {
          if (d.vars.length > 1) {
            // bi-factor, tri-factor etc...
            d.vars.forEach((v) =>
              g
                .append("line")
                .attr("x1", d.dot_factor_position.x)
                .attr("y1", d.dot_factor_position.y)
                .attr("x2", 0.2 * v.mean.x + 0.8 * d.dot_factor_position.x)
                .attr("y2", 0.2 * v.mean.y + 0.8 * d.dot_factor_position.y)
                .classed(v.var_id, true)
                .transition(t_graph_motion)
                .attr("x1", d.dot_factor_position.x)
                .attr("y1", d.dot_factor_position.y)
                .attr("x2", v.mean.x)
                .attr("y2", v.mean.y)
            );
          } else {
            // unifactor
            g.append("line")
              .attr("x1", d.dot_factor_position.x)
              .attr("y1", d.dot_factor_position.y)
              .attr("x2", d.dot_factor_position.x)
              .attr("y2", d.dot_factor_position.y)
              .transition(t_graph_motion)
              .attr("x1", d.vars[0].mean.x)
              .attr("y1", d.vars[0].mean.y)
              .attr("x2", d.dot_factor_position.x)
              .attr("y2", d.dot_factor_position.y);
          }

          g.append("circle")
            .attr(
              "cx",
              (d) => d.dot_factor_position.x
              // (d) => (d.vars[0].mean.x + d.vars[1].mean.x) / 2
            )
            .attr(
              "cy",
              (d) => d.dot_factor_position.y
              // (d) => (d.vars[0].mean.y + d.vars[1].mean.y) / 2
            )
            // .style("opacity", 0)
            .attr("r", 0.3 * 2)
            .on("mouseover", mouseover_mg(`${d.factor_id}`))
            .on("mouseout", mouseout_mg())
            // opacity transition not necessary here
            .transition("fc")
            .duration(2200)
            // .style("opacity")
            .attr("r", 0.3);
        });
    });
}

function join_update_factor(update) {
  // TODO:
  // Imho best way to avoid to define those transitions everywhere is to
  // transform those functions in classes of which the transitions are members
  const t_graph_motion = d3.transition().duration(1000).ease(d3.easeCubicInOut);

  return update.each(function (d) {
    d3.select(this)
      .select("g")
      .select("g")
      .selectAll("line")
      // .selectChildren("line")
      // .selectChild("line") // TODO: all children
      // .call(function (lines) {
      // lines.each(function (dd, i) {
      .each(function (dd, i) {
        if (d.vars.length > 1) {
          // line
          d3.select(this)
            .transition(t_graph_motion)
            .attr("x1", d.dot_factor_position.x)
            .attr("y1", d.dot_factor_position.y)
            .attr("x2", d.vars[i].mean.x)
            .attr("y2", d.vars[i].mean.y);
        } else {
          // update unary factor
          // WARN TODO: a factor_id should not change its vars_id
          d3.select(this)
            .transition(t_graph_motion)
            .attr("x1", d.vars[0].mean.x)
            .attr("y1", d.vars[0].mean.y)
            .attr("x2", d.dot_factor_position.x)
            .attr("y2", d.dot_factor_position.y);
        }
        // });
      });
    // the little factor circle (to visually differentiate from with MRF)
    d3.select(this)
      .selectChild("g")
      .select("circle")
      .transition(t_graph_motion)
      .attr("cx", d.dot_factor_position.x)
      .attr("cy", d.dot_factor_position.y);
  });
}

function join_exit_factor(exit) {
  return (
    exit
      // .call((ex) =>
      //   ex
      .select("g")
      .call(function (ex) {
        ex.select("g").selectAll("line").style("stroke", "brown");
        ex.select("circle").style("fill", "brown");
      })
      // .selectAll("line")
      // .style("stroke", "brown")
      // )
      // .call((ex) => ex.selectChild("g").select("circle").style("fill", "brown"))
      .transition("exit_factor") // TODO: Define outside
      .duration(1000)
      .style("opacity", 0)
      .remove()
  );
}

function join_enter_vertex(enter) {
  // TODO:
  // Imho best way to avoid to define those transitions everywhere is to
  // transform those functions in classes of which the transitions are members
  const t_vertex_entry = d3.transition().duration(400);

  return enter
    .append("g")
    .classed("vertex", true)
    .attr("id", (d) => d.var_id)
    .each(function (d) {
      d3.select(this)
        .append("g")
        .attr("transform", "translate(" + d.mean.x + "," + d.mean.y + ")")
        .append("g")
        .attr("transform", "rotate(0)")
        .call(function (g) {
          g.append("circle")
            .attr("r", 1 * 3)
            .style("opacity", 0)
            .transition(t_vertex_entry)
            .attr("r", 1)
            .style("opacity");
          // text: variable name inside the circle
          g.append("text")
            .text((d) => d.var_id)
            // .attr("stroke-width", "0.1px")
            .attr("text-anchor", "middle")
            .attr("alignment-baseline", "central")
            .style("opacity", 0)
            .transition(t_vertex_entry)
            .attr("font-size", 1)
            .style("opacity");
          // covariance (-> a rotated group that holds an ellipse)
          g.append("g")
            .attr("transform", `rotate(${(d.covariance.rot * 180) / Math.PI})`)
            .append("ellipse")
            .attr("rx", d.covariance.sigma[0] * Math.sqrt(9.21))
            .attr("ry", d.covariance.sigma[1] * Math.sqrt(9.21))
            .style("opacity", 0) // wow! (see next wow) Nota: doesnt  work with attr()
            .transition(t_vertex_entry)
            .style("opacity"); // wow! this will look for the CSS (has to a style)
        });
    });
}

function join_update_vertex(update) {
  // TODO:
  // Imho best way to avoid to define those transitions everywhere is to
  // transform those functions in classes of which the transitions are members
  const t_graph_motion = d3.transition().duration(1000).ease(d3.easeCubicInOut);

  return update.each(function (d) {
    d3.select(this)
      .selectChild("g")
      .transition(t_graph_motion)
      .attr("transform", "translate(" + d.mean.x + "," + d.mean.y + ")")
      .selection();

    d3.select(this)
      .selectChild("g") //translate
      .selectChild("g") //rotate
      .selectChild("g") // group (incl. rotate)
      .transition(t_graph_motion)
      .attr("transform", `rotate(${(d.covariance.rot * 180) / Math.PI})`)
      .selection()
      .selectChild("ellipse")
      .transition(t_graph_motion)
      .attr("rx", d.covariance.sigma[0] * Math.sqrt(9.21))
      .attr("ry", d.covariance.sigma[1] * Math.sqrt(9.21))
      .selection();
  });
}

function join_exit_vertex(exit) {
  // TODO:
  return exit;
}

// function that given the data angle cover and range, draws the string data for the
// svg-path element
sensor_visual = function (d) {
  rx = d.sensor.range;
  ry = rx;
  if (d.sensor.angle_coverage > 1 || d.sensor.angle_coverage < 0)
    console.error("Sensor angle_coverage out of bound");
  // def px py as the starting point along the sensor range arc
  px = Math.cos(Math.PI * d.sensor.angle_coverage) * d.sensor.range;
  py = Math.sin(Math.PI * d.sensor.angle_coverage) * d.sensor.range;
  sweep = true * (d.sensor.angle_coverage > 0.5);
  return `M0,0 l${px},${py} A ${rx} ${ry} 0 ${sweep} 0 ${px} ${-py}`;
};

function join_agent_truth_enter(enter) {
  return enter
    .append("g")
    .classed("agent_true_group", true)
    .classed("selected", (d) => d.isSelected)
    .attr("id", (d) => d.robot_id)
    .each(function (d) {
      d3.select(this)
        .append("g")
        .attr("transform", (d) => `translate(${d.state.x},${d.state.y})`)
        .append("g")
        .attr("transform", `rotate(${d.state.th})`)
        .call(function (g) {
          // adding all display components
          // 1. the sensor
          if (d.sensor != null)
            g.append("path").classed("sensor", true).attr("d", sensor_visual);
          // 2. the robot
          g.append("polygon")
            .attr("points", "0,-1 0,1 3,0") // TODO: append a <g> first
            .on("mouseover", mouseover_mg(`${d.robot_id}`))
            .on("mouseout", mouseout_mg());
          g.append("line")
            .attr("x1", 0)
            .attr("y1", 0)
            .attr("x2", 1)
            .attr("y2", 0);
        });
      // the state history
      if (d.state_history != null) {
        d3.select(this)
          .append("polyline")
          .classed("state_history", true)
          .attr("id", (d) => d.seq)
          // the points of the polyline are the history + the rt state
          .attr(
            "points",
            d.state_history.map((e) => e.join(",")).join(" ") +
              ` ${d.state.x},${d.state.y}`
          );
      }
    })
    .transition()
    .duration(500)
    .attr("opacity", 1)
    .selection();
}

function join_agent_truth_update(update) {
  return update
    .classed("selected", (d) => d.isSelected)
    .each(function (d) {
      d3.select(this).call(applyMove_gg, [d.state.x, d.state.y, d.state.th]);
      // update the state history (if seq has change in the stamp)
      if (d.state_history != null) {
        d3.select(this)
          .select("polyline.state_history")
          .call(function (polyline) {
            if (d.seq !== polyline.attr("id")) {
              polyline
                .attr("id", (d) => d.seq)
                // the points of the polyline are the history + the rt state
                .attr(
                  "points",
                  d.state_history.map((e) => e.join(",")).join(" ") +
                    ` ${d.state.x},${d.state.y}`
                );
            }
          });
      }
    });
}

/******************************************************************************
 *                            Data Massage estimation
 *****************************************************************************/

// in-place changes to the data structure for convenience when joining
function estimation_data_massage(estimation_data) {
  // Data massage before integration: some data on the vertices array are needed
  // for the factors (1), and the other way around is also true (2)
  // (1) the factors need the position of the vertices (which is found in the data array)
  //     in order to draw the factor/edge at the right position (a line between fact-vertex)
  //     and the position of the full 'dot' representing the factor.
  estimation_data.factors.forEach((f) => {
    f.vars = estimation_data.marginals.filter((marginal) =>
      f.vars_id.includes(marginal.var_id)
    );
    // automagically compute the factor position
    // Obviously (or not), for an unary factor, the factor dot position will reduce
    // to its unique associated node, which is suboptimal...
    if (f.vars.length > 1) {
      f.dot_factor_position = {
        x:
          f.vars.map((a_var) => a_var.mean.x).reduce((a, b) => a + b, 0) /
          f.vars.length,
        y:
          f.vars.map((a_var) => a_var.mean.y).reduce((a, b) => a + b, 0) /
          f.vars.length,
      };
    } else {
      f.dot_factor_position = {
        x: f.vars[0].mean.x,
        y: f.vars[0].mean.y + 5,
      };
    }
  });

  // (2) the unary factors need the neighbors of their associated node to position
  //      intuitively this factor
  //      So the proposed solution is to add a neighbors array to each vertex containing
  //      the vertices id of its neighbors.
  //      This rely on first step
  //      Seems that there is 2 cases, the node has neighbor(s) or has not (typicaly
  //      happens initially with the initial pose)
  estimation_data.factors
    .filter((f) => f.vars_id.length == 1) // unary factor selection
    .forEach((uf) => {
      const unique_node = uf.vars_id[0];
      //vectors of thetas
      const neighbors = estimation_data.factors.filter(
        (f) => f.factor_id !== uf.factor_id && f.vars_id.includes(unique_node)
      ); // neighbors factors of the node associated with that unary factor
      // TODO: care if no neighbor
      if (neighbors.length > 0) {
        // if there are neighbors factors, the unary factor position must be placed
        // at the biggest angle gap
        const thetas = neighbors
          .map((neighbors_f) =>
            Math.atan2(
              neighbors_f.dot_factor_position.y - uf.vars[0].mean.y,
              neighbors_f.dot_factor_position.x - uf.vars[0].mean.x
            )
          )
          .sort((a, b) => a - b); // mandatory sorting

        const thetas_2pi = thetas.map((t) => t - thetas[0]);
        const dthetas2 = thetas_2pi.map((n, i) => {
          if (i !== thetas_2pi.length - 1) {
            return thetas_2pi[i + 1] - n;
          } else return 2 * Math.PI - n;
        });
        const idx_max = indexOfMax(dthetas2);
        const theta_unary = ecpi(thetas[idx_max] + dthetas2[idx_max] / 2);

        // distance of the factor wrt the vertex.
        const squares_distances = neighbors.map(
          (nf) =>
            (nf.dot_factor_position.y - uf.vars[0].mean.y) ** 2 +
            (nf.dot_factor_position.x - uf.vars[0].mean.x) ** 2
        );
        const u_distance = Math.sqrt(
          Math.min(25, Math.max(...squares_distances))
        );
        // TODO: place the hard-coded 25 in globalUI

        // position of factor dot infered from polar coordinates
        const new_uf_position = {
          x: uf.vars[0].mean.x + u_distance * Math.cos(theta_unary),
          y: uf.vars[0].mean.y + u_distance * Math.sin(theta_unary),
        };
        // giving new position
        uf.dot_factor_position = new_uf_position;
      } else {
        // no neighbors
        const theta_unary = Math.PI / 2;
        const u_distance = 5;
        const new_uf_position = {
          x: uf.vars[0].mean.x + u_distance * Math.cos(theta_unary),
          y: uf.vars[0].mean.y + u_distance * Math.sin(theta_unary),
        };
        // giving new position
        uf.dot_factor_position = new_uf_position;
      }
    });
}

function estimation_query_last_pose(an_agent_estimation) {
  an_agent_estimation.last_pose.state = an_agent_estimation.graph.marginals.filter(
    (marginal) => marginal.var_id == an_agent_estimation.last_pose.last_pose_id
  )[0].mean;
}
/******************************************************************************
 *                            UI Events
 *****************************************************************************/

// mouse over-out
function mouseover_mg(text_str) {
  return function (e, d) {
    div_tooltip
      .style("left", `${e.pageX}px`)
      .style("top", `${e.pageY - 6}px`)
      .style("visibility", "visible")
      // .transition()
      // .duration(200)
      .style("opacity", 0.9);
    div_tooltip.html(text_str);
    d3.select(this).style("cursor", "pointer");
  };
}
function mouseout_mg() {
  return function (e, d) {
    d3.select(this).style("cursor", "default");
    div_tooltip
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

body.on("keydown", (e) => {
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

canvas.on("click", (e) => console.log(d3.pointer(e, main_group.node())));

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
setTimeout((_) => client.publish("request_ground_truth", "2"), 2500);
setTimeout((_) => client.publish("request_estimation", " "), 1500);
setTimeout((_) => client.publish("request_estimation", "1"), 2500);
setTimeout((_) => client.publish("request_estimation", "2"), 3500);
setTimeout((_) => client.publish("request_estimation", "3"), 4500);
// setTimeout((_) => client.publish("request_estimation", "4"), 5500);

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
 *                            Prototypes
 *****************************************************************************/
// Not an in-place method
function randomArraySplice(my_array) {
  return [...my_array].splice(Math.round(Math.random() * my_array.length));
}

// some derpery to prove concept
const poc_data = [2, 14, 26, 35, 44, 52, 68, 81];

let hlderp = d3
  .select(".main_group")
  .selectAll(".derp_agent")
  .selectAll("rect");
// a .derp_agent is a group (arbitrary)

const hl_data = [
  { name: "agent1", poc_data: poc_data, pos: 2 },
  { name: "agent2", poc_data: poc_data, pos: 31 },
];

// call with  hlderpery(hl_data)
function hlderpery(bigdata) {
  // has to be a global
  d3.select(".main_group")
    .selectAll(".derp_agent")
    .data(bigdata, (d) => d.name)
    .join(
      (enter) =>
        enter
          .append("g")
          .classed("derp_agent", true)
          .attr("transform", (d) => `translate(0,${d.pos})`),
      (update) => update
    ) // could be simply join('g').classed(...).attr(transform...)
    .selectAll("rect")
    .data(
      function (d) {
        return randomArraySplice(d.poc_data); // d is hl_data[0], hl_data[1] ...
      },
      (d) => d
    )
    .join(derp_enter, derp_update, derp_exit);
}

function derp_enter(enter) {
  enter
    .append("rect")
    .attr("stroke", "black")
    .attr("stroke-width", 0.1)
    .attr("width", 8)
    .attr("x", (d) => d)
    .attr("y", 2)
    .transition()
    .duration(750)
    .attr("opacity", 1)
    .attr("fill", "#5E7146")
    .attr("height", (_) => getRandomInt(25))
    .transition("other")
    .duration(750)
    .attr("fill", "lightblue")
    .selection();
}

function derp_update(u) {
  u.transition("uot")
    .duration(750)
    .ease(d3.easeCubic)
    .attr("fill", "khaki")
    .attr("height", (_) => getRandomInt(25))
    .transition()
    .duration(750)
    .attr("fill", "lightblue");
}
function derp_exit(ex_sel) {
  ex_sel
    .transition("oiuqwerpo")
    .duration(500)
    .attr("fill", "indigo")
    .transition()
    .duration(700)
    .ease(d3.easeCubicIn)
    .attr("opacity", 0)
    .attr("height", 0)
    .remove();
}

// Note: enter uses transition.selection() while update uses call() wrapper on the
//  transition. Afaik there is no difference.
//  Original, non nested selection
let derp = hlderp.selectAll("rect");
function derpery(my_sel) {
  return my_sel
    .data(randomArraySplice(poc_data), (d) => d)
    .join(derp_enter, derp_update, derp_exit);
}
