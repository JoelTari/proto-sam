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
const elAgents = main_group
  .append("g")
  .classed("agents", true);
const elLandmarks = main_group
  .append("g")
  .classed("landmarks", true);

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
  } else if (topic == "estimation") {
    // should disappear
  }
  
  // AgentTeam.checkSubscriptions()

});

/******************************************************************************
 *                            Class AgentViz
 *****************************************************************************/

class AgentViz{

  constructor(id, mqttc, container){
    this.id = id;
    this.container = container; // d3 selection inside which this agent will append stuff
    this.mqttc = mqttc;
    this.history_odom = []   // successive poses of the odometry (from the last graph pose): emptied when a new pose is created on the graph
    this.history_graph = []  // succesives poses of the graph (x0 to x{last_pose})
    this.history_true = []   // successives poses of the 
    this.max_history_elements = 500 // TODO: apply (maybe not the same size-reducing rules for the 3)...
                                    //      ex: history_true: there could be a min eucl. distance btw elements to reduce size
                                    //          history_odom: remove every other element each time the threshold is reached
    
    // do the subscriptions
    mqttc.subscribe(`${this.id}/odom`
      , err => {if(!err) console.log(`[mqtt] subscribed to the topic >> ${id}/odom`)})
    mqttc.subscribe(`${this.id}/graphs`
      , err => {if(!err) console.log(`[mqtt] subscribed to the topic >> ${id}/graphs`)})
    mqttc.subscribe(`${this.id}/measures`
      , err => {if(!err) console.log(`[mqtt] subscribed to the topic >> ${id}/measures`)})
    mqttc.subscribe(`${this.id}/ground_truth`
      , err => {if(!err) console.log(`[mqtt] subscribed to the topic >> ${id}/ground_truth`)})

    // publishers topic name
    // id/request_ground_truth (expected id/ground_truth back)
    // id/cmd (expected id/odom back)

  }

  // define the callbacks
  odomCallback = function (data){
    // TODO
  }
  graphsCallback = function (data){
    // TODO
  }
  measuresCallback = function (data){
    // TODO
  }
  groundTruthCallback = function (data){
    // TODO
  }
  // define an entry point for the topic suffix (that will dispatch to appropriate callback)
  mqttProcessTopicSuffix = function(suffix_topic_name, msg){
    data = JSON.loads(msg)
    switch(suffix_topic_name){
      case `${this.id}/odom`:
        this.odomCallback(data);
        break;
      case `${this.id}/graphs`:
        this.graphsCallback(data);
        break;
      case `${this.id}/measures`:
        this.measuresCallback(data);
        break;
      case `${this.id}/ground_truth`:
        this.groundTruthCallback(data);
        break;
      default:
        console.error('unknown suffix topic name : ' + suffix_topic_name)
        break;
    }
  }
  // general update patterns (are used in the callbacks

}

// AgentTeam: define an object to hold the instances of AgentViz
// AgentTeam['r1']= new AgentViz()...
