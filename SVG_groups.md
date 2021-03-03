# SVG Groups

## Main Canvas : graph

At the root, we find the svg tag of class _canvas_. Then, the tree is :

- `main_group` as an svg group
  - `agents_true_group` the robots, the real ones (if GT available)!
    - For each robot: `id_robot_true_group` whose descendants are the graphical
      components. Also the sensor coverage !
      **Class list**: `agent` `[selected]`
  - `landmarks_true_group` are the landmarks (if GT available)
    - For each landmark : `id_landmark_true_group` that contains simply a circle.
      **Class list**: `landmark`
  - `agents_estimated_group` the robots as estimated
    - For each robot: `id_robot_estimated_group` whose descendants are the graphical
      components.
  - `landmark_estimated_group` are the landmarks estimated positions (not used
    for now).
  - `graphs` : subdivided (depth) in two groups
    - `id_robot_graphs` : has a classlist of `[selected]`, as one robot can maintain
      several hypothesis (in nonlinear context):
      - For each graph: `id_robot_id_graph_group`: class `main_hypothesis` which is composed of:
        - `factors_group`
          - `id_factors_group` (TODO: deal with factors sharing the same vertices set)
        - `edges_group`
          - `id_edges_group` : containing a line
        - `vertices_group`
          - `id_vertex_group` : contains a circle, and a cov ellipse. A tooltip
            (selection.on('mouseover') to be defined)
- `axes_group` (to be defined)

Ideas :

- Which groups should have `.on('mouseover')` tooltips ?

```dot
digraph G {
  rankdir="TB"
  penwidth=2;
  labelloc="t";
  label="SVG Group Hierarchy"
  labelloc="b";
  node [fontname="Helvetica sans-serif",fontsize=11,style=filled,color=azure2];

  main_group -> agents_true_group
  subgraph cluster_0 {
    style=rounded;
    bgcolor=ivory;
    color=red;
    ID_agent_true_group;
    label = "Number of agents";
  }
  agents_true_group -> ID_agent_true_group


  main_group -> landmarks_true_group

  subgraph cluster_2 {
    style=rounded;
    bgcolor=ivory;
    color=blue;
    ID_landmark_true_group;
    label = "Number of landmarks";
  }
  landmarks_true_group -> ID_landmark_true_group

  main_group -> agents_graphs_group
  subgraph cluster_3 {
    style=rounded;
    bgcolor=ivory;
    color=red;
    ID_agent_graphs_group;
    label = "Number of agents";
  }
  agents_graphs_group -> ID_agent_graphs_group

  subgraph cluster_4 {
    style=rounded;
    bgcolor=ivory;
    color=magenta;
    ID_agent_ID_graph_group;
    label = "Number of hypothesis per agent";
  }
  ID_agent_graphs_group -> ID_agent_ID_graph_group

ID_agent_ID_graph_group-> ID_agent_ID_graph_vertices_group
  subgraph cluster_6 {
    style=rounded;
    bgcolor=ivory;
    color=orange;
    ID_agent_ID_graph_ID_vertices_group;
    label = "Number of vertices per agent";
  }
ID_agent_ID_graph_vertices_group -> ID_agent_ID_graph_ID_vertices_group

ID_agent_ID_graph_group-> ID_agent_ID_graph_factors_group
  subgraph cluster_5 {
    style=rounded;
    bgcolor=ivory;
    color=green;
    ID_agent_ID_graph_ID_factors_group;
    label = "Number of factors per agent";
  }
ID_agent_ID_graph_factors_group -> ID_agent_ID_graph_ID_factors_group

ID_agent_ID_graph_group-> ID_agent_ID_graph_edges_group
  subgraph cluster_7 {
    style=rounded;
    bgcolor=ivory;
    color=green;
    ID_agent_ID_graph_ID_edges_group;
    label = "Number of edges per agent";
  }
ID_agent_ID_graph_edges_group -> ID_agent_ID_graph_ID_edges_group

  main_group -> agents_estimated_group
  subgraph cluster_1 {
    style=rounded;
    bgcolor=ivory;
    color=red;
    ID_agent_estimated_group;
    label = "Number of agents";
  }
  agents_estimated_group -> ID_agent_estimated_group
  }
```
