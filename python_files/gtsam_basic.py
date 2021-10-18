#!/bin/python3

%% Initialize graph, initial estimate, and odometry noise
datafile = findExampleDataFile('test.json');
model = noiseModel.Diagonal.Sigmas([0.05; 0.05; 5*pi/180]);
[graph,initial] = load2D(datafile, model);

%% Add a Gaussian prior on pose x_0
priorMean = Pose2(0, 0, 0);
priorNoise = noiseModel.Diagonal.Sigmas([0.01; 0.01; 0.01]);
graph.add(PriorFactorPose2(0, priorMean, priorNoise));

%% Optimize using Levenberg-Marquardt optimization and get marginals
optimizer = LevenbergMarquardtOptimizer(graph, initial);
result = optimizer.optimizeSafely;
marginals = Marginals(graph, result);

%% Add a Gaussian prior on pose x_0
priorMean = Pose2(2, 0, 0);
priorNoise = noiseModel.Diagonal.Sigmas([0.01; 0.01; 0.01]);
graph.add(PriorFactorPose2(0, priorMean, priorNoise));

%% Optimize using Levenberg-Marquardt optimization and get marginals
optimizer = LevenbergMarquardtOptimizer(graph, initial);
result = optimizer.optimizeSafely;
marginals = Marginals(graph, result);

%% Add a Gaussian prior on pose x_0
priorMean = Pose2(0, 4, 0);
priorNoise = noiseModel.Diagonal.Sigmas([0.01; 0.01; 0.01]);
graph.add(PriorFactorPose2(0, priorMean, priorNoise));

%% Optimize using Levenberg-Marquardt optimization and get marginals
optimizer = LevenbergMarquardtOptimizer(graph, initial);
result = optimizer.optimizeSafely;
marginals = Marginals(graph, result);
