function input = M5_CreateStopScenario

input.ego.position = [0 0];
input.ego.velocity = [5 0];
input.ego.heading = 0;

input.objects.class = "pedestrian";
input.objects.position = [0 3];
input.objects.velocity = [0 0];
input.objects.predictedTrajectory = [0 3;0 3;0 3];

end