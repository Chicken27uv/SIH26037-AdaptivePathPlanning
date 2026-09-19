function nextState = M7_VehicleModel(state,steering,speedCommand,dt)

wheelbase = 2.7;
speedResponse = 2.0;

x = state(1);
y = state(2);
heading = state(3);
speed = state(4);

acceleration = speedResponse * ...
    (speedCommand - speed);

speed = speed + acceleration * dt;

speed = max(0,speed);

x = x + speed * cos(heading) * dt;
y = y + speed * sin(heading) * dt;
heading = heading + ...
    speed / wheelbase * tan(steering) * dt;

heading = atan2(sin(heading),cos(heading));

nextState = [x y heading speed];

end