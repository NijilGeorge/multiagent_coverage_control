
kinematicModel = unicycleKinematics;
initialState = [0 0 0];
finalState = [2 3 pi/2];
tspan=0:0.05:1;
initialinput=[0 0 0]
inputs=;
[t,y] = ode45(@(t,y)derivative(kinematicModel,y,inputs),tspan,initialState);
figure;
plot(y(:,1),y(:,2));