%use this for vector based control
function unicycle_test1
[t,q] = ode45( @unicycle, 0:0.1:10, [0 0 0] )
plot( q(:,1), q(:,2) )


function dq = unicycle( t, q )
x = q(1);
y = q(2);
theta = q(3);
finalpos=[-2 3];
% d=sqrt((finalpos(1)-x)^2+(finalpos(2)-y)^2);
v= (finalpos(1)-x)*cos(theta)+(finalpos(2)-y)*sin(theta);
% v=1
w= -(finalpos(1)-x)*sin(theta)+(finalpos(2)-y)*cos(theta);


dq = [ v*cos(theta) ; ...
       v*sin(theta) ; ...
       w ];
