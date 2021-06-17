function dq = unicycle( t, q )
x = q(1);
y = q(2);
theta = q(3);
v = 1;
w = 1;
dq = [ v*cos(theta) ; ...
       v*sin(theta) ; ...
       w ];
function unicycle_test
[t,q] = ode45( @unicycle, 0:0.01:10, [5 4 pi/3] )
plot( q(:,1), q(:,2) )