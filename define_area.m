% f = @(X,Y)  ((2*pi.*Y).^2)/2  %cos(2*pi.*X).*(1+3.*Y) +
% fsurf(f)
rng default;
x = rand([1 10]);
y = rand([1 10]);
voronoi(x,y)
axis equal