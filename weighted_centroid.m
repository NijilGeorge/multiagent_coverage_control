[X,Y] = meshgrid(0:.1:10);
[Cx,Cy] = meshgrid(1:1:9);
sigma=ones(size(Cx))*0.18;
% weights_gaussian=ones(size(Cx))
weights_gaussian=Cx+Cy;
% weights_gaussian=gaussC(Cx, Cy, 3, [5,5]);
size(weights_gaussian);
% Z = gauss_basis(X,Y,sigma,[Cx,Cy],weights_gaussian);
Z=gaussC(X,Y,3,[5,5])
surf(X,Y,Z);

function val = gauss_basis(x, y, sigma, centers,w)
n=size(centers);
cx=centers(:,1:n(1));
cy=centers(:,n(1)+1:n(2));
val=zeros(size(x));

for i=1:n(1)
    for j=1:n(1)
        xc = cx(i,j);
        yc = cy(i,j);
        exponent = ((x-xc).^2 + (y-yc).^2)./(2*sigma(i,j));
        val       =val+w(i,j)*(exp(-exponent));
    end
end
end

function val = gaussC(x, y, sigma, center)
xc = center(1);
yc = center(2);
exponent = ((x-xc).^2 + (y-yc).^2)./(2*sigma);
val       = (exp(-exponent));
end