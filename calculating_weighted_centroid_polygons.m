[X,Y] = meshgrid(0:.1:10);
Xn=X(:);
Yn=Y(:);
xv=[3 3 9 7]'; %vertices of polygon
yv=[3 9 9 1]'; %vertices of polygon
xv=[xv;xv(1)]; %vertices of polygon closed back
yv=[yv;yv(1)]; %vertices of polygon closed back
in=inpolygon(Xn,Yn,xv,yv); %getting points that are inside the given polygon
in=reshape(in,size(X));
% surf(X,Y,in)
M=gauss_multiple(X, Y, [1 1],[3 3;6 6],[1 0.5]);
surf(X,Y,M.*in)
hold on
cent_x=sum(M.*X.*in,'all')/sum(M.*in,'all')
cent_y=sum(M.*Y.*in,'all')/sum(M.*in,'all')
plot(cent_x,cent_y,'or','linewidth',2)


function [Cx,Cy] = WtPolyCentroid(xv,yv,M)

end 

function val = gauss_multiple(x, y, sigma, centers,w)
n=size(centers);
cx=centers(:,1);
cy=centers(:,2);
val=zeros(size(x));
for i=1:n(1)
    xc = cx(i);
    yc = cy(i);
    exponent = ((x-xc).^2 + (y-yc).^2)./(2*sigma(i));
    val       =val+w(i)*(exp(-exponent));
end
end