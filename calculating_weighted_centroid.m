% Calculates the weighted centroids of a given area, the area has to be
% supplied as a matrix M containing weight at each point i,j. 
[X,Y] = meshgrid(0:.1:10);
D=sqrt((X.*X)+(Y.*Y)); %distance matrix
% M=ones(size(X));
M=gauss_multiple(X, Y, [1 1],[6 6;6 6],[1 1])
% M=gaussC(X,Y,0.1,[3 3]); %weight matrix
surf(X,Y,M);
hold on;
cent_x=sum(M.*X,'all')/sum(M,'all')
cent_y=sum(M.*Y,'all')/sum(M,'all')
plot(cent_x,cent_y,'or','linewidth',2);

function D = distance_mat(N)
[X,Y] = meshgrid(0:.1:N);
D=sqrt((X.*X)+(Y.*Y));
end

function val = gaussC(x, y, sigma, center)
xc = center(1);
yc = center(2);
exponent = ((x-xc).^2 + (y-yc).^2)./(2*sigma);
val       = (exp(-exponent));
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