global X
global Y
global M
global cost_list
global sensing %changed inside cost_cell

sensing_list=zeros(size(M));
showPlot = true;
numIterations  = 100;
xrange = 10;  %region size
yrange = 10;
n = 15; %number of robots  (changing the number of robots is interesting)
[X,Y] = meshgrid(0:.1:max(xrange,yrange));
M =gauss_multiple(X, Y, [5 5],[3 3;9 9],[3 0]);
% M=ones(size(X));

cost_list=zeros(numIterations,1);
% Generate and Place  n stationary robots
Px = 0.01*mod(1:n,ceil(sqrt(n)))'*xrange; %start the robots in a small grid
Py = 0.01*floor((1:n)/sqrt(n))'*yrange;
crs = [ 0, 0;    
    0, yrange;
    xrange, yrange;
    xrange, 0];

% crs = [ 0, 0;    
%     0, yrange;
%     1/3*xrange, yrange;  % a world with a narrow passage
%     1/3*xrange, 1/4*yrange;
%     2/3*xrange, 1/4*yrange;
%     2/3*xrange, yrange;
%     xrange, yrange;
%     xrange, 0];

% surf(X,Y,ones(size(valid_reg)).*valid_reg)
valid_reg=valid_region(crs);
M=M.*valid_reg;
M=M/sum(M,'all');
    
for i = 1:numel(Px)  
    while ~inpolygon(Px(i),Py(i),crs(:,1),crs(:,2))% ensure robots are inside the boundary
        Px(i) = rand(1,1)*xrange; 
        Py(i) = rand(1,1)*yrange;
    end
end


[Px, Py]=lloydsAlgorithm(Px,Py, crs, numIterations, showPlot)
% plot(cost_list)
f4=figure(4);
surf(X,Y,M);
xlabel('x');
ylabel('y');
zlabel('Density function');
title('Density Function');
colorbar


function [Px, Py] = lloydsAlgorithm(Px,Py, crs, numIterations, showPlot)
% LLOYDSALGORITHM runs Lloyd's algorithm on the particles at xy positions 
% (Px,Py) within the boundary polygon crs for numIterations iterations
% showPlot = true will display the results graphically.  
% 
% Lloyd's algorithm starts with an initial distribution of samples or
% points and consists of repeatedly executing one relaxation step:
%   1.  The Voronoi diagram of all the points is computed.
%   2.  Each cell of the Voronoi diagram is integrated and the centroid is computed.
%   3.  Each point is then moved to the centroid of its Voronoi cell.
%
% Inspired by http://www.mathworks.com/matlabcentral/fileexchange/34428-voronoilimit
% Requires the Polybool function of the mapping toolbox to run.
%
% Run with no input to see example.  To initialize a square with 50 robots 
% in left middle, run:
%lloydsAlgorithm(0.01*rand(50,1),zeros(50,1)+1/2, [0,0;0,1;1,1;1,0], 200, true)
%
% Made by: Aaron Becker, atbecker@uh.edu
close all
format compact
global cost_list
global sensing %changed inside cost_cell
global M
global X
global Y
% initialize random generator in repeatable fashion
sd = 20;
rng(sd);
if nargin < 1   % demo mode
    showPlot = true;
    numIterations  = 200;
    xrange = 10;  %region size
    yrange = 10;
    n = 20; %number of robots  (changing the number of robots is interesting)

% Generate and Place  n stationary robots
    Px = 0.01*mod(1:n,ceil(sqrt(n)))'*xrange; %start the robots in a small grid
    Py = 0.01*floor((1:n)/sqrt(n))'*yrange;
    
%     Px = 0.1*rand(n,1)*xrange; % place n  robots randomly
%     Py = 0.1*rand(n,1)*yrange;
%     
%     crs = [ 0, 0;    
%         0, yrange;
%         1/3*xrange, yrange;  % a world with a narrow passage
%         1/3*xrange, 1/4*yrange;
%         2/3*xrange, 1/4*yrange;
%         2/3*xrange, yrange;
%         xrange, yrange;
%         xrange, 0];
    crs = [ 0, 0;    
        0, yrange;
        xrange, yrange;
        xrange, 0];
    
    for i = 1:numel(Px)  
        while ~inpolygon(Px(i),Py(i),crs(:,1),crs(:,2))% ensure robots are inside the boundary
            Px(i) = rand(1,1)*xrange; 
            Py(i) = rand(1,1)*yrange;
        end
    end
else
    xrange = max(crs(:,1));
    yrange = max(crs(:,2));
    n = numel(Px); %number of robots  
end


%%%%%%%%%%%%%%%%%%%%%%%% VISUALIZATION %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
if showPlot
    verCellHandle = zeros(n,1);
    cellColors = cool(n);
    for i = 1:numel(Px) % color according to
        verCellHandle(i)  = patch(Px(i),Py(i),cellColors(i,:)); % use color i  -- no robot assigned yet
        hold on
    end
    pathHandle = zeros(n,1);    
    %numHandle = zeros(n,1);    
    for i = 1:numel(Px) % color according to
        pathHandle(i)  = plot(Px(i),Py(i),'-','color',cellColors(i,:)*.8);
    %    numHandle(i) = text(Px(i),Py(i),num2str(i));
    end
    goalHandle = plot(Px,Py,'+','linewidth',2);
    currHandle = plot(Px,Py,'o','linewidth',2);
    titleHandle = title(['o = Robots, + = Goals, Iteration ', num2str(0)]);
end
%%%%%%%%%%%%%%%%%%%%%%%% END VISUALIZATION %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% Iteratively Apply LLYOD's Algorithm
for counter = 1:numIterations
    sensing=zeros(size(M));
    %[v,c]=VoronoiLimit(Px,Py, crs, false);
    [v,c]=VoronoiBounded(Px,Py, crs);
    
    if showPlot
        set(currHandle,'XData',Px,'YData',Py);%plot current position
        for i = 1:numel(Px) % color according to
            xD = [get(pathHandle(i),'XData'),Px(i)];
            yD = [get(pathHandle(i),'YData'),Py(i)];
            set(pathHandle(i),'XData',xD,'YData',yD);%plot path position
     %       set(numHandle(i),'Position',[ Px(i),Py(i)]);
        end 
    end
    costfn=0;
    for i = 1:numel(c) %calculate the centroid of each cell   
        [cx,cy] = WtPolyCentroid(v(c{i},1),v(c{i},2));
        costfn=costfn+cost_cell(v(c{i},1),v(c{i},2),cx,cy);
        cx = min(xrange,max(0, cx));
        cy = min(yrange,max(0, cy));
        if ~isnan(cx) && inpolygon(cx,cy,crs(:,1),crs(:,2))
            Px(i) = cx;  %don't update if goal is outside the polygon
            Py(i) = cy;
        end
    end
    cost_list(counter)=costfn;
    if showPlot
        
        f1=figure(1);
        for i = 1:numel(c) % update Voronoi cells
            set(verCellHandle(i), 'XData',v(c{i},1),'YData',v(c{i},2));
        end

        set(titleHandle,'string',['o = Robots, + = Goals, Iteration ', num2str(counter,'%3d')]);
        set(goalHandle,'XData',Px,'YData',Py);%plot goal position
        
        
        drawnow
        filename1 = '/home/nijil/Documents/Nijil/MTP/Report/Simultaion Figures/homogeneous/single_gaussian_15agents/path.gif';
        savegif(f1,filename1,counter);
        
        f2=figure(2);
        plot(cost_list);
        xlabel('Iteration');
        ylabel('Cost Function');
        title('Cost Function v/s Iteration');
        filename3 = '/home/nijil/Documents/Nijil/MTP/Report/Simultaion Figures/homogeneous/single_gaussian_15agents/costvsiter.gif';
        savegif(f2,filename3,counter);
        
        f3=figure(3);
        surf(X,Y,sensing,'EdgeColor', [0 0 0]);
        xlabel('x');
        ylabel('y');
        zlabel('Cost function');
        title('Cost Function');
        colorbar
        filename2 = '/home/nijil/Documents/Nijil/MTP/Report/Simultaion Figures/homogeneous/single_gaussian_15agents/cost_function.gif';
        savegif(f3,filename2,counter);
%         if mod(counter,50) ==0
%             pause
%             %pause(0.1)
%         end
    end
    %find cost function value here and store
end
end

function cost_i=cost_cell(Xa,Ya,cx,cy) 
    %cost incurred by the agent at position (cx,cy) with voronoi region
    %with vertices given by Xa,Ya
    global X;
    global Y;
    global M;
    global sensing
    Xn=X(:);
    Yn=Y(:);
    Xa = [Xa(2:end);Xa(1)];
    Ya = [Ya(2:end);Ya(1)];
    in=inpolygon(Xn,Yn,Xa,Ya); %getting points that are inside the given polygon
    in=reshape(in,size(M));
    costx=(X-ones(size(X)).*cx);
    costy=(Y-ones(size(Y)).*cy);
    cost_i=sum(((costx.*costx+costy.*costy).*M).*in,'all')/2;
    sensing=sensing+((costx.*costx+costy.*costy).*M).*in;
end

function [Cx,Cy] = WtPolyCentroid(Xa,Ya)
    global X
    global Y
    global M
    Xn=X(:);
    Yn=Y(:);
    Xa = [Xa(2:end);Xa(1)];
    Ya = [Ya(2:end);Ya(1)];
    in=inpolygon(Xn,Yn,Xa,Ya); %getting points that are inside the given polygon
    in=reshape(in,size(X));
    Cx=sum(M.*X.*in,'all')/sum(M.*in,'all');
    Cy=sum(M.*Y.*in,'all')/sum(M.*in,'all');
end

function val = gauss_multiple(x, y, sigma, centers,w)
    nc=size(centers);
    Cx=centers(:,1);
    Cy=centers(:,2);
    val=zeros(size(x));
    for ii=1:nc(1)
        xc = Cx(ii);
        yc = Cy(ii);
        exponent = ((x-xc).^2 + (y-yc).^2)./(2*sigma(ii));
        val       =val+w(ii)*(exp(-exponent));
    end
end

function [V,C]=VoronoiBounded(x,y, crs)
    % VORONOIBOUNDED computes the Voronoi cells about the points (x,y) inside
    % the bounding box (a polygon) crs.  If crs is not supplied, an
    % axis-aligned box containing (x,y) is used.

    bnd=[min(x) max(x) min(y) max(y)]; %data bounds
    if nargin < 3
        crs=double([bnd(1) bnd(4);bnd(2) bnd(4);bnd(2) bnd(3);bnd(1) bnd(3);bnd(1) bnd(4)]);
    end

    rgx = max(crs(:,1))-min(crs(:,1));
    rgy = max(crs(:,2))-min(crs(:,2));
    rg = max(rgx,rgy);
    midx = (max(crs(:,1))+min(crs(:,1)))/2;
    midy = (max(crs(:,2))+min(crs(:,2)))/2;

    % add 4 additional edges
    xA = [x; midx + [0;0;-5*rg;+5*rg]];
    yA = [y; midy + [-5*rg;+5*rg;0;0]];

    [vi,ci]=voronoin([xA,yA]);

    % remove the last 4 cells
    C = ci(1:end-4);
    V = vi;
    % use Polybool to crop the cells
    %Polybool for restriction of polygons to domain.

    for ij=1:length(C)
            % thanks to http://www.mathworks.com/matlabcentral/fileexchange/34428-voronoilimit
            % first convert the contour coordinate to clockwise order:
            [X2, Y2] = poly2cw(V(C{ij},1),V(C{ij},2));
            [xb, yb] = polybool('intersection',crs(:,1),crs(:,2),X2,Y2);
            ix=nan(1,length(xb));
            for il=1:length(xb)
                if any(V(:,1)==xb(il)) && any(V(:,2)==yb(il))
                    ix1=find(V(:,1)==xb(il));
                    ix2=find(V(:,2)==yb(il));
                    for ib=1:length(ix1)
                        if any(ix1(ib)==ix2)
                            ix(il)=ix1(ib);
                        end
                    end
                    if isnan(ix(il))==1
                        lv=length(V);
                        V(lv+1,1)=xb(il);
                        V(lv+1,2)=yb(il);
                        ix(il)=lv+1;
                    end
                else
                    lv=length(V);
                    V(lv+1,1)=xb(il);
                    V(lv+1,2)=yb(il);
                    ix(il)=lv+1;
                end
            end
            C{ij}=ix;
    end
end

function returning=savegif(figure,filename,n)
    h = figure;
    axis tight manual % this ensures that getframe() returns a consistent size
    %filename = '/home/nijil/Documents/Nijil/Matlab_simulations/testAnimated.gif';
    frame = getframe(h); 
    im = frame2im(frame); 
    [imind,cm] = rgb2ind(im,256); 
    % Write to the GIF File 
    if n == 1 
      imwrite(imind,cm,filename,'gif', 'Loopcount',inf,'DelayTime',0.5); 
    else 
      imwrite(imind,cm,filename,'gif','WriteMode','append','DelayTime',0.5); 
    end
    returning=1;
end

function vr=valid_region(crs)
    global X;
    global Y;
    Xn=X(:)
    Yn=Y(:)
    c=[crs;crs(1,:)];
    vr=inpolygon(Xn,Yn,c(:,1),c(:,2));
    vr=reshape(vr,size(X));
end