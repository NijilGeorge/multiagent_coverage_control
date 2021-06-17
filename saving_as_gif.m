function returning=savegif(figure,filename,n)
h = figure;
axis tight manual % this ensures that getframe() returns a consistent size
%filename = '/home/nijil/Documents/Nijil/Matlab_simulations/testAnimated.gif';
frame = getframe(h); 
im = frame2im(frame); 
[imind,cm] = rgb2ind(im,256); 
% Write to the GIF File 
if n == 1 
  imwrite(imind,cm,filename,'gif', 'Loopcount',inf); 
else 
  imwrite(imind,cm,filename,'gif','WriteMode','append'); 
end
returning=1;
end