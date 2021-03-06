close all;
clc;
clear all;
t_series=[1:0.01:100];
Target_coordinates=[0.00 ;0.00]%[t_series,ones(size(t_series))*0,ones(size(t_series))*0];
Current_coordinates=[0.001 ;0.001];
init_heading=0;
for i=1:10
%     open('unicycle_simulink_const_input.slx');
    sim('unicycle_simulink_const_input.slx');
    a=ans.path_coordinates.data;
    b=ans.heading.data;
    init_heading=b(end);
%     figure;
    plot(a(:,1),a(:,2),'color','b');
    hold on;
    plot(Target_coordinates(1),Target_coordinates(2),'+','linewidth',2,'color','r');
%     text(Target_coordinates(1),Target_coordinates(2),num2str(i));
    plot(Current_coordinates(1),Current_coordinates(2),'o','linewidth',2,'color','g');
    Current_coordinates=Target_coordinates;
    Target_coordinates=[round(rand(1)*10);round(rand(1)*10) ];
end