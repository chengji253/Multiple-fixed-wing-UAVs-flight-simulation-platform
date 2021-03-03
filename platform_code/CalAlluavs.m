close all;clear;
addpath 'data'
addpath 'uavA1'
% addpath 'uavShow'

%----------------
uavW = 1;

save('uavW.mat','uavW');
sim('New_mavsim_chap12');

ii = 1;
eval(['x' num2str(ii) '= x;'])
eval(['path' num2str(ii) '= path;'])
eval(['waypoints' num2str(ii) '= waypoints;'])

save('x1.mat','x1','path1','waypoints1');
%----------------
clear;
uavW = 2;
uavi = uavW;
save('uavW.mat','uavW');
sim('New_mavsim_chap12');

ii = 2;
eval(['x' num2str(ii) '= x;'])
eval(['path' num2str(ii) '= path;'])
eval(['waypoints' num2str(ii) '= waypoints;'])
save('x2.mat','x2','path2','waypoints2');
%----------------
clear;
uavW = 3;
save('uavW.mat','uavW');
sim('New_mavsim_chap12');

ii = 3;
eval(['x' num2str(ii) '= x;'])
eval(['path' num2str(ii) '= path;'])
eval(['waypoints' num2str(ii) '= waypoints;'])
save('x3.mat','x3','path3','waypoints3');
%----------------
clear;
uavW = 4;

save('uavW.mat','uavW');
sim('New_mavsim_chap12');

ii = 4;
eval(['x' num2str(ii) '= x;'])
eval(['path' num2str(ii) '= path;'])
eval(['waypoints' num2str(ii) '= waypoints;'])
save('x4.mat','x4','path4','waypoints4');
%----------------
clear;
uavW = 5;
save('uavW.mat','uavW');
sim('New_mavsim_chap12');

ii = 5;
eval(['x' num2str(ii) '= x;'])
eval(['path' num2str(ii) '= path;'])
eval(['waypoints' num2str(ii) '= waypoints;'])
save('x5.mat','x5','path5','waypoints5');
%----------------





