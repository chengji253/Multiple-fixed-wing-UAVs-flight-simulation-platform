# Multiple fixed-wing UAVs flight simulation platform

## Introduction

A multiple fixed-wing UAVs flight simulation platform built by matlab and simulink.

The example given here has 5 UAVs, but of course you can expand it to 10, 20 or even more if you are willing to take the time.

Input: The path of all uavs 

Output: 13 state quantities per drone per moment
    
    pn1            % inertial Northposition     
    pe1            % inertial East position
    pd1            % inertial Down position
    u1             % body frame velocities
    v1              
    w1            
    phi1           % roll angle         
    theta1         % pitch angle     
    psi1           % yaw angle     
    p1             % roll rate
    q1             % pitch rate     
    r1             % yaw rate    
    t1             % time

![avatar](picture/fixobs.gif)
![avatar](picture/1.gif)
---



##  How to use 

The simulation platform can be divided into two parts, one is the calculation part 'uavA1' and the other is the display part 'uavShow'. 

Just run the main.m file directly.

In fact, you can also synchronize the calculation and display, real-time calculation and then display. But personally, I think this will affect the smoothness of the display.  The more aircraft the greater the impact will be.

###  Calculation part

The state of each aircraft is calculated in turn over time and will be stored in the x1.mat file (x1 can be x2,x3.... which indicates the number of aircraft).

- CalAlluavs.m 

![avatar](picture/1.png)

###  Show part

- ShowAlluavs.m 

The data of each aircraft is stored in x, path, waypoint data. Using all the data, the show part could work.


##  How to read path files

The folder 'data' provides some path files for 5 aircraft that can be used.

If you want to calculate your own route data, you can follow these steps.
1. uavA1/getWpp.m     -> load '5jia.mat'
2. uavA1/para_chap1.m -> load '5jia.mat'

Find the corresponding code in the file and change the name of '5jia.mat' .

'getWpp.m'  Read the path

'para_chap1.m' reads the initial position of the aircraft

The simulink time needs to be adjusted according to the length of your path file, if your uav obviously did not run through your path, you need to adjust the time longer.

### Details about--- getWpp( ) function

```matlab
function [num_waypoints , wpp] = getWpp(P,uav)
    load '5jia3.mat'

    a1 = 2;
    a2 = 400;
    num_waypoints = a2/a1;       
    wpp = [];
    i1 = uav;

    for i = 1:a1:a2
        x = [];
        x = Xplot2(i,6*i1-5:6*i1-3);
        x(3) = -x(3);
        x = [x -9999 Xplot2(i,6*i1-2)];
        wpp = [wpp;x];    
    end
```


5jia3.mat: The file which stores  the states of 5 aircraft

400x30

400:Step length 

30: 5x6  6 state quantities for 5 aircraft

positions in three directions：x y z 

velocity :v 

Two angles of the velocity : theta  phi (velocity has no Rolling angle )_

14-20 line:dealing with the file 5jia3.mat 

x = Xplot2(i,6*i1-5:6*i1-3): 6*i1-5:6*i1-3 to get the positions x,y,z

 6*i1-2 to get the velocity

Now I have 400 steps of states to pass, and I don't want to pass each of them. So I'll pass every other one(1 , 3, 5... ), which is a1=2. Every two passes a1=3(1, 4, 7...). 

If you want to read your own path file, you can follow the format of my "5jia3.mat" file to generate it, or you can set up your own path file.

As you can see, I just passed the position and velocity, not the angle of the velocity. So the subsequent flight control is only tracking the position point according to the reference speed when tracking.




##  How to increase the number of uavs

 How to increase the aircraft is actually very easy but a little bit of boring. You need to add some code and change the corresponding numbers. The steps are as follows.
 
## The steps are as follows:

### 1. main.m
First of all, in the 'main.m' file, you can see that the code statements for each aircraft are obvious, add the corresponding sentences. 


```matlab
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

```

### uavShow/drawEnvironments5.m

Add the sentence of uavShow/drawEnvironments5.m. The sentence here looks complicated, but you don't have to figure out what it means. Just add it mechanically and change the numbers. 
If you look at the file uavShow/drawEnvironments5.m, you'll see what I'm talking about.

```matlab
%----------------1-----------------------------------
    NN = 0;
    pn1       = uu(1+NN);       % inertial North position     
    pe1       = uu(2+NN);       % inertial East position
    pd1       = uu(3+NN);       % inertial Down position
    u1        = uu(4+NN);       % body frame velocities
    v1        = uu(5+NN);       
    w1        = uu(6+NN);       
    phi1      = uu(7+NN);       % roll angle         
    theta1    = uu(8+NN);       % pitch angle     
    psi1      = uu(9+NN);       % yaw angle     
    p1        = uu(10+NN);      % roll rate
    q1        = uu(11+NN);      % pitch rate     
    r1        = uu(12+NN);      % yaw rate    
    t1        = uu(13+NN);      % time
    
    NN = NN + 13;
    path1     = uu(1+NN:13+NN); 
    NN = NN + 13;
    num_waypoints1 = uu(1+NN);
    waypoints1     = reshape(uu(2+NN:5*num_waypoints1+1+NN),5,num_waypoints1)'; 

    % define persistent variables 
    persistent aircraft_handle1;  % figure handle for MAV
    persistent path_handle1;      % handle for straight-line or orbit path
    persistent waypoint_handle1;  % handle for waypoints
    persistent Faces1
    persistent Vertices1
    persistent facecolors1

```
---

```matlab
[Vertices1,Faces1,facecolors1] = defineAircraftBody(scale);                              
        aircraft_handle1 = drawBody(Vertices1,Faces1,facecolors1,...
                                   pn1,pe1,pd1,phi1,theta1,psi1,...
                                   [], 'normal');
        hold on
        waypoint_handle1 = drawWaypoints(waypoints1, P.R_min, [], 'normal');
        path_handle1 = drawPath(path1, S, [], 'normal');


```

---

```matlab
drawBody(Vertices1,Faces1,facecolors1,...
                     pn1,pe1,pd1,phi1,theta1,psi1,...
                     aircraft_handle1);
        drawWaypoints(waypoints1, P.R_min, waypoint_handle1);
        drawPath(path1, S, path_handle1);
```

### uavShow/mavsim_show.slx

Open the uavShow/mavsim_show.slx file, simply add a few boxes and then just connect them.

For example, if you want to add the sixth uav, add four boxes: x6, time, path6, waypoints6, and then line them up down behind.

![avatar](picture/3.png)

--------
##  How to add obstacles

You can add some obstacles to test your collision avoidance algorithm.

In the file 'drawEnvironment1.m', the  "buildingVertFace" is the main function to build your map. Just add some parameters to it. Then you can get your own obstacles map.

```matlab
function drawMap(map)%,path,smoothedPath,tree,R_min)
  
 
  % draw buildings 
  V = [];
  F = [];
  patchcolors = [];
  count = 0;
  for i=1:map.NumBlocks,
      for j=1:map.NumBlocks,
        [Vtemp,Ftemp,patchcolorstemp] = buildingVertFace(map.buildings_n(i),...
            map.buildings_e(j),map.BuildingWidth,map.heights(j,i));
        V = [V; Vtemp];
        Ftemp = Ftemp + count;
        F = [F; Ftemp];
        count = count + 8;
        patchcolors = [patchcolors;patchcolorstemp];
      end
  end
  
  patch('Vertices', V, 'Faces', F,...
                 'FaceVertexCData',patchcolors,...
                 'FaceColor','flat');
 

end

```


## Acknowledgment

Randal's "Small Unmanned Aircraft Theory and Practice".

![avatar](picture/small.png)
