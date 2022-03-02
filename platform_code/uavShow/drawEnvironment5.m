function drawEnvironment1(uu,P)

    % process inputs to function
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
  %----------------2-----------------------------------
    uu(1:5*num_waypoints1+1+NN) = [];
      while(uu(1)== -9999)
          uu(1) = [];
      end
    NN = 0;
    pn2       = uu(1+NN);       % inertial North position     
    pe2       = uu(2+NN);       % inertial East position
    pd2       = uu(3+NN);       % inertial Down position
    u2        = uu(4+NN);       % body frame velocities
    v2        = uu(5+NN);       
    w2        = uu(6+NN);       
    phi2      = uu(7+NN);       % roll angle         
    theta2    = uu(8+NN);       % pitch angle     
    psi2      = uu(9+NN);       % yaw angle     
    p2        = uu(10+NN);      % roll rate
    q2        = uu(11+NN);      % pitch rate     
    r2        = uu(12+NN);      % yaw rate    
    t2        = uu(13+NN);      % time
    
    NN = NN + 13;
    path2     = uu(1+NN:13+NN); 
    NN = NN + 13;
    num_waypoints2 = uu(1+NN);
    waypoints2     = reshape(uu(2+NN:5*num_waypoints2+1+NN),5,num_waypoints2)'; 

    % define persistent variables 
    persistent aircraft_handle2;  % figure handle for MAV
    persistent path_handle2;      % handle for straight-line or orbit path
    persistent waypoint_handle2;  % handle for waypoints
    persistent Faces2
    persistent Vertices2
    persistent facecolors2

%-----------------3------------------------------------------
uu(1:5*num_waypoints2+1+NN) = [];
      while(uu(1)== -9999)
          uu(1) = [];
      end
    NN = 0;
    pn3       = uu(1+NN);       % inertial North position     
    pe3       = uu(2+NN);       % inertial East position
    pd3       = uu(3+NN);       % inertial Down position
    u3        = uu(4+NN);       % body frame velocities
    v3        = uu(5+NN);       
    w3        = uu(6+NN);       
    phi3      = uu(7+NN);       % roll angle         
    theta3    = uu(8+NN);       % pitch angle     
    psi3      = uu(9+NN);       % yaw angle     
    p3        = uu(10+NN);      % roll rate
    q3        = uu(11+NN);      % pitch rate     
    r3        = uu(12+NN);      % yaw rate    
    t3        = uu(13+NN);      % time
    
    NN = NN + 13;
    path3     = uu(1+NN:13+NN); 
    NN = NN + 13;
    num_waypoints3 = uu(1+NN);
    waypoints3     = reshape(uu(2+NN:5*num_waypoints3+1+NN),5,num_waypoints3)'; 

    % define persistent variables 
    persistent aircraft_handle3;  % figure handle for MAV
    persistent path_handle3;      % handle for straight-line or orbit path
    persistent waypoint_handle3;  % handle for waypoints
    persistent Faces3
    persistent Vertices3
    persistent facecolors3

%------------------4---------------------------------------------------------
uu(1:5*num_waypoints3+1+NN) = [];
      while(uu(1)== -9999)
          uu(1) = [];
      end
    NN = 0;
    pn4       = uu(1+NN);       % inertial North position     
    pe4       = uu(2+NN);       % inertial East position
    pd4       = uu(3+NN);       % inertial Down position
    u4        = uu(4+NN);       % body frame velocities
    v4        = uu(5+NN);       
    w4        = uu(6+NN);       
    phi4      = uu(7+NN);       % roll angle         
    theta4    = uu(8+NN);       % pitch angle     
    psi4      = uu(9+NN);       % yaw angle     
    p4        = uu(10+NN);      % roll rate
    q4        = uu(11+NN);      % pitch rate     
    r4        = uu(12+NN);      % yaw rate    
    t4        = uu(13+NN);      % time
    
    NN = NN + 13;
    path4     = uu(1+NN:13+NN); 
    NN = NN + 13;
    num_waypoints4 = uu(1+NN);
    waypoints4     = reshape(uu(2+NN:5*num_waypoints4+1+NN),5,num_waypoints4)'; 

    % define persistent variables 
    persistent aircraft_handle4;  % figure handle for MAV
    persistent path_handle4;      % handle for straight-line or orbit path
    persistent waypoint_handle4;  % handle for waypoints
    persistent Faces4
    persistent Vertices4
    persistent facecolors4
%------------------5---------------------------------------------------------
uu(1:5*num_waypoints4+1+NN) = [];
      while(uu(1)== -9999)
          uu(1) = [];
      end
    NN = 0;
    pn5       = uu(1+NN);       % inertial North position     
    pe5       = uu(2+NN);       % inertial East position
    pd5       = uu(3+NN);       % inertial Down position
    u5        = uu(4+NN);       % body frame velocities
    v5        = uu(5+NN);       
    w5        = uu(6+NN);       
    phi5      = uu(7+NN);       % roll angle         
    theta5    = uu(8+NN);       % pitch angle     
    psi5      = uu(9+NN);       % yaw angle     
    p5        = uu(10+NN);      % roll rate
    q5        = uu(11+NN);      % pitch rate     
    r5        = uu(12+NN);      % yaw rate    
    t5        = uu(13+NN);      % time
    
    NN = NN + 13;
    path5     = uu(1+NN:13+NN); 
    NN = NN + 13;
    num_waypoints5 = uu(1+NN);
    waypoints5     = reshape(uu(2+NN:5*num_waypoints5+1+NN),5,num_waypoints5)'; 

    % define persistent variables 
    persistent aircraft_handle5;  % figure handle for MAV
    persistent path_handle5;      % handle for straight-line or orbit path
    persistent waypoint_handle5;  % handle for waypoints
    persistent Faces5
    persistent Vertices5
    persistent facecolors5
%---------------------------------------------------------------------------

    S = 4000; % plot size
    
    % first time function is called, initialize plot and persistent vars
    if t1==0

        figure(1), clf
        scale = 4;
%---------------------------------------------------------------------------
       [Vertices1,Faces1,facecolors1] = defineAircraftBody(scale);                              
        aircraft_handle1 = drawBody(Vertices1,Faces1,facecolors1,...
                                   pn1,pe1,pd1,phi1,theta1,psi1,...
                                   [], 'normal');
        hold on
        waypoint_handle1 = drawWaypoints(waypoints1, P.R_min, [], 'normal');
        path_handle1 = drawPath(path1, S, [], 'normal');


        [Vertices2,Faces2,facecolors2] = defineAircraftBody(scale);                              
        aircraft_handle2 = drawBody(Vertices2,Faces2,facecolors2,...
                                   pn2,pe2,pd2,phi2,theta2,psi2,...
                                   [], 'normal');
        hold on
        waypoint_handle2 = drawWaypoints(waypoints2, P.R_min, [], 'normal');
        path_handle2 = drawPath(path2, S, [], 'normal');

        [Vertices3,Faces3,facecolors3] = defineAircraftBody(scale);                              
        aircraft_handle3 = drawBody(Vertices3,Faces3,facecolors3,...
                                   pn3,pe3,pd3,phi3,theta3,psi3,...
                                   [], 'normal');
        hold on
        waypoint_handle3 = drawWaypoints(waypoints3, P.R_min, [], 'normal');
        path_handle3 = drawPath(path3, S, [], 'normal');


        [Vertices4,Faces4,facecolors4] = defineAircraftBody(scale);                              
        aircraft_handle4 = drawBody(Vertices4,Faces4,facecolors4,...
                                   pn4,pe4,pd4,phi4,theta4,psi4,...
                                   [], 'normal');
        hold on
        waypoint_handle4 = drawWaypoints(waypoints4, P.R_min, [], 'normal');
        path_handle4 = drawPath(path4, S, [], 'normal');

        [Vertices5,Faces5,facecolors5] = defineAircraftBody(scale);                              
        aircraft_handle5 = drawBody(Vertices5,Faces5,facecolors5,...
                                   pn5,pe5,pd5,phi5,theta5,psi5,...
                                   [], 'normal');
        hold on
        waypoint_handle5 = drawWaypoints(waypoints5, P.R_min, [], 'normal');
        path_handle5 = drawPath(path5, S, [], 'normal');
%---------------------------------------------------------------------------



      

        drawMap(P.map);
        
        title('UAV')
        xlabel('East')
        ylabel('North')
        zlabel('-Down')
        axis([-S/5,S,-S/5,S,0,3*P.map.MaxHeight]);
        view(-40,70)  % set the view angle for figure
        grid on
              
    % at every other time step, redraw MAV
    else 
%---------------------------------------------------------------------------
        drawBody(Vertices1,Faces1,facecolors1,...
                     pn1,pe1,pd1,phi1,theta1,psi1,...
                     aircraft_handle1);
        drawWaypoints(waypoints1, P.R_min, waypoint_handle1);
        drawPath(path1, S, path_handle1);

        drawBody(Vertices2,Faces2,facecolors2,...
                  pn2,pe2,pd2,phi2,theta2,psi2,...
                  aircraft_handle2);
        drawWaypoints(waypoints2, P.R_min, waypoint_handle2);
        drawPath(path2, S, path_handle2);

        drawBody(Vertices3,Faces3,facecolors3,...
                  pn3,pe3,pd3,phi3,theta3,psi3,...
                  aircraft_handle3);
        drawWaypoints(waypoints3, P.R_min, waypoint_handle3);
        drawPath(path3, S, path_handle3);

        drawBody(Vertices4,Faces4,facecolors4,...
                  pn4,pe4,pd4,phi4,theta4,psi4,...
                  aircraft_handle4);
        drawWaypoints(waypoints4, P.R_min, waypoint_handle4);
        drawPath(path4, S, path_handle4);

        drawBody(Vertices5,Faces5,facecolors5,...
                  pn5,pe5,pd5,phi5,theta5,psi5,...
                  aircraft_handle5);
        drawWaypoints(waypoints5, P.R_min, waypoint_handle5);
        drawPath(path5, S, path_handle5);
%---------------------------------------------------------------------------

    end
end







%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
function handle = drawBody(V,F,colors,...
                               pn, pe, pd, phi, theta, psi,...
                               handle, mode)
  V = rotate(V', phi, theta, psi)';  % rotate rigid body  
  V = translate(V', pn, pe, pd)';  % translate after rotation

  % transform vertices from NED to XYZ (for matlab rendering)
  R = [...
      0, 1, 0;...
      1, 0, 0;...
      0, 0, -1;...
      ];
  V = V*R;

  if isempty(handle),
    handle = patch('Vertices', V, 'Faces', F,...
                 'FaceVertexCData',colors,...
                 'FaceColor','flat',...
                 'EraseMode', mode);
  else
    set(handle,'Vertices',V,'Faces',F);
    drawnow
  end
  
end 

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
function handle = drawPath(path, S, handle, mode)
    flag = path(1); 
    r    = [path(3); path(4); path(5)];
    q    = [path(6); path(7); path(8)];
    c    = [path(9); path(10); path(11)];
    rho  = path(12);
    lam  = path(13);

    switch flag,
        case 1,
            XX = [r(1), r(1)+S*q(1)];
            YY = [r(2), r(2)+S*q(2)];
            ZZ = [r(3), r(3)+S*q(3)];
        case 2,
            N = 100;
            th = [0:2*pi/N:2*pi];
            XX = c(1) + rho*cos(th);
            YY = c(2) + rho*sin(th);
            ZZ = c(3)*ones(size(th));
    end
    
    if isempty(handle),
        handle = plot3(YY,XX,-ZZ,'r', 'EraseMode', mode);
    else
        set(handle,'XData', YY, 'YData', XX, 'ZData', -ZZ);
        drawnow
    end
end 

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
function handle = drawWaypoints(waypoints, R_min, handle, mode)

    if waypoints(1,4)==-9999, % check to see if Dubins paths
        XX = [waypoints(:,1)];
        YY = [waypoints(:,2)];
        ZZ = [waypoints(:,3)];
    else
        XX = [];
        YY = [];
        for i=2:size(waypoints,1),
            dubinspath = dubinsParameters(waypoints(i-1,:),waypoints(i,:),R_min);
            [tmpX,tmpY] = pointsAlongDubinsPath(dubinspath,0.1);
            XX = [XX; tmpX];
            YY = [YY; tmpY];     
        end
        ZZ = waypoints(i,3)*ones(size(XX));
    end
    
    if isempty(handle),
        handle = plot3(YY,XX,-ZZ,'b', 'EraseMode', mode);
    else
        set(handle,'XData', YY, 'YData', XX, 'ZData', -ZZ);
        drawnow
    end
end 


%%%%%%%%%%%%%%%%%%%%%%%
function XYZ=rotate(XYZ,phi,theta,psi)
  % define rotation matrix
  R_roll = [...
          1, 0, 0;...
          0, cos(phi), -sin(phi);...
          0, sin(phi), cos(phi)];
  R_pitch = [...
          cos(theta), 0, sin(theta);...
          0, 1, 0;...
          -sin(theta), 0, cos(theta)];
  R_yaw = [...
          cos(psi), -sin(psi), 0;...
          sin(psi), cos(psi), 0;...
          0, 0, 1];

  % rotate vertices
  XYZ = R_yaw*R_pitch*R_roll*XYZ;
  
end

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% translate vertices by pn, pe, pd
function XYZ = translate(XYZ,pn,pe,pd)

  XYZ = XYZ + repmat([pn;pe;pd],1,size(XYZ,2));
  
end


%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% drawMap
%   plot obstacles and path
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

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%% buildingVertFace(x,y,width,height)
%%   define patches for a building located at (x,y)
function [V,F,patchcolors] = buildingVertFace(n,e,width,height)
 
  % vertices of the building
  V = [...
        e+width/2, n+width/2, 0;...
        e+width/2, n-width/2, 0;...
        e-width/2, n-width/2, 0;...
        e-width/2, n+width/2, 0;...
        e+width/2, n+width/2, height;...
        e+width/2, n-width/2, height;...
        e-width/2, n-width/2, height;...
        e-width/2, n+width/2, height;...
        ];    
  % define faces of fuselage
  F = [...
        1, 4, 8, 5;... % North Side
        1, 2, 6, 5;... % East Side
        2, 3, 7, 6;... % South Side
        3, 4, 8, 7;... % West Side
        5, 6, 7, 8;... % Top
        ];   

  myred = [1, 0, 0];
  mygreen = [0, 1, 0];
  myblue = [0, 0, 1];
  myyellow = [1,1,0];
  mymagenta   = [0, 1, 1];

  patchcolors = [...
    mygreen;... % North
    mygreen;... % East
    mygreen;... % South
    mygreen;... % West
    myyellow;...  % Top
    ];

end

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%% pointsAlongDubinsPath
%%   Find points along Dubin's path separted by Del (to be used in
%%   collision detection)
function [X,Y] = pointsAlongDubinsPath(dubinspath,Del)


  % points along start circle
  th1 = mod(atan2(dubinspath.ps(2)-dubinspath.cs(2),dubinspath.ps(1)-dubinspath.cs(1)),2*pi);
  th2 = mod(atan2(dubinspath.w1(2)-dubinspath.cs(2),dubinspath.w1(1)-dubinspath.cs(1)),2*pi);
  if dubinspath.lams>0,
      if th1>=th2,
        th = [th1:Del:2*pi,0:Del:th2];
      else
        th = [th1:Del:th2];
      end
  else
      if th1<=th2,
        th = [th1:-Del:0,2*pi:-Del:th2];
      else
        th = [th1:-Del:th2];
      end
  end
  X = [];
  Y = [];
  for i=1:length(th),
    X = [X; dubinspath.cs(1)+dubinspath.R*cos(th(i))]; 
    Y = [Y; dubinspath.cs(2)+dubinspath.R*sin(th(i))];
  end
  
  % points along straight line 
  sig = 0;
  while sig<=1,
      X = [X; (1-sig)*dubinspath.w1(1) + sig*dubinspath.w2(1)];
      Y = [Y; (1-sig)*dubinspath.w1(2) + sig*dubinspath.w2(2)];
      sig = sig + Del;
  end
    
  % points along end circle
  th2 = mod(atan2(dubinspath.pe(2)-dubinspath.ce(2),dubinspath.pe(1)-dubinspath.ce(1)),2*pi);
  th1 = mod(atan2(dubinspath.w2(2)-dubinspath.ce(2),dubinspath.w2(1)-dubinspath.ce(1)),2*pi);
  if dubinspath.lame>0,
      if th1>=th2,
        th = [th1:Del:2*pi,0:Del:th2];
      else
        th = [th1:Del:th2];
      end
  else
      if th1<=th2,
        th = [th1:-Del:0,2*pi:-Del:th2];
      else
        th = [th1:-Del:th2];
      end
  end
  for i=1:length(th),
    X = [X; dubinspath.ce(1)+dubinspath.R*cos(th(i))]; 
    Y = [Y; dubinspath.ce(2)+dubinspath.R*sin(th(i))];
  end
  
end

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% define aircraft vertices and faces
function [V,F,colors] = defineAircraftBody(scale)


% parameters for drawing aircraft
  % scale size
  fuse_l1    = 7;
  fuse_l2    = 4;
  fuse_l3    = 15;
  fuse_w     = 2;
  wing_l     = 6;
  wing_w     = 20;
  tail_l     = 3;
  tail_h     = 3;
  tailwing_w = 10;
  tailwing_l = 3;
  % colors
  red     = [1, 0, 0];
  green   = [0, 1, 0];
  blue    = [0, 0, 1];
  yellow  = [1,1,0];
  magenta = [0, 1, 1];
  

% define vertices and faces for aircraft
  V = [...
    fuse_l1,             0,             0;...        % point 1
    fuse_l2,            -fuse_w/2,     -fuse_w/2;... % point 2     
    fuse_l2,             fuse_w/2,     -fuse_w/2;... % point 3     
    fuse_l2,             fuse_w/2,      fuse_w/2;... % point 4
    fuse_l2,            -fuse_w/2,      fuse_w/2;... % point 5
   -fuse_l3,             0,             0;...        % point 6
    0,                   wing_w/2,      0;...        % point 7
   -wing_l,              wing_w/2,      0;...        % point 8
   -wing_l,             -wing_w/2,      0;...        % point 9
    0,                  -wing_w/2,      0;...        % point 10
   -fuse_l3+tailwing_l,  tailwing_w/2,  0;...        % point 11
   -fuse_l3,             tailwing_w/2,  0;...        % point 12
   -fuse_l3,            -tailwing_w/2,  0;...        % point 13
   -fuse_l3+tailwing_l, -tailwing_w/2,  0;...        % point 14
   -fuse_l3+tailwing_l,  0,             0;...        % point 15
   -fuse_l3+tailwing_l,  0,             -tail_h;...  % point 16
   -fuse_l3,             0,             -tail_h;...  % point 17
  ];
  
  F = [...
        1,  2,  3,  1;... % nose-top
        1,  3,  4,  1;... % nose-left
        1,  4,  5,  1;... % nose-bottom
        1,  5,  2,  1;... % nose-right
        2,  3,  6,  2;... % fuselage-top
        3,  6,  4,  3;... % fuselage-left
        4,  6,  5,  4;... % fuselage-bottom
        2,  5,  6,  2;... % fuselage-right
        7,  8,  9, 10;... % wing
       11, 12, 13, 14;... % tailwing
        6, 15, 17, 17;... % tail
        
  ];  
  
  colors = [...
        yellow;... % nose-top
        yellow;... % nose-left
        yellow;... % nose-bottom
        yellow;... % nose-right
        blue;... % fuselage-top
        blue;... % fuselage-left
        red;... % fuselage-bottom
        blue;... % fuselage-right
        green;... % wing
        green;... % tailwing
        blue;... % tail
    ];

  V = scale*V;   % rescale vertices

end
  