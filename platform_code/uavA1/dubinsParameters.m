% dubinsParameters
%   - Find Dubin's parameters between two configurations
%
% input is:
%   start_node  - [wn_s, we_s, wd_s, chi_s, 0, 0]
%   end_node    - [wn_e, wn_e, wd_e, chi_e, 0, 0]
%   R           - minimum turn radius
%
% output is:
%   dubinspath  - a matlab structure with the following fields
%       dubinspath.ps   - the start position in re^3
%       dubinspath.chis - the start course angle
%       dubinspath.pe   - the end position in re^3
%       dubinspath.chie - the end course angle
%       dubinspath.R    - turn radius
%       dubinspath.L    - length of the Dubins path
%       dubinspath.cs   - center of the start circle
%       dubinspath.lams - direction of the start circle
%       dubinspath.ce   - center of the end circle
%       dubinspath.lame - direction of the end circle
%       dubinspath.w1   - vector in re^3 defining half plane H1
%       dubinspath.q1   - unit vector in re^3 along straight line path
%       dubinspath.w2   - vector in re^3 defining position of half plane H2
%       dubinspath.w3   - vector in re^3 defining position of half plane H3
%       dubinspath.q3   - unit vector defining direction of half plane H3
% 

function dubinspath = dubinsParameters(start_node, end_node, R)

  ell = norm(start_node(1:2)-end_node(1:2));
  if ell<2*R,
      disp('The distance between nodes must be larger than 2R.');
      dubinspath = [];
  else
    
    ps   = start_node(1:3);
    chis = start_node(4);
    pe   = end_node(1:3);
    chie = end_node(4);

    crs = ps' + R*rotz(pi/2)*[cos(chis); sin(chis); 0];
    cls = ps' + R*rotz(-pi/2)*[cos(chis); sin(chis); 0];
    cre = pe' + R*rotz(pi/2)*[cos(chie); sin(chie); 0];
    cle = pe' + R*rotz(-pi/2)*[cos(chie); sin(chie); 0];
   
    % compute L1
    connecting_line = cre-crs;
    north_unit = [1; 0; 0];
%     theta = acos(dot(connecting_line,north_unit)/norm(connecting_line));
    theta = atan2(connecting_line(2), connecting_line(1));
    L1 = norm(crs-cre) + R*mod(2*pi + mod(theta-pi/2, 2*pi) - mod(chis-pi/2, 2*pi), 2*pi) ...
         + R*mod(2*pi + mod(chie-pi/2, 2*pi) - mod(theta-pi/2, 2*pi), 2*pi);
    % compute L2
    connecting_line = cle-crs;
    ell = norm(connecting_line);
%     theta = acos(dot(connecting_line,north_unit)/norm(connecting_line));
    theta = atan2(connecting_line(2), connecting_line(1));
    theta2 = theta - pi/2 + asin(2*R/ell);
    if isreal(theta2)==0, 
      L2 = 9999; 
    else
      L2 = sqrt(ell^2-4*R^2) + R*mod(2*pi + mod(theta2, 2*pi) - mod(chis-pi/2, 2*pi), 2*pi) ...
         + R*mod(2*pi + mod(theta2+pi, 2*pi) - mod(chie+pi/2, 2*pi), 2*pi);
    end
    % compute L3
    connecting_line = cre-cls;
    ell = norm(connecting_line);
%     theta = acos(dot(connecting_line,north_unit)/norm(connecting_line));
    theta = atan2(connecting_line(2), connecting_line(1));
    theta2 = acos(2*R/ell);
    if isreal(theta2)==0,
      L3 = 9999;
    else
      L3 = sqrt(ell^2-4*R^2) + R*mod(2*pi + mod(chis+pi/2, 2*pi) - mod(theta+theta2, 2*pi), 2*pi) ...
         + R*mod(2*pi + mod(chie-pi/2, 2*pi) - mod(theta+theta2-pi, 2*pi), 2*pi);
    end
    % compute L4
    connecting_line = cle-cls;
%     theta = acos(dot(connecting_line,north_unit)/norm(connecting_line));
    theta = atan2(connecting_line(2), connecting_line(1));
    L4 = norm(cls-cle) + R*mod(2*pi + mod(chis+pi/2, 2*pi) - mod(theta+pi/2, 2*pi), 2*pi) ...
         + R*mod(2*pi + mod(theta+pi/2, 2*pi) - mod(chie+pi/2, 2*pi), 2*pi);
    % L is the minimum distance
    [L,idx] = min([L1,L2,L3,L4]);
    e1 = [1; 0; 0];     % north unit vector
    switch(idx),
        case 1,
            cs = crs;
            lams = 1;
            ce = cre;
            lame = 1;
            q1 = (ce-cs)/norm(ce-cs);
            w1 = cs + R*rotz(-pi/2)*q1;
            w2 = ce + R*rotz(-pi/2)*q1;
        case 2,   
            cs = crs;
            lams = 1;
            ce = cle;
            lame = -1;
            ell = norm(ce-cs);
            connecting_line = ce - cs;
            theta = atan2(connecting_line(2), connecting_line(1));
%             theta = acos(dot(ce-cs,e1)/ell);
            theta2 = theta - pi/2 + asin(2*R/ell);
            q1 = rotz(theta2+pi/2)*e1;
            w1 = cs + R*rotz(theta2)*e1;
            w2 = ce + R*rotz(theta2+pi)*e1;
        case 3,
            cs = cls;
            lams = -1;
            ce = cre;
            lame = 1;
            ell = norm(ce-cs);
            connecting_line = ce - cs;
%             theta = acos(dot(ce-cs,e1)/ell);
            theta = atan2(connecting_line(2), connecting_line(1));
            theta2 = acos(2*R/ell);
            q1 = rotz(theta+theta2-pi/2)*e1;
            w1 = cs + R*rotz(theta+theta2)*e1;
            w2 = ce + R*rotz(theta+theta2-pi)*e1;
         case 4,
            cs = cls;
            lams = -1;
            ce = cle;
            lame = -1;
            q1 = (ce-cs)/norm(ce-cs);
            w1 = cs + R*rotz(pi/2)*q1;
            w2 = ce + R*rotz(pi/2)*q1;
    end
    w3 = pe';
    q3 = rotz(chie)*e1;
    
    % assign path variables
    dubinspath.ps   = ps;
    dubinspath.chis = chis;
    dubinspath.pe   = pe;
    dubinspath.chie = chie;
    dubinspath.R    = R;
    dubinspath.L    = L;
    dubinspath.cs   = cs;
    dubinspath.lams = lams;
    dubinspath.ce   = ce;
    dubinspath.lame = lame;
    dubinspath.w1   = w1;
    dubinspath.q1   = q1;
    dubinspath.w2   = w2;
    dubinspath.w3   = w3;
    dubinspath.q3   = q3;
  end
end

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%% rotz(theta)
%%   rotation matrix about the z axis.
function R = rotz(theta)
    R = [...
        cos(theta), -sin(theta), 0;...
        sin(theta), cos(theta), 0;...
        0, 0, 1;...
        ];
end
