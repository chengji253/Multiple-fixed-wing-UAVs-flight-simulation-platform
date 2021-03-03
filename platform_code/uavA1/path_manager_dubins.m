% path_manager_dubins 
%   - follow Dubins paths between waypoint configurations
%
% input is:
%   num_waypoints - number of waypoint configurations
%   waypoints    - an array of dimension 5 by P.size_waypoint_array.
%                - the first num_waypoints rows define waypoint
%                  configurations
%                - format for each waypoint configuration:
%                  [wn, we, wd, chi_d, Va_d]
%                  where the (wn, we, wd) is the NED position of the
%                  waypoint, chi_d is the desired course at the waypoint,
%                  and Va_d is the desired airspeed along the path. 
%
% output is:
%   flag - if flag==1, follow waypoint path
%          if flag==2, follow orbit
%   
%   Va^d - desired airspeed
%   r    - inertial position of start of waypoint path
%   q    - unit vector that defines inertial direction of waypoint path
%   c    - center of orbit
%   rho  - radius of orbit
%   lambda = direction of orbit (+1 for CW, -1 for CCW)
%
function out = path_manager_dubins(in,P,start_of_simulation)

  NN = 0;
  num_waypoints = in(1+NN);
  waypoints = reshape(in(2+NN:5*P.size_waypoint_array+1+NN),5,P.size_waypoint_array);
  NN = NN + 1 + 5*P.size_waypoint_array;
  pn        = in(1+NN);
  pe        = in(2+NN);
  h         = in(3+NN);
  % Va      = in(4+NN);
  % alpha   = in(5+NN);
  % beta    = in(6+NN);
  % phi     = in(7+NN);
  % theta   = in(8+NN);
  chi     = in(9+NN);
  % p       = in(10+NN);
  % q       = in(11+NN);
  % r       = in(12+NN);
  % Vg      = in(13+NN);
  % wn      = in(14+NN);
  % we      = in(15+NN);
  % psi     = in(16+NN);
  state     =  in(1+NN:16+NN);
  NN = NN + 16;
  t         = in(1+NN);
 
  
  p = [pn; pe; -h];

  persistent waypoints_old   % stored copy of old waypoints
  persistent ptr_a           % waypoint pointer
  persistent ptr_b
  persistent state_transition % state of transition state machine
  persistent dubinspath
  persistent flag_need_new_waypoints % flag that request new waypoints from path planner
  persistent flag_first_time_in_state
  
  if start_of_simulation,
      waypoints_old = zeros(5,P.size_waypoint_array);
      flag_need_new_waypoints = 0;
      state_transition = 0;
      flag_first_time_in_state = 1;
  end
  
  
  % if the waypoints have changed, update the waypoint pointer and plan new
  % Dubin's path
  if min(min(waypoints==waypoints_old))==0,
      waypoints_old = waypoints;
      state_transition = 1;
      ptr_a = 1;
      ptr_b = 2;
      start_node = [waypoints(1:4,ptr_a)', 0, 0];    % previous waypoint position (a waypoint that MAV is flying from)
      end_node   = [waypoints(1:4,ptr_b)', 0, 0];    % current waypoint position (a waypoint that MAV is flying to)
      dubinspath = dubinsParameters(start_node, end_node, P.R_min);  
      flag_need_new_waypoints = 0;
      flag_first_time_in_state = 1;
  end
  
  % define transition state machine
  switch state_transition,
      case 0, % beginning of simulation
          flag   = 1;
          Va_d   = waypoints(5,ptr_a);
          r      = waypoints(1:3,ptr_a);
          w      = waypoints(1:3,ptr_b);
          q      = (w-r)/norm(w-r);
          c      = [0; 0; 0];
          rho    = 1;
          lambda = 1;
          if flag_first_time_in_state,
              flag_first_time_in_state =0;
          end
      
      case 1, % follow first orbit on Dubins path until intersect H1
          flag   = 2;  % following orbit
          Va_d   = waypoints(5,ptr_a); % desired airspeed along waypoint path
          r      = waypoints(1:3,ptr_a);
          q      = dubinspath.q1;
          c      = dubinspath.cs;
          rho    = P.R_min;
          lambda = dubinspath.lams;
          
          if ((p-dubinspath.w1)'*dubinspath.q1 >= 0)&&(flag_first_time_in_state==1), % start in H1
              state_transition = 2;
              flag_first_time_in_state = 1;
          elseif (p-dubinspath.w1)'*dubinspath.q1 >= 0, % entering H1
              state_transition = 3;
              flag_first_time_in_state = 1;
          else
              flag_first_time_in_state = 0;
          end
          
      case 2, % follow first orbit on Dubins path until on right side of H1
          flag   = 2;  % following orbit
          Va_d   = waypoints(5,ptr_a); % desired airspeed along waypoint path
          r      = waypoints(1:3,ptr_a);
          q      = dubinspath.q1;
          c      = dubinspath.cs;
          rho    = P.R_min;
          lambda = dubinspath.lams;
          
          if (p-dubinspath.w1)'*dubinspath.q1 < 0, % get to right side H1
              state_transition = 1;
              flag_first_time_in_state = 1;
          else
              flag_first_time_in_state = 0;
          end
          
      case 3, % follow straight line on Dubins path until intersect H2
          flag   = 1;  % following straight line path
          Va_d   = waypoints(5,ptr_b); % desired airspeed along waypoint path
          r      = dubinspath.w1;
          q      = dubinspath.q1;
          c      = [0; 0; 0];
          rho    = 1;
          lambda = dubinspath.lams;
          flag_first_time_in_state = 0;
          
          if (p-dubinspath.w2)'*dubinspath.q1 >= 0, % entering H2
              state_transition = 4;
              flag_first_time_in_state = 1;
          end
              
      case 4, % follow second orbit on Dubins path until intersect H3
          flag   = 2;  % following orbit
          Va_d   = waypoints(5,ptr_b); % desired airspeed along waypoint path
          r      = dubinspath.w2;
          q      = dubinspath.q3;
          c      = dubinspath.ce;
          rho    = P.R_min;
          lambda = dubinspath.lame;
          flag_first_time_in_state = 0;
          
          if ((p-dubinspath.w3)'*dubinspath.q3 >= 0)&&(flag_first_time_in_state==1), % start in H3
              state_transition = 5;
              flag_first_time_in_state=1;
          elseif (p-dubinspath.w3)'*dubinspath.q3 >= 0, % entering H3
              % increase the waypoint pointer
              if ptr_a==num_waypoints-1,      % original code
                  flag_need_new_waypoints = 1;
                  ptr_b = ptr_a+1;
%               if ptr_a==100,            % modified code to loop around
%                   ptr_b = 1;
%                   ptr_a = ptr_a+1;
%                   state_transition = 1;
%                   flag_first_time_in_state = 1;
              else
                  ptr_a = ptr_a+1;
                  if ptr_a == num_waypoints+1
                      ptr_a = 1;
                  end
                  ptr_b = ptr_a+1;
                  if ptr_b == num_waypoints+1
                      ptr_b = 1;
                  end                  
                  state_transition = 1;
                  flag_first_time_in_state = 1;
              end
              % plan new Dubin's path to next waypoint configuration
              start_node = [waypoints(1:4,ptr_a)', 0, 0];
              end_node   = [waypoints(1:4,ptr_b)', 0, 0];      
              dubinspath = dubinsParameters(start_node, end_node, P.R_min);    
          else
              flag_first_time_in_state = 0;
          end

      case 5, % follow first orbit on Dubins path until on right side of H3
          flag   = 2;  % following orbit
          Va_d   = waypoints(5,ptr_b); % desired airspeed along waypoint path
          r      = dubinspath.w2;
          q      = dubinspath.q3;
          c      = dubinspath.ce;
          rho    = P.R_min;
          lambda = dubinspath.lame;
          flag_first_time_in_state = 0;
          
          if (p-dubinspath.w3)'*dubinspath.q3 < 0, % get to right side of H3
              state_transition = 4;
              flag_first_time_in_state = 1;
          else
              flag_first_time_in_state = 0;
          end
          
              
  end
  
  out = [flag; Va_d; r; q; c; rho; lambda; state; flag_need_new_waypoints];

end

