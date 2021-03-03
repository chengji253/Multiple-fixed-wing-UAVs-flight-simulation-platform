function y = autopilot(uu,P)
%
% autopilot for mavsim
% 
% Modification History:
%   2/11/2010 - RWB
%   5/14/2010 - RWB
%   9/30/2014 - RWB
%   

    % process inputs
    NN = 0;
%    pn       = uu(1+NN);  % inertial North position
%    pe       = uu(2+NN);  % inertial East position
    h        = uu(3+NN);  % altitude
    Va       = uu(4+NN);  % airspeed
%    alpha    = uu(5+NN);  % angle of attack
%    beta     = uu(6+NN);  % side slip angle
    phi      = uu(7+NN);  % roll angle
    theta    = uu(8+NN);  % pitch angle
    chi      = uu(9+NN);  % course angle
    p        = uu(10+NN); % body frame roll rate
    q        = uu(11+NN); % body frame pitch rate
    r        = uu(12+NN); % body frame yaw rate
%    Vg       = uu(13+NN); % ground speed
%    wn       = uu(14+NN); % wind North
%    we       = uu(15+NN); % wind East
%    psi      = uu(16+NN); % heading
%    bx       = uu(17+NN); % x-gyro bias
%    by       = uu(18+NN); % y-gyro bias
%    bz       = uu(19+NN); % z-gyro bias
    NN = NN+19;
    Va_c     = uu(1+NN);  % commanded airspeed (m/s)
    h_c      = uu(2+NN);  % commanded altitude (m)
    chi_c    = uu(3+NN);  % commanded course (rad)
    NN = NN+3;
    t        = uu(1+NN);   % time
    
    autopilot_version = 2;
        % autopilot_version == 1 <- used for tuning
        % autopilot_version == 2 <- standard autopilot defined in book
        % autopilot_version == 3 <- Total Energy Control for longitudinal AP
    switch autopilot_version
        case 1,
           [delta, x_command] = autopilot_tuning(Va_c,h_c,chi_c,Va,h,chi,phi,theta,p,q,r,t,P);
        case 2,
           [delta, x_command] = autopilot_uavbook(Va_c,h_c,chi_c,Va,h,chi,phi,theta,p,q,r,t,P);
        case 3,
               [delta, x_command] = autopilot_TECS(Va_c,h_c,chi_c,Va,h,chi,phi,theta,p,q,r,t,P);
    end
    y = [delta; x_command];
end
    
   
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% Autopilot versions
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%


%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% autopilot_tuning
%   - used to tune each loop
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
function [delta, x_command] = autopilot_tuning(Va_c,h_c,chi_c,Va,h,chi,phi,theta,p,q,r,t,P)

    mode = 5;
    switch mode
        case 1, % tune the roll loop
            phi_c = chi_c; % interpret chi_c to autopilot as course command
            delta_a = roll_hold(phi_c, phi, p, P);
            delta_r = 0; % no rudder
            % use trim values for elevator and throttle while tuning the lateral autopilot
            delta_e = P.u_trim(1);
            delta_t = P.u_trim(4);
            theta_c = 0;
        case 2, % tune the course loop
            if t==0,
                phi_c   = course_hold(chi_c, chi, r, 1, P);
            else
                phi_c   = course_hold(chi_c, chi, r, 0, P);
            end                
            delta_a = roll_hold(phi_c, phi, p, P);
            delta_r = 0; % no rudder
            % use trim values for elevator and throttle while tuning the lateral autopilot
            delta_e = P.u_trim(1);
            delta_t = P.u_trim(4);
            theta_c = 0;
        case 3, % tune the throttle to airspeed loop and pitch loop simultaneously
            theta_c = 20*pi/180 + h_c;
            chi_c = 0;
            if t==0,
                phi_c   = course_hold(chi_c, chi, r, 1, P);
                delta_t = airspeed_with_throttle_hold(Va_c, Va, 1, P);
           else
                phi_c   = course_hold(chi_c, chi, r, 0, P);
                delta_t = airspeed_with_throttle_hold(Va_c, Va, 0, P);
            end
            delta_e = pitch_hold(theta_c, theta, q, P);
            delta_a = roll_hold(phi_c, phi, p, P);
            delta_r = 0; % no rudder
            % use trim values for elevator and throttle while tuning the lateral autopilot
        case 4, % tune the pitch to airspeed loop 
            chi_c = 0;
            delta_t = P.u_trim(4);
            if t==0,
                phi_c   = course_hold(chi_c, chi, r, 1, P);
                theta_c = airspeed_with_pitch_hold(Va_c, Va, 1, P);
           else
                phi_c   = course_hold(chi_c, chi, r, 0, P);
                theta_c = airspeed_with_pitch_hold(Va_c, Va, 0, P);
            end
            delta_a = roll_hold(phi_c, phi, p, P);
            delta_e = pitch_hold(theta_c, theta, q, P);
            delta_r = 0; % no rudder
            % use trim values for elevator and throttle while tuning the lateral autopilot
        case 5, % tune the pitch to altitude loop 
            chi_c = 0;
            if t==0,
                phi_c   = course_hold(chi_c, chi, r, 1, P);
                theta_c = altitude_hold(h_c, h, 1, P);
                delta_t = airspeed_with_throttle_hold(Va_c, Va, 1, P);
           else
                phi_c   = course_hold(chi_c, chi, r, 0, P);
                theta_c = altitude_hold(h_c, h, 0, P);
                delta_t = airspeed_with_throttle_hold(Va_c, Va, 0, P);
            end
            delta_a = roll_hold(phi_c, phi, p, P);
            delta_e = pitch_hold(theta_c, theta, q, P);
            delta_r = 0; % no rudder
            % use trim values for elevator and throttle while tuning the lateral autopilot
         case 6, % tune the pitch loop
            phi_c = 0;
            theta_c = 20*pi/180 + h_c;
            delta_a = P.u_trim(2);
            delta_r = 0; % no rudder
            % use trim values for elevator and throttle while tuning the lateral autopilot
            delta_e = pitch_hold(theta_c, theta, q, P);
            delta_t = P.u_trim(4);
    end
    %----------------------------------------------------------
    % create outputs
    
    % control outputs
    delta = [delta_e; delta_a; delta_r; delta_t];
    % commanded (desired) states
    x_command = [...
        0;...                    % pn
        0;...                    % pe
        h_c;...                  % h
        Va_c;...                 % Va
        0;...                    % alpha
        0;...                    % beta
        phi_c;...                % phi
        %theta_c*P.K_theta_DC;... % theta
        theta_c;
        chi_c;...                % chi
        0;...                    % p
        0;...                    % q
        0;...                    % r
        ];
            
end

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% autopilot_uavbook
%   - autopilot defined in the uavbook
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
function [delta, x_command] = autopilot_uavbook(Va_c,h_c,chi_c,Va,h,chi,phi,theta,p,q,r,t,P)

    %----------------------------------------------------------
    % lateral autopilot
    if t==0,
        % assume no rudder, therefore set delta_r=0
        delta_r = 0;%coordinated_turn_hold(beta, 1, P);
        phi_c   = course_hold(chi_c, chi, r, 1, P);

    else
        phi_c   = course_hold(chi_c, chi, r, 0, P);
        delta_r = 0;%coordinated_turn_hold(beta, 0, P);
    end
    delta_a = roll_hold(phi_c, phi, p, P);
  
    
    %----------------------------------------------------------
    % longitudinal autopilot
    
    % define persistent variable for state of altitude state machine
    persistent altitude_state;
    persistent initialize_integrator;
    % initialize persistent variable
    if h<=P.altitude_take_off_zone,     
        altitude_state = 1;
    elseif h<=h_c-P.altitude_hold_zone, 
        altitude_state = 2;
    elseif h>=h_c+P.altitude_hold_zone, 
        altitude_state = 3;
    else
        altitude_state = 4;
    end
    initialize_integrator = 1;
%     display(altitude_state);
    % implement state machine
    switch altitude_state,
        case 1,  % in take-off zone
            delta_t = 0.5;
            theta_c = P.theta_max;
        case 2,  % climb zone
            delta_t = 0.5;
            theta_c = airspeed_with_pitch_hold(Va_c, Va, 0, P);
        case 3, % descend zone
            delta_t = 0;
            theta_c = airspeed_with_pitch_hold(Va_c, Va, 0, P);
        case 4, % altitude hold zone
            delta_t = airspeed_with_throttle_hold(Va_c, Va, flag, P);
            theta_c = altitude_hold(h_c, h, flag, P);
    end
    
    delta_e = pitch_hold(theta_c, theta, q, P);
    % artificially saturation delta_t
    delta_t = sat(delta_t,1,0);
 
    
    %----------------------------------------------------------
    % create outputs
    
    % control outputs
    delta = [delta_e; delta_a; delta_r; delta_t];
    % commanded (desired) states
    x_command = [...
        0;...                    % pn
        0;...                    % pe
        h_c;...                  % h
        Va_c;...                 % Va
        0;...                    % alpha
        0;...                    % beta
        phi_c;...                % phi
        %theta_c*P.K_theta_DC;... % theta
        theta_c;
        chi_c;...                % chi
        0;...                    % p
        0;...                    % q
        0;...                    % r
        ];
            
    y = [delta; x_command];
 
    end

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% autopilot_TECS
%   - longitudinal autopilot based on total energy control systems
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
function [delta, x_command] = autopilot_TECS(Va_c,h_c,chi_c,Va,h,chi,phi,theta,p,q,r,t,P)

    %----------------------------------------------------------
    % lateral autopilot
    if t==0,
        % assume no rudder, therefore set delta_r=0
        delta_r = 0;%coordinated_turn_hold(beta, 1, P);
        phi_c   = course_hold(chi_c, chi, r, 1, P);

    else
        phi_c   = course_hold(chi_c, chi, r, 0, P);
        delta_r = 0;%coordinated_turn_hold(beta, 0, P);
    end
    delta_a = roll_hold(phi_c, phi, p, P);     
  
    
    %----------------------------------------------------------
    % longitudinal autopilot based on total energy control
    
    
    delta_e = 0;
    delta_t = 0;
 
    
    %----------------------------------------------------------
    % create outputs
    
    % control outputs
    delta = [delta_e; delta_a; delta_r; delta_t];
    % commanded (desired) states
    x_command = [...
        0;...                    % pn
        0;...                    % pe
        h_c;...                  % h
        Va_c;...                 % Va
        0;...                    % alpha
        0;...                    % beta
        phi_c;...                % phi
        %theta_c*P.K_theta_DC;... % theta
        theta_c;
        chi_c;...                % chi
        0;...                    % p
        0;...                    % q
        0;...                    % r
        ];
            
    y = [delta; x_command];
 
end
   


%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% Autopilot functions
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
function delta_a = roll_hold(phi_c, phi, p, P)
    persistent integrator;
    persistent error_d1;
    if isempty(integrator) % reset (initialize) persistent variables when flag==1
        integrator = 0;
        error_d1 = 0;   % _d1 means delayed by one time step
    end
    error = phi_c - phi;    % compute the current error
    integrator = integrator + (P.Ts / 2) * (error + error_d1); % update integrator
    error_d1 = error; % update the error for next time through the loop
    delta_a = sat(...                   % implement PID control
        P.kp_phi * error + ...         % proportional term
        P.ki_phi * integrator - ...    % integral term
        P.kd_phi * p,...  % derivative term
        P.delta_a_max, -P.delta_a_max ... % ensure abs(u)<=limit
        );
    if P.ki_phi~=0        % implement integrator anti-windup
        u_unsat = P.kp_phi * error + P.ki_phi * integrator - P.kd_phi * p;
        integrator = integrator + P.Ts / P.ki_phi * (delta_a - u_unsat);
    end
end

function phi_c = course_hold(chi_c, chi, r, flag, P)
    persistent integrator;
    persistent error_d1;
    if isempty(integrator)        % reset (initialize) persistent variables when flag==1
        integrator = 0;
        error_d1 = 0;   % _d1 means delayed by one time step
    end
    error = chi_c - chi;    % compute the current error
    integrator = integrator + (P.Ts / 2) * (error + error_d1); % update integrator
    error_d1 = error; % update the error for next time through the loop
    phi_c = sat(...                   % implement PID control
        P.kp_chi * error + ...         % proportional term
        P.ki_chi * integrator, ...    % integral term
        P.roll_max, -P.roll_max ... % ensure abs(u)<=limit
        );
    if P.ki_chi~=0        % implement integrator anti-windup
        u_unsat = P.kp_chi * error + P.ki_chi * integrator;
        integrator = integrator + P.Ts / P.ki_chi * (phi_c - u_unsat);
    end
end

function delta_e = pitch_hold(theta_c, theta, q, P)
    persistent error_d1;
    if isempty(error_d1)       % reset (initialize) persistent variables when flag==1
        error_d1 = 0;   % _d1 means delayed by one time step
    end
    error = theta_c - theta;    % compute the current error\
    error_d1 = error; % update the error for next time through the loop
    delta_e = sat(...                   % implement PID control
        P.kp_theta * error - ...         % proportional term
        P.kd_theta * q,...  % derivative term
        P.delta_e_max, -P.delta_e_max ... % ensure abs(u)<=limit
        );
end

function delta_t = airspeed_with_throttle_hold(Va_c, Va, flag, P)
    persistent integrator;
    persistent error_d1;
    if isempty(integrator)        % reset (initialize) persistent variables when flag==1
        integrator = 0;
        error_d1 = 0;   % _d1 means delayed by one time step
    end
    error = Va_c - Va;    % compute the current error
    integrator = integrator + (P.Ts / 2) * (error + error_d1); % update integrator
    error_d1 = error; % update the error for next time through the loop
    delta_t = sat(...                   % implement PID control
        P.kp_v * error + ...         % proportional term
        P.ki_v * integrator, ...    % integral term
        P.delta_t_max, P.delta_t_min ... % ensure abs(u)<=limit
        );
    if P.ki_v~=0        % implement integrator anti-windup
        u_unsat = P.kp_v * error + P.ki_v * integrator;
        integrator = integrator + P.Ts / P.ki_v * (delta_t - u_unsat);
    end
end

function theta_c = airspeed_with_pitch_hold(Va_c, Va, flag, P)
    persistent integrator;
    persistent error_d1;
    if isempty(integrator)        % reset (initialize) persistent variables when flag==1
        integrator = 0;
        error_d1 = 0;   % _d1 means delayed by one time step
    end
    error = Va_c - Va;    % compute the current error
    integrator = integrator + (P.Ts / 2) * (error + error_d1); % update integrator
    error_d1 = error; % update the error for next time through the loop
    theta_c = sat(...                   % implement PID control
        P.kp_v2 * error + ...         % proportional term
        P.ki_v2 * integrator, ...    % integral term
        P.pitch_max, -P.pitch_max ... % ensure abs(u)<=limit
        );
    if P.ki_v2~=0        % implement integrator anti-windup
        u_unsat = P.kp_v2 * error + P.ki_v2 * integrator;
        integrator = integrator + P.Ts / P.ki_v2 * (theta_c - u_unsat);
    end
end

function theta_c = altitude_hold(h_c, h, flag, P)
    persistent integrator;
    persistent error_d1;
    if isempty(integrator)        % reset (initialize) persistent variables when flag==1
        integrator = 0;
        error_d1 = 0;   % _d1 means delayed by one time step
    end
    error = h_c - h;    % compute the current error
    integrator = integrator + (P.Ts / 2) * (error + error_d1); % update integrator
    error_d1 = error; % update the error for next time through the loop
    theta_c = sat(...                   % implement PID control
        P.kp_h * error + ...         % proportional term
        P.ki_h * integrator, ...    % integral term
        P.pitch_max, -P.pitch_max ...    % ensure abs(u)<=limit
        );
    if P.ki_h~=0        % implement integrator anti-windup
        u_unsat = P.kp_h * error + P.ki_h * integrator;
        integrator = integrator + P.Ts / P.ki_h * (theta_c - u_unsat);
    end    
end

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% sat
%   - saturation function
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
function out = sat(in, up_limit, low_limit)
  if in > up_limit,
      out = up_limit;
  elseif in < low_limit;
      out = low_limit;
  else
      out = in;
  end
end
  
 