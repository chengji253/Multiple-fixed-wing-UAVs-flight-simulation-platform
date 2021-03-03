% forces_moments.m
%   Computes the forces and moments acting on the airframe. 
%
%   Output is
%       F     - forces
%       M     - moments
%       Va    - airspeed
%       alpha - angle of attack
%       beta  - sideslip angle
%       wind  - wind vector in the inertial frame
%

function out = forces_moments(x, delta, wind, P)

    % relabel the inputs
    pn      = x(1);
    pe      = x(2);
    pd      = x(3);
    u       = x(4);
    v       = x(5);
    w       = x(6);
    phi     = x(7);
    theta   = x(8);
    psi     = x(9);
    p       = x(10);
    q       = x(11);
    r       = x(12);
    delta_e = delta(1);
    delta_a = delta(2);
    delta_r = delta(3);
    delta_t = delta(4);
    w_ns    = wind(1); % steady wind - North
    w_es    = wind(2); % steady wind - East
    w_ds    = wind(3); % steady wind - Down
    u_wg    = wind(4); % gust along body x-axis
    v_wg    = wind(5); % gust along body y-axis    
    w_wg    = wind(6); % gust along body z-axis
    
    % rotation matrix from vehicle to body
    sp = sin(phi);
    cp = cos(phi);
    st = sin(theta);
    ct = cos(theta);
    ss = sin(psi);
    cs = cos(psi);
    
    rotation_to_body = [ct*cs ct*ss -st;
                        sp*st*cs-cp*ss sp*st*ss+cp*cs sp*ct;
                        cp*st*cs+sp*ss cp*st*ss-sp*cs cp*ct
                        ];
    
    V_wind_body = rotation_to_body * [w_ns; w_es; w_ds] + [u_wg; v_wg; w_wg];
    V_airspeed_body = [u-V_wind_body(1); v-V_wind_body(2); w-V_wind_body(3)];
                    
    % compute wind data in NED
    w_n = 0;
    w_e = 0;
    w_d = 0;
    
    % compute air data
    u_r = V_airspeed_body(1);
    v_r = V_airspeed_body(2);
    w_r = V_airspeed_body(3);
    Va = sqrt(u_r^2 + v_r^2 + w_r^2);     % air speed
    alpha = atan(w_r / u_r);  % angle of attack
    beta = asin(v_r / (Va));   % side slip
    
    % coefficients
    sigma = (1 + exp(-P.M * (alpha - P.alpha0)) + exp(P.M * (alpha + P.alpha0))) ...
            / ((1 + exp(-P.M * (alpha - P.alpha0))) * (1 + exp(P.M * (alpha + P.alpha0))));
    C_L = (1 - sigma) * (P.C_L_0 + P.C_L_alpha * alpha) + sigma * (2 * sign(alpha) * (sin(alpha))^2 * cos(alpha));
    C_D = P.C_D_p + (P.C_L_0 + P.C_L_alpha * alpha)^2 / (pi * P.e * P.b^2 / P.S_wing);
    
    C_X = -C_D * cos(alpha) + C_L * sin(alpha);
    C_X_q = -P.C_D_q * cos(alpha) + P.C_L_q * sin(alpha);
    C_X_delta_e = -P.C_D_delta_e * cos(alpha) + P.C_L_delta_e * sin(alpha);
    C_Z = -C_D * sin(alpha) - C_L * cos(alpha);
    C_Z_q = -P.C_D_q * sin(alpha) - P.C_L_q * cos(alpha);
    C_Z_delta_e = -P.C_D_delta_e * sin(alpha) - P.C_L_delta_e * cos(alpha);
    
    % compute external forces and torques on aircraft
    Force(1) = -P.mass * P.gravity * sin(theta) ...
                + 0.5 * P.rho * Va^2 * P.S_wing ...
                * (C_X + C_X_q * P.c * q / (2 * Va) + C_X_delta_e * delta_e) ...
                + 0.5 * P.rho * P.S_prop * P.C_prop * ((P.k_motor * delta_t)^2 - Va^2);
            
    Force(2) =  P.mass * P.gravity * cos(theta) * sin(phi) ...
                + 0.5 * P.rho * Va^2 * P.S_wing ...
                * (P.C_Y_0 + P.C_Y_beta * beta + P.C_Y_p * P.b * p / (2 * Va) ...
                + P.C_Y_r * P.b * r / (2 * Va) + P.C_Y_delta_a * delta_a + P.C_Y_delta_r * delta_r);
            
    Force(3) =  P.mass * P.gravity * cos(theta) * cos(phi) ...
                + 0.5 * P.rho * Va^2 * P.S_wing ...
                * (C_Z + C_Z_q * P.c * q / (2 * Va) + C_Z_delta_e * delta_e);
    
    Torque(1) = 0.5 * P.rho * Va^2 * P.S_wing ...
                * (P.b * (P.C_ell_0 + P.C_ell_beta * beta + P.C_ell_p * P.b * p / (2 * Va) ...
                + P.C_ell_r * P.b * r / (2 * Va) + P.C_ell_delta_a * delta_a + P.C_ell_delta_r * delta_r)) ...
                - P.k_T_P * (P.k_Omega * delta_t)^2;
                
    Torque(2) = 0.5 * P.rho * Va^2 * P.S_wing ...
                * (P.c * (P.C_m_0 + P.C_m_alpha * alpha ...
                + P.C_m_q * P.c * q / (2 * Va) + P.C_m_delta_e * delta_e));   
    Torque(3) = 0.5 * P.rho * Va^2 * P.S_wing ...
                * (P.b * (P.C_n_0 + P.C_n_beta * beta + P.C_n_p * P.b * p / (2 * Va) ...
                + P.C_n_r * P.b * r / (2 * Va) + P.C_n_delta_a * delta_a + P.C_n_delta_r * delta_r));
   
    out = [Force'; Torque'; Va; alpha; beta; w_n; w_e; w_d];
end