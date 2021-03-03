% some variables needed
u = P.x_trim(4);
v = P.x_trim(5);
w = P.x_trim(6);
Va_trim = sqrt(u^2 + v^2 + w^2);
theta_trim = P.x_trim(8);
alpha_trim = atan(w/u);
delta_e_trim = P.u_trim(1);
delta_t_trim = P.u_trim(4);

% Roll attitude hold
P.delta_a_max = 45 * pi / 180;
P.phi_max = 15 * pi / 180;
P.roll_max = 45 * pi / 180;
a_phi1 = -0.5 * P.rho * Va_trim^2 * P.S_wing * P.b * P.C_p_p * P.b / (2 * Va_trim);
a_phi2 = 0.5 * P.rho * Va_trim^2 * P.S_wing * P.b * P.C_p_delta_a;
P.kp_phi = P.delta_a_max / P.phi_max * sign(a_phi2);
omega_phi = sqrt(abs(a_phi2) * P.delta_a_max / P.phi_max);
zeta_phi = 1.2;   % design parameter
P.kd_phi = (2 * zeta_phi * omega_phi - a_phi1) / a_phi2;
P.ki_phi = 0.2;    % design parameter

% Course hold
W_chi = 8;  % design parameter
omega_chi = 1 / W_chi * omega_phi;
zeta_chi = 0.707;   % design parameter
Vg = P.Va0;
P.kp_chi = 2 * zeta_chi * omega_chi * Vg / P.gravity;
P.ki_chi = omega_chi^2 * Vg / P.gravity;

% Pitch attitude hold
P.delta_e_max = 45 * pi / 180;
P.e_theta_max = 10 * pi / 180;
a_theta1 = -P.rho * Va_trim^2 * P.c * P.S_wing * P.C_m_q * P.c / (2 * P.Jy * 2 * Va_trim);
a_theta2 = -P.rho * Va_trim^2 * P.c * P.S_wing * P.C_m_alpha / (2 * P.Jy);
a_theta3 = P.rho * Va_trim^2 * P.c * P.S_wing * P.C_m_delta_e / (2 * P.Jy);
P.kp_theta = P.delta_e_max / P.e_theta_max * sign(a_theta3);
omega_theta = sqrt(a_theta2 + P.delta_e_max / P.e_theta_max * abs(a_theta3));
zeta_theta = 0.707;     % design parameter
P.kd_theta = (2 * zeta_theta * omega_theta - a_theta1) / a_theta3;
P.theta_max = 20 * pi / 180;
P.pitch_max = 40 * pi / 180;

% Airspeed hold using Throttle
a_V1 = P.rho * Va_trim * P.S_wing / P.mass ...
       * (P.C_D_0 + P.C_D_alpha * alpha_trim + P.C_L_delta_e * delta_e_trim) ...
       + P.rho * P.S_prop / P.mass * P.C_prop * Va_trim;
a_V2 = P.rho * P.S_prop / P.mass * P.C_prop * P.k_motor^2 * delta_t_trim;
a_V3 = P.gravity * cos(theta_trim - alpha_trim);
omega_v = 5;   % design parameter
zeta_v = 0.707;     % design parameter
P.delta_t_max = 1;
P.delta_t_min = 0;
P.ki_v = omega_v^2 / a_V2;
P.kp_v = (2 * zeta_v * omega_v - a_V1) / a_V2;

% Airspeed hold using Pitch
W_v2 = 7;   % design parameter
zeta_v2 = 0.707;    % design parameter
omega_v2 = 1 / W_v2 * omega_theta;
K_theta_dc = P.kp_theta * a_theta3 / (a_theta2 + P.kp_theta * a_theta3);
P.ki_v2 = -omega_v2^2/(K_theta_dc * P.gravity);
P.kp_v2 = (a_V1 - 2 * zeta_v2 * omega_v2) / (K_theta_dc * P.gravity);

% Altitude hold using Pitch
W_h = 10;   % design parameter
omega_h = 1 / W_h * omega_theta;
Va = P.Va0;
zeta_h = 1.2;     % design parameter
P.h_max = 1000;
P.h_min = 0;
P.ki_h = omega_h^2 / (K_theta_dc * Va);
P.kp_h = 2 * zeta_h * omega_h / (K_theta_dc * Va);

