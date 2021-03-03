% estimate_states
%   - estimate the MAV states using gyros, accels, pressure sensors, and
%   GPS.
%
% Outputs are:
%   pnhat    - estimated North position, 
%   pehat    - estimated East position, 
%   hhat     - estimated altitude, 
%   Vahat    - estimated airspeed, 
%   alphahat - estimated angle of attack
%   betahat  - estimated sideslip angle
%   phihat   - estimated roll angle, 
%   thetahat - estimated pitch angel, 
%   chihat   - estimated course, 
%   phat     - estimated roll rate, 
%   qhat     - estimated pitch rate, 
%   rhat     - estimated yaw rate,
%   Vghat    - estimated ground speed, 
%   wnhat    - estimate of North wind, 
%   wehat    - estimate of East wind
%   psihat   - estimate of heading angle
% 
% 
% Modified:  3/15/2010 - RB
%            5/18/2010 - RB
%

function xhat = estimate_states(uu, P)

   % rename inputs
   y_gyro_x      = uu(1);
   y_gyro_y      = uu(2);
   y_gyro_z      = uu(3);
   y_accel_x     = uu(4);
   y_accel_y     = uu(5);
   y_accel_z     = uu(6);
   y_static_pres = uu(7);
   y_diff_pres   = uu(8);
   y_gps_n       = uu(9);
   y_gps_e       = uu(10);
   y_gps_h       = uu(11);
   y_gps_Vg      = uu(12);
   y_gps_course  = uu(13);
   t             = uu(14);
   
  
    % not estimating these states 
    alphahat = 0;
    betahat  = 0;
    bxhat    = 0;
    byhat    = 0;
    bzhat    = 0;
    
    %%%%%%%%%%%%%%%%%%%%%%%%%%%%%
    % low-pass filter for p, q, r
    persistent phat;
    persistent qhat;
    persistent rhat;
    persistent hhat;
    persistent Vahat;
    
    if t==0
        phat = 0;
        qhat = 0;
        rhat = 0;
        hhat = 0;
        Vahat = P.Va0;
    end
    
    phat = P.alpha_lpf_gyro * phat + (1 - P.alpha_lpf_gyro) * y_gyro_x;
    qhat = P.alpha_lpf_gyro * qhat + (1 - P.alpha_lpf_gyro) * y_gyro_y;
    rhat = P.alpha_lpf_gyro * rhat + (1 - P.alpha_lpf_gyro) * y_gyro_z;
    hhat_temp = P.alpha_lpf_static_pres * hhat + (1 - P.alpha_lpf_static_pres) * y_static_pres;
    hhat = hhat_temp / (P.rho * P.gravity);
    Vahat_temp = P.alpha_lpf_diff_pres * Vahat + (1 - P.alpha_lpf_diff_pres) * y_diff_pres;
    Vahat = sqrt(2 / P.rho * Vahat_temp);
    
    %%%%%%%%%%%%%%%%%%%%%%%%
    % EKF for roll and pitch
    persistent phihat;
    persistent thetahat;
    if t==0
        phihat = 0;
        thetahat = 0;        
    end
    
    T_out = 0.01;
    N = 10;
    Q_attitude = 10^(-9) * diag([1 1]);
    R_accel = P.sigma_accel^2;
    P_attitude = zeros(2,2);
    % prediction step
    for i=1:N
       f_x = [phat+qhat*sin(phihat)*tan(thetahat)+rhat*cos(phihat)*tan(thetahat);...
              qhat*cos(phihat)-rhat*sin(phihat)];
       phihat = phihat + (T_out/N)*f_x(1);
       thetahat = thetahat + (T_out/N)*f_x(2);
       A = [qhat*cos(phihat)*tan(thetahat)-rhat*sin(phihat)*tan(thetahat)     (qhat*sin(phihat)-rhat*cos(phihat))/cos(phihat)^2;...
            -qhat*sin(phihat)-rhat*cos(phihat)       0];
       P_attitude = P_attitude + (T_out/N)*(A*P_attitude + P_attitude*A' + Q_attitude);
    end
    % measurement update
    C_1 = [0    qhat*Vahat*cos(thetahat)+P.gravity*cos(thetahat)];
    L_1 = P_attitude * C_1' / (R_accel + C_1*P_attitude*C_1');
    P_attitude = (eye(2) - L_1*C_1) * P_attitude;
    h_1 = qhat*Vahat*sin(thetahat) + P.gravity*sin(thetahat);
    phihat = phihat + L_1(1) * (y_accel_x - h_1);
    thetahat = thetahat + L_1(2) * (y_accel_x - h_1);
    
    C_2 = [-P.gravity*cos(phihat)*cos(thetahat)   -rhat*Vahat*sin(thetahat)-phat*Vahat*cos(thetahat)+P.gravity*sin(phihat)*sin(thetahat)];
    L_2 = P_attitude * C_2' / (R_accel + C_2*P_attitude*C_2');
    P_attitude = (eye(2) - L_2*C_2) * P_attitude;
    h_2 = rhat*Vahat*cos(thetahat) - phat*Vahat*sin(thetahat) - P.gravity*cos(thetahat)*sin(phihat);
    phihat = phihat + L_2(1) * (y_accel_y - h_2);
    thetahat = thetahat + L_2(2) * (y_accel_y - h_2);
    
    C_3 = [P.gravity*sin(phihat)*cos(thetahat)    (qhat*Vahat+P.gravity*cos(phihat))*sin(thetahat)];
    L_3 = P_attitude * C_3' / (R_accel + C_3*P_attitude*C_3');
    P_attitude = (eye(2) - L_3*C_3) * P_attitude;
    h_3 = -qhat*Vahat*cos(thetahat) - P.gravity*cos(thetahat)*cos(phihat);
    phihat = phihat + L_3(1) * (y_accel_z - h_3);
    thetahat = thetahat + L_3(2) * (y_accel_z - h_3);
    
    %%%%%%%%%%%%%%%%%%%%%%%%
    % GPS smoothing
    persistent pnhat;
    persistent pehat;
    persistent Vghat;
    persistent chihat;
    persistent wnhat;
    persistent wehat;
    persistent psihat;
    
    if t==0
        pnhat = 0;
        pehat = 0;
        Vghat = P.Va0;
        chihat = 0;
        wnhat = 0;
        wehat = 0;
        psihat = 0;
    end
    
    T_out = 0.01;
    N = 10;
    Q_gps = diag([1 1 0.1 0.001 0.001 0.01 0.001]);
    P_gps = zeros(7,7);
    % prediction step
    for i=1:N
       f_x = [Vghat*cos(chihat);...
              Vghat*sin(chihat);...
              Vahat*(qhat*sin(phihat)/cos(thetahat)+rhat*cos(phihat)/cos(thetahat))*(-wnhat*sin(psihat)+wehat*cos(psihat))/Vghat;...
              P.gravity/Vghat*tan(phihat)*cos(chihat-psihat);...
              0;...
              0;...
              qhat*sin(phihat)/cos(thetahat)+rhat*cos(phihat)/cos(thetahat);...
              ];
       pnhat = pnhat + (T_out/N)*f_x(1);
       pehat = pehat + (T_out/N)*f_x(2);
       Vghat = Vghat + (T_out/N)*f_x(3);
       chihat = chihat + (T_out/N)*f_x(4);
       wnhat = wnhat + (T_out/N)*f_x(5);
       wehat = wehat + (T_out/N)*f_x(6);
       psihat = psihat + (T_out/N)*f_x(7);
              
       A = [0 0 cos(chihat) -Vghat*sin(chihat) 0 0 0;...
            0 0 sin(chihat) Vghat*cos(chihat) 0 0 0;...
            0 0 -Vahat*(qhat*sin(phihat)/cos(thetahat)+rhat*cos(phihat)/cos(thetahat))*(-wnhat*sin(psihat)+wehat*cos(psihat))/Vghat^2 ...
            0 -(qhat*sin(phihat)/cos(thetahat)+rhat*cos(phihat)/cos(thetahat))*Vahat*sin(psihat) ...
            (qhat*sin(phihat)/cos(thetahat)+rhat*cos(phihat)/cos(thetahat))*Vahat*cos(psihat) ...
            -(qhat*sin(phihat)/cos(thetahat)+rhat*cos(phihat)/cos(thetahat))*Vahat*(wnhat*cos(psihat)+wehat*sin(psihat))/Vghat;...
            0 0 -P.gravity*tan(phihat)*cos(chihat-psihat)/Vghat^2 -P.gravity*tan(phihat)*sin(chihat-psihat)/Vghat 0 0 ...
            P.gravity*tan(phihat)*sin(chihat-psihat)/Vghat;...
            0 0 0 0 0 0 0;...
            0 0 0 0 0 0 0;...
            0 0 0 0 0 0 0
            ];
       P_gps = P_gps + (T_out/N)*(A*P_gps + P_gps*A' + Q_gps);
    end
    
    % measurement update
    if mod(t,1)==0
    C_1 = [1 0 0 0 0 0 0];
    R_gps_n = P.sigma_gps_n^2;
    L_1 = P_gps * C_1' / (R_gps_n + C_1*P_gps*C_1');
    P_gps = (eye(7) - L_1*C_1) * P_gps;
    h_1 = pnhat;
    pnhat = pnhat + L_1(1) * (y_gps_n - h_1);
    pehat = pehat + L_1(2) * (y_gps_n - h_1);
    Vghat = Vghat + L_1(3) * (y_gps_n - h_1);
    chihat = chihat + L_1(4) * (y_gps_n - h_1);
    wnhat = wnhat + L_1(5) * (y_gps_n - h_1);
    wehat = wehat + L_1(6) * (y_gps_n - h_1);
    psihat = psihat + L_1(7) * (y_gps_n - h_1);
    
    C_2 = [0 1 0 0 0 0 0];
    R_gps_e = P.sigma_gps_e^2;
    L_2 = P_gps * C_2' / (R_gps_e + C_2*P_gps*C_2');
    P_gps = (eye(7) - L_2*C_2) * P_gps;
    h_2 = pehat;
    pnhat = pnhat + L_2(1) * (y_gps_e - h_2);
    pehat = pehat + L_2(2) * (y_gps_e - h_2);
    Vghat = Vghat + L_2(3) * (y_gps_e - h_2);
    chihat = chihat + L_2(4) * (y_gps_e - h_2);
    wnhat = wnhat + L_2(5) * (y_gps_e - h_2);
    wehat = wehat + L_2(6) * (y_gps_e - h_2);
    psihat = psihat + L_2(7) * (y_gps_e - h_2);
    
    C_3 = [0 0 1 0 0 0 0];
    R_gps_Vg = P.sigma_gps_V_g^2;
    L_3 = P_gps * C_3' / (R_gps_Vg + C_3*P_gps*C_3');
    P_gps = (eye(7) - L_3*C_3) * P_gps;
    h_3 = Vghat;
    pnhat = pnhat + L_3(1) * (y_gps_Vg - h_3);
    pehat = pehat + L_3(2) * (y_gps_Vg - h_3);
    Vghat = Vghat + L_3(3) * (y_gps_Vg - h_3);
    chihat = chihat + L_3(4) * (y_gps_Vg - h_3);
    wnhat = wnhat + L_3(5) * (y_gps_Vg - h_3);
    wehat = wehat + L_3(6) * (y_gps_Vg - h_3);
    psihat = psihat + L_3(7) * (y_gps_Vg - h_3);
    
    C_4 = [0 0 0 1 0 0 0];
    R_gps_chi = 0;
    L_4 = P_gps * C_4' / (R_gps_chi + C_4*P_gps*C_4');
    P_gps = (eye(7) - L_4*C_4) * P_gps;
    h_4 = chihat;
    pnhat = pnhat + L_4(1) * (y_gps_course - h_4);
    pehat = pehat + L_4(2) * (y_gps_course - h_4);
    Vghat = Vghat + L_4(3) * (y_gps_course - h_4);
    chihat = chihat + L_4(4) * (y_gps_course - h_4);
    wnhat = wnhat + L_4(5) * (y_gps_course - h_4);
    wehat = wehat + L_4(6) * (y_gps_course - h_4);
    psihat = psihat + L_4(7) * (y_gps_course - h_4);
    
    C_5 = [0 0 -cos(chihat) Vghat*sin(chihat) 1 0 -Vahat*sin(psihat)];
    R_gps_wn = 0;
    L_5 = P_gps * C_5' / (R_gps_wn + C_5*P_gps*C_5');
    P_gps = (eye(7) - L_5*C_5) * P_gps;
    h_5 = Vahat*cos(psihat) + P.wind_n - Vghat*cos(chihat);
    pnhat = pnhat + L_5(1) * (P.wind_n - h_5);
    pehat = pehat + L_5(2) * (P.wind_n - h_5);
    Vghat = Vghat + L_5(3) * (P.wind_n - h_5);
    chihat = chihat + L_5(4) * (P.wind_n - h_5);
    wnhat = wnhat + L_5(5) * (P.wind_n - h_5);
    wehat = wehat + L_5(6) * (P.wind_n - h_5);
    psihat = psihat + L_5(7) * (P.wind_n - h_5);

    C_6 = [0 0 -sin(chihat) -Vghat*cos(chihat) 0 1 Vahat*cos(psihat)];
    R_gps_we = 0;
    L_6 = P_gps * C_6' / (R_gps_we + C_6*P_gps*C_6');
    P_gps = (eye(7) - L_6*C_6) * P_gps;
    h_6 = Vahat*sin(psihat) + P.wind_e - Vghat*sin(chihat);
    pnhat = pnhat + L_6(1) * (P.wind_e - h_6);
    pehat = pehat + L_6(2) * (P.wind_e - h_6);
    Vghat = Vghat + L_6(3) * (P.wind_e - h_6);
    chihat = chihat + L_6(4) * (P.wind_e - h_6);
    wnhat = wnhat + L_6(5) * (P.wind_e - h_6);
    wehat = wehat + L_6(6) * (P.wind_e - h_6);
    psihat = psihat + L_6(7) * (P.wind_e - h_6);
    
    end
      xhat = [...
        pnhat;...
        pehat;...
        hhat;...
        Vahat;...
        alphahat;...
        betahat;...
        phihat;...
        thetahat;...
        chihat;...
        phat;...
        qhat;...
        rhat;...
        Vghat;...
        wnhat;...
        wehat;...
        psihat;...
        bxhat;...
        byhat;...
        bzhat;...
        ];
end
