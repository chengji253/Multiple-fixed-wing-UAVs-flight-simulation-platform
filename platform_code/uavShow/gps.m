% gps.m
%   Compute the output of gps sensor
%
%  Revised:
%   3/5/2010 - RB 
%   5/14/2010 - RB

function y = gps(uu, P)

    % relabel the inputs
    Va      = uu(1);
%    alpha   = uu(2);
%    beta    = uu(3);
    wn      = uu(4);
    we      = uu(5);
%    wd      = uu(6);
    pn      = uu(7);
    pe      = uu(8);
    pd      = uu(9);
%    u       = uu(10);
%    v       = uu(11);
%    w       = uu(12);
%    phi     = uu(13);
%    theta   = uu(14);
    psi     = uu(15);
%    p       = uu(16);
%    q       = uu(17);
%    r       = uu(18);
    t       = uu(19);
    
    persistent v_n;
    persistent v_e;
    persistent v_h;
    
    if t==0
        v_n = 0;
        v_e = 0;
        v_h = 0;
    else
        v_n = exp(-P.k_gps * P.Ts_gps) * v_n + P.sigma_gps_n * randn;
        v_e = exp(-P.k_gps * P.Ts_gps) * v_e + P.sigma_gps_e * randn;
        v_h = exp(-P.k_gps * P.Ts_gps) * v_h + P.sigma_gps_altitude * randn;
    end
    
    % construct North, East, and altitude GPS measurements
    y_gps_n = pn + v_n;
    y_gps_e = pe + v_e; 
    y_gps_h = -pd + v_h; 
    
    % construct groundspeed and course measurements
    V_n = Va * cos(psi) + wn;
    V_e = Va * sin(psi) + we;
    V_g = sqrt(V_n^2 + V_e^2);
    
    sigma_course = P.sigma_gps_V_g / V_g;
    
    y_gps_Vg     = sqrt(V_n^2 + V_e^2) + P.sigma_gps_V_g * randn;
    y_gps_course = atan2(V_e, V_n) + sigma_course * randn;

    % construct total output
    y = [...
        y_gps_n;...
        y_gps_e;...
        y_gps_h;...
        y_gps_Vg;...
        y_gps_course;...
        ];
    
end



