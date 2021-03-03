function out=airdata(uu)
%
% Fake air data - this will be replaced with real air data in Chapter 4
%
% modified 12/11/2009 - RB

    % process inputs to function
    pn          = uu(1);             % North position (meters)
    pe          = uu(2);             % East position (meters)
    h           = -uu(3);            % altitude (meters)
    u           = uu(4);             % body velocity along x-axis (meters/s)
    v           = uu(5);             % body velocity along y-axis (meters/s)
    w           = uu(6);             % body velocity along z-axis (meters/s)
    phi         = 180/pi*uu(7);      % roll angle (degrees)   
    theta       = 180/pi*uu(8);      % pitch angle (degrees)
    psi         = 180/pi*uu(9);      % yaw angle (degrees)
    p           = 180/pi*uu(10);     % body angular rate along x-axis (degrees/s)
    q           = 180/pi*uu(11);     % body angular rate along y-axis (degrees/s)
    r           = 180/pi*uu(12);     % body angular rate along z-axis (degrees/s)

    Va = sqrt(u^2+v^2+w^2);
    alpha = atan2(w,u);
    beta  = asin(v);
    wn    = 0;
    we    = 0;
    wd    = 0;
    
    out = [Va; alpha; beta; wn; we; wd];
    