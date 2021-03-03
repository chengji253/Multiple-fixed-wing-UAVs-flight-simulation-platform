function [num_waypoints , wpp] = getWpp(P,uav)

load '5jia3.mat'

 a1 = 2;
 a2 = 400;
 num_waypoints = a2/a1;       
%  wpp = [...
%      100, 0, -100, -9999, P.Va0;...
%      200, 100, -100, -9999, 45;...
%      100, 300, -100, -9999, P.Va0;...
%      100, 400, -100, -9999, P.Va0;...
wpp = [];
i1 = uav;



    for i = 1:a1:a2
        x = [];
        x = Xplot2(i,6*i1-5:6*i1-3);
        x(3) = -x(3);
        x = [x -9999 Xplot2(i,6*i1-2)];
        wpp = [wpp;x];
        
    end

% 
% if(uav == 2)
%     for i = 1:a1:a2
%         x = [];
%         x = Xplot2(i,7:9);
%         x(3) = -x(3);
%         x = [x -9999 Xplot2(i,10)];
%         wpp = [wpp;x];     
%     end
% end
%    
% if(uav == 3)
%     for i = 1:a1:a2
%         x = [];
%         x = Xplot2(i,13:15);
%         x(3) = -x(3);
%         x = [x -9999 Xplot2(i,16)];
%         wpp = [wpp;x];     
%     end
% end



