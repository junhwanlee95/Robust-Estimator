function [xhatstore]= EKFonBiasModel(SatState,xState, WP, WR)
% This is the EKF over the clock Model
% x(k+1)= A x(k)+w(k)
% rho (k) = f(x(k))+v(k)
% where x=[ b bdot]
% This code has been started by Ali Khalajmehrabadi
% on February, 24, 2017
% Input Data Structure

%
% xx = [svid , prM , prrMps, satPos, dtsv, svXyzDot, dtsvDot];
%        1      2     3       4:6     7      8:10        11

% xState
% [ x y z cb xdot ydot zdot cdotb]



deltat=1; % positioning interval
N = size(xState,1);% total number of steps
maxIter = 400;
% % attack=20;
% % outlier_index=50;
xstore=[xState(:,4) xState(:,8)];

x = [];
x=[xState(1,4); xState(1,8)];
% % % xoutlier=x;
% % % xoutlier(outlier_index:end,2)=xoutlier(outlier_index:end,2)+attack;

SigmaX = diag([30 3]);


% Reserve storage for variables we might want to plot/evaluate
xhat = x;

xhatstore = zeros(maxIter, length(x));
xhatstoreOutlier = zeros(maxIter, length(x));

xhatstore (1,:) = x' ;

A= [1 deltat;0 1];
B= blkdiag(GpsConstants.LIGHTSPEED,GpsConstants.LIGHTSPEED);
h_0= 2*10e-19; h_2=2*10e-20;
h_0= 8*10e-20; h_2=4*10e-23;
S_bias=h_0/2; S_drift=2*pi^2*h_2;
Q = [S_bias*deltat+S_drift*(deltat^3)/3     S_drift*(deltat^2)/2;
    S_drift*(deltat^2)/2                   S_drift*deltat];


% Without Attack

for i=2:maxIter
    
    %SatState
    % xx = [svid , prM , prrMps, satPos, dtsv, svXyzDot, dtsvDot];
    %        1      2     3       4:6     7      8:10        11
    
    % xState
    % [ x y z cb xdot ydot zdot cdotb]
    
    R1= WP {i};
    
    R2= WR {i};
    
    SigmaV = blkdiag(R1, R2);
    SigmaX = diag([30 3]);
    
    w = [1000 1]'*rand;
    
    rho=[SatState{i}(:,2) ; SatState{i}(:,3)];
    SatPos=SatState{i}(:,4:6);
    SatVel=SatState{i}(:,8:10);
    SatBias=SatState{i}(:,7)*GpsConstants.LIGHTSPEED;
    SatDrift=SatState{i}(:,11)*GpsConstants.LIGHTSPEED;
    
    C=[ones(size(SatState{i},1),1) zeros(size(SatState{i},1),1); zeros(size(SatState{i},1),1) ones(size(SatState{i},1),1)];
    
    % KF Step 1: State estimate time update
    xhat = A*xhat; % use prior value of "u“
    
    % KF Step 2: Error covariance time update
    SigmaX = A*SigmaX*A' + B*Q*B;
    
    Const=zeros(size(SatPos,1)*2, 1);
    
    for ii=1:size(SatPos,1)
        
        Const(ii,1)= norm( xState(i,1:3)-SatPos(ii,:),2)-SatBias(ii);
        
    end
    for ii=1:size(SatVel,1)
        
        Const(ii+size(SatPos,1),1)= dot(SatVel(ii,:), SatPos(ii,:)-xState(i,1:3))/norm(xState(i,1:3)-SatPos(ii,:),2)-SatDrift(ii);
        
    end
    
    % KF Step 3: Estimate system output
    zhat = C*xhat + Const + C*w;
    
    % KF Step 4: Compute Kalman gain matrix
    %     G = SigmaX*C'/(C*SigmaX*C' + SigmaV);
    G = SigmaX*C'*pinv(C*SigmaX*C' + SigmaV);
    
    % KF Step 5: State estimate measurement update
    xhat = xhat + G*(rho - zhat);
    
    % KF Step 6: Error covariance measurement update
    SigmaX = SigmaX - G*C*SigmaX;
    
    
    
    xhatstore (i,:) =xhat' ;
    
    
end




% % % figure
% % % label = {char('Bias (m)'), char('Drift (m/s)')};
% % % for i = 1:2
% % %     subplot(2,1,i)
% % %     plot(1:maxIter, xhatstore (:,i),'--b')
% % %     hold on;grid on;
% % % %     plot(1:maxIter, xstore(1:maxIter,i),'--b')
% % % %     hold on;
% % %     plot(1:maxIter, xhatstoreOutlier (:,i),'-r')
% % %     hold on;grid on;
% % % %     plot(1:maxIter, xstoreOutlier(1:maxIter,i),'--g')
% % % %     grid on;
% % % %     legend('EKF','WLS','EKFSpoofed','WLSSpoofed')
% % %     legend('EKF Normal','EKF Spoofed')
% % %     xlabel('Time(s)')
% % %     ylabel(label{i})
% % % end
% % % ha = axes('Position',[0 0 1 1],'Xlim',[0 1],'Ylim',[0 1],'Box','off','Visible','off','Units','normalized', 'clipping' , 'off');
% % % text(0.5, 1,'\bf Clock Bias and Drift','HorizontalAlignment','center','VerticalAlignment', 'top');

% 
% for ii = 2:1:N
%     deltaxhatstore(ii-1,1) = xhatstore(ii,1)-xhatstore(ii-1,1);
%     deltaxhatstore(ii-1,2) = xhatstore(ii,2)-xhatstore(ii-1,2);
% 
% end
% 
% figure
% label = {char('Bias (m)'), char('Drift (m/s)')};
% i=1;
% subplot(2,1,i)
% plot(1:maxIter, xhatstore (:,i),'b')
% hold on;
% % plot(1:maxIter, xhatstoreOutlier (:,i),'-r')
% % hold on;
% % plot(1:maxIter, xPF,'k')
% grid on;
% % legend('EKF Normal','EKF Spoofed','Anti-Spoofed PF')
% legend('EKF Normal')
% xlabel('Time(s)')
% ylabel(label{i})
% 
% 
% i=2;
% subplot(2,1,i)
% plot(1:maxIter, xhatstore (:,i),'b')
% hold on;grid on;
% %     plot(1:maxIter, xstore(1:maxIter,i),'--b')
% %     hold on;
% % plot(1:maxIter, xhatstoreOutlier (:,i),'-r')
% % hold on;grid on;
% %     plot(1:maxIter, xstoreOutlier(1:maxIter,i),'--g')
% %     grid on;
% %     legend('EKF','WLS','EKFSpoofed','WLSSpoofed')
% % legend('EKF Normal','EKF Spoofed')
% legend('EKF Normal')
% xlabel('Time(s)')
% ylabel(label{i})
% 
% ha = axes('Position',[0 0 1 1],'Xlim',[0 1],'Ylim',[0 1],'Box','off','Visible','off','Units','normalized', 'clipping' , 'off');
% text(0.5, 1,'\bf Clock Bias and Drift','HorizontalAlignment','center','VerticalAlignment', 'top');
% 
% EKF = sqrt(sum(((xhatstore (:,1)- xhatstoreOutlier (:,1))).^2))/386


% figure
% label = {char('Bias Diff (m)'), char('Drift Diff (m/s)')};
%
% subplot(2,1,1)
% plot(1:maxIter, xhatstoreOutlier(:,1)- xhatstore (:,1),'--b')
% ylabel(label{1})
% hold on;
%
% grid on;
% subplot(2,1,2)
% plot(1:maxIter, xhatstoreOutlier(:,2)- xhatstore (:,2),'-r')
% grid on;
% xlabel('Time(s)')
% ylabel(label{2})
%
% ha = axes('Position',[0 0 1 1],'Xlim',[0 1],'Ylim',[0 1],'Box','off','Visible','off','Units','normalized', 'clipping' , 'off');
% text(0.5, 1,'\bf Clock Bias and Drift Difference','HorizontalAlignment','center','VerticalAlignment', 'top');
%






end
