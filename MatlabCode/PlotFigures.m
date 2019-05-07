clc;
close all;
clear all;

load('RSE_data.mat');

N = length(Xhat);

PlotEstimatedData(X_EKF, Xhat, ClockDisturbanceWrong, Dhat);

absError = abs(Xhat - X_EKF);

% absoluteError = figure;
% subplot(2,1,1);
% plot(1:N,absError(:,1),'b','Linewidth',3);
% title('$\mid\hat{\mathbf{x}}_{C}(1)$-$\hat{\mathbf{x}}_{GT}(1)\mid$','interpreter','latex');
% ylabel('Bias (m)');
% subplot(2,1,2);
% plot(1:N,absError(:,2),'b','Linewidth',3);
% title('$\mid\hat{\mathbf{x}}_{C}(2)$-$\hat{\mathbf{x}}_{GT}(2)\mid$','interpreter','latex');
% ylabel('Drift (m/s)');
% 
% 
% for i = 1:1:N
%     nrmError(i) = norm(X_EKF(i,:)-Xhat(i,:),2);
% end
% normError = figure;
% plot(1:N,nrmError,'b','Linewidth',3);
% title('$|| \hat{\mathbf{x}}_{C}$-$\hat{\mathbf{x}}_{GT}||_2$','interpreter','latex');


rltvErrorBias = absError(:,1)/norm(X_EKF(:,1));
rltvErrorDrift = absError(:,2)/norm(X_EKF(:,2));
relativeError = figure;
plot(1:N,rltvErrorBias,'b','Linewidth',3);
hold on;
% title('$\frac{\mid\hat{\mathbf{x}}_{C}(1)-\hat{\mathbf{x}}_{GT}(1)\mid}{||\hat{\mathbf{x}}_{GT}(1)||}$','interpreter','latex');
plot(1:N,rltvErrorDrift,'r','Linewidth',3);
ylim([0 4.0e-3]);
xlabel('Time(s)','FontSize',22);
ylabel('Relative Error','FontSize',22);
set(gca,'FontSize',22);
ldg = legend('$\mathrm{RE}_1$','$\mathrm{RE}_2$','interpreter','latex');
legend('Location','Northwest');
legend('FontSize',22);




