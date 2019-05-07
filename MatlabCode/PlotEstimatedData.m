function PlotEstimatedData(xTrue, xClean, dTrue, dEstimated)
    close all;
%% Estimated Disturbance Graph
%     M = size(dEstimated,1);
%     K = 1:M;
%     
%     disturbance = figure;
%     subplot(2,1,1)
%     plot(K, dTrue(1:M,1), 'b', K, dEstimated(:,1),'--r', 'LineWidth', 3);
%     lgd1 = legend('$d$','$\hat{d}_c$','interpreter','latex');
%     lgd1.FontSize = 16;
% %     ylim([-200000 100000]);
%     xlim([0 410])
%     ylabel('Bias (m)','FontSize',16);
%     set(gca, 'FontSize', 14);
%     xlabel({'Time (s)';'(a)'});
%     grid on;
%     
%     subplot(2,1,2)
%     plot(K, dTrue(1:M,2), 'b', K, dEstimated(:,2),'--r','LineWidth', 3);
%     lgd2 = legend('$d$','$\hat{d}_c$','interpreter','latex');
%     lgd2.FontSize = 16;
% %     ylim([-600 0]);
%     xlim([0 410]);
%     ylabel('Drift (m/s)','FontSize',16);
%     set(gca,'FontSize',14);
%     xlabel({'Time (s)';'(b)'});
%     grid on;

%% Estimated Clock Bias/Drift Graph    
    P = length(xClean(2:end,1));
    K = 1:P;
    boxRange = 340:400;

%     figure('Units','normalized','Position',[0.3,0.3,0.5,0.5]);
    figure;
    subplot(2,1,1);
%     ax1 = axes('Position',[0.1 0.6 0.57 0.33]);
    plot(K,xTrue(1:P,1), 'b', K, xClean(2:end,1),'--r','LineWidth', 3);
    lgd3 = legend('$\hat{\mathbf{x}}_{GT}$','$\hat{\mathbf{x}}_{c}$','interpreter','latex');
    lgd3.FontSize = 22;
    lgd3.Location = 'southwest';
%     ylim([-50000 1000]);
    xlim([0 410])
    ylabel('Bias (m)');
    xlabel('(a)');
    set(gca,'FontSize',18);
    grid on;
    hold on;
    rectangle('Position',[boxRange(1),-4e4,length(boxRange),1e4]);
    annotation('arrow',[0.8 0.75],[0.7 0.75]);
    hold off;
    
    
    absError = abs(xClean - xTrue);
    ax2 = axes('Position',[0.73 0.75 0.15 0.15]);
%     plot(ax2,boxRange,rltvErrorBias(boxRange,1),'b','Linewidth',2);
%     legend('$\frac{\mid\hat{\mathbf{x}}_{C}(1)-\hat{\mathbf{x}}_{GT}(1)\mid}{\hat{\mathbf{x}}_{GT}(1)}$','interpreter','latex');
%     ylim([0 3e-3]);
    plot(ax2,boxRange,xTrue(boxRange,1),'b',boxRange,xClean(boxRange,1),'--r','Linewidth',3);
    set(gca,'FontSize',18);
%     title('Estimation Error');
    grid on;
    
%     ax3 = axes('Position',[0.1 0.2 0.57 0.265]);
    subplot(2,1,2);
    plot(K(1:end-1), xTrue(2:P,2), 'b', K, xClean(2:end,2),'--r','LineWidth', 3);
%     lgd4 = legend('$\hat{\mathbf{x}}_{GT}$','$\hat{\mathbf{x}}_{c}$','interpreter','latex');
%     lgd4.FontSize = 16;
    ylim([-105 -80]);
    xlim([0 410])
    ylabel('Drift (m/s)');
    xlabel({'Time (s)';'(b)'});
    set(gca,'FontSize',18);
    grid on;

%% Estimated Error Graph
%     Error = xTrue - xClean;
% 
%     errorgraph = figure;
%     subplot(2,1,1);
%     plot(1:P, Error(2:end,1),'b','Linewidth',3);
%     xlabel('Time (s)');
%     ylabel('Bias (m)');
%     ylim([-1000,1000]);
%     xlim([0 410]);
%     set(gca,'FontSize',14);
%     grid on;
%     title('Bias error ($\hat{\mathbf{x}}_{GT}$[1]-$\hat{\mathbf{x}}_{c}$[1])',...
%           'interpreter','latex','FontSize',16);
% %     Error_part = Error(28:36,:);  
% %     axes('Position',[.6 .6 .3 .3])
% %     box on
% %     plot(28:36,Error_part(:,1),'Linewidth',3);
%     
%     subplot(2,1,2);
%     plot(1:P, Error(2:end,2),'b','Linewidth',3);
%     xlabel({'Time (s)','(b)'});
%     ylabel('Drift (m/s)');
%     ylim([-5,5]);
%     xlim([0 410]);
%     set(gca,'FontSize',14);
%     grid on;
%     title('Drift error ($\hat{\mathbf{x}}_{GT}$[2]-$\hat{\mathbf{x}}_{c}$[2])',...
%           'interpreter','latex','FontSize',16);
% %     Error_part = Error(31:33,:);  
% %     axes('Position',[.7 .7 .2 .2])
% %     box on
% %     plot(31:33,Error_part(:,2),'Linewidth',3);
end
