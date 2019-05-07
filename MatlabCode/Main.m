clc;
close all;
clear all;

load('RSE_data.mat');

%% Select Type of Attack
Type = 2; % or 2

if Type == 1
   [gpsPvt, gnssMeas, SatState, xState, WP, WR, SatStateOutlier, xStateOutlier, ...
    WPOutlier, WROutlier, attacktime] = GpsWlsPvtTypeI(gnssMeas,allGpsEph,dirName);
elseif Type == 2
   [gpsPvt, gnssMeas, SatState, xState, WP, WR, SatStateOutlier, xStateOutlier, ...
    WPOutlier, WROutlier, attacktime] = GpsWlsPvtTypeII(gnssMeas,allGpsEph,dirName);
end
%% Constructing state parameter
Pu = [median(xStateOutlier(:,1)) median(xStateOutlier(:,2)) median(xStateOutlier(:,3))];

[Constant_L, Observation_Y] = creatingConstantVector(SatState,Pu);
[Constant_Ls, Observation_Ys] = creatingConstantVector(SatStateOutlier,Pu);
[Computed_Y] = ValidatingObservationData(Observation_Y, xState, Constant_L);
[ClockDisturbance] = creatingClockDisturbance(xStateOutlier(:,[4,8]));
[ClockDisturbanceWrong] = [xStateOutlier(:,4)-xState(:,4) xStateOutlier(:,8)-xState(:,8)];

fake_Observation_Y = Observation_Y-Constant_L;
fake_Observation_Ys = Observation_Ys-Constant_Ls;

%% Plot Pseudorange Data
SatelliteDisplay =[3,6];
rho = [gnssMeas.PrM(1:400,SatelliteDisplay) gnssMeas.PrrMps(1:400,SatelliteDisplay)];
rho_s = [gnssMeas.PrMOutlier(1:400,SatelliteDisplay) gnssMeas.PrrMpsOutlier(1:400,SatelliteDisplay)];
% PseudorangeData = NaNCorrection(PseudorangeData);
% SpoofedPseudorangeData = NaNCorrection(SpoofedPseudorangeData);

% h1 = figure;
% subplot(2,1,1)
% [colors] = PlotPseudoranges(gnssMeas,prFileName,PseudorangeData,SpoofedPseudorangeData);
% subplot(2,1,2)
% PlotPseudorangeRatesOutlier(gnssMeas,prFileName,PseudorangeData,SpoofedPseudorangeData);



%% Compute GPS PVT Solution with EKF
[X_EKF] = EKFonBiasModel(SatState,xState,WP,WR);
[Xs_EKF]= EKFonBiasModel(SatStateOutlier,xStateOutlier,WPOutlier,WROutlier);

%% Compute GPS PVT Solution with Luenberger Observer
[XL, XLhat] = L_observer(SatStateOutlier,Pu);

%% Compute GPS PVT Solution with TSARM
% normTSARM = KSTotalVar(gnssMeas,SatState,xState,WP,WR,SatStateOutlier,xStateOutlier,WPOutlier,WROutlier);


%% Plot EKF and Luenberger Observer Output
figure
subplot(2,1,1)
plot(1:length(X_EKF),X_EKF(:,1),'--b','LineWidth',3)
hold on;
plot(1:length(Xs_EKF), Xs_EKF(:,1), 'r','LineWidth',2.5)
hold on;
plot(1:length(XLhat), XLhat(:,1),':k','LineWidth',3)
ylim([-200000 200000])
xlim([0 410])
lgd1 = legend('$x_\mathrm{GT}$','$x_\mathrm{EKF}$','$x_\mathrm{Luen}$','interpreter','latex');
lgd1.FontSize = 16;
ylabel('Bias(m)','interpreter','latex','FontSize',16)
set(gca, 'FontSize', 12);
xlabel({'Time(s)';'(a)'})
grid on

subplot(2,1,2)
plot(1:length(X_EKF),X_EKF(:,2),'--b','LineWidth',3)
hold on;
plot(1:length(Xs_EKF), Xs_EKF(:,2),'r','LineWidth',2.5)
hold on;
plot(1:length(XLhat), XLhat(:,2), ':k','LineWidth',3)
ylim([-1000 1000])
xlim([0 410])
lgd2 = legend('$x_\mathrm{GT}$','$x_\mathrm{EKF}$','$x_\mathrm{Luen}$','interpreter','latex');
lgd2.FontSize = 16;
ylabel('Drift(m/s)','interpreter','latex','FontSize',16)
set(gca, 'FontSize', 14);
xlabel({'Time(s)';'(b)'})
grid on

%% Compute GPS PVT Solution with Horatio Observer
% [Augmented, Y_computed] = H_observer(xStateOutlier,SatStateOutlier,ClockDisturbance);
[Xhat, Dhat, Yhat] = H_observer_modified(Observation_Ys, Constant_Ls);
% [XCleanHat,Dhatwrong] = correctingXState(Xhat',Dhat');

%% Plot the Output Data
PlotEstimatedData(X_EKF, Xhat, ClockDisturbanceWrong, Dhat);

KK = size(Xhat,1);
for i = 1:1:KK
    normEKF(i) = norm(X_EKF(i,:)-Xs_EKF(i,:),2);
    normLuenberger(i) = norm(X_EKF(i,:)-XLhat(i,:),2);
    normRSE(i) = norm(X_EKF(i,:)-Xhat(i,:),2);
end
%% Calculating RMSE Value
% error_Total = 0;

RMSE_EKF = norm(abs(X_EKF-Xs_EKF),2)/KK;
RMSE_Luenberger = norm(abs(X_EKF-XLhat),2)/KK;
RMSE_RSE = norm(abs(X_EKF-Xhat),2)/KK
% for kk = 1:KK
%     error = (abs(Xhat(kk,1))-abs(X_EKF(kk,1))).^2;
%     error_Total = error_Total + error;
% end
% RMSE = sqrt(error_Total)/KK
