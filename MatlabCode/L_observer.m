function [X, Xhat] = L_observer(SatState,Pu)

    syms check_obsv;
    numOfSat = 4;
    deltat = 1;
 
    xhat = [0;-95];
    x = [100;-115];
    Xhat = xhat;
    X = randn(2,1);
    Y = zeros(numOfSat*2,1);
    Yhat = zeros(numOfSat*2,1);
    
    
     
    n = 2;
    p = 2*numOfSat;

    A = [1 deltat; 0 1];
    B = 0;
    C = [ones(numOfSat,1), zeros(numOfSat,1); ...
         zeros(numOfSat,1), ones(numOfSat,1)];
    D = 0;
    
    Ob_check = rank(obsv(A,C));
    if Ob_check == size(A',1)
        check_obsv = 1;
    else 
        check_obsv = 0;
    end
    
    L = place(A',C',[0.5 0.7])';
    
    
%     %% CVX Programming
%     cvx_begin sdp quiet
% 
%     variable P(n,n) symmetric
%     variable Y(n,p)
% 
%     [-P, A'*P-C'*Y'; P*A-Y*C, -P] <= 0
%     P >= eps*eye(n)
% 
%     cvx_end
% 
%     L(2*i-1:2*i,1:p) = P\Y;
    %% Iteration Starts 
    for i = 2:400
        Cl = [];
        Const=[];
        
        SatPos=SatState{i}(1:numOfSat,4:6);
        SatVel=SatState{i}(1:numOfSat,8:10);
        SatBias=SatState{i}(1:numOfSat,7)*GpsConstants.LIGHTSPEED;
        SatDrift=SatState{i}(1:numOfSat,11)*GpsConstants.LIGHTSPEED;

        for ii=1:size(SatPos,1)

            Const(ii,1)= norm(Pu-SatPos(ii,:),2)-SatBias(ii);

        end
        for ii=1:size(SatVel,1)

            Const(ii+size(SatPos,1),1)= dot(SatVel(ii,:), SatPos(ii,:)-Pu)/norm(Pu-SatPos(ii,:),2)-SatDrift(ii);

        end

        Cl=[Const];
        
        % Construct x_hat
%         if i > 30 && i < 100
%             di = 500;
%         elseif i >= 100
%             di = 500 + (100*i-100);
%         else 
%             di = 0;
%         end
% 
%         d = [di;di];
        
        pre_y = SatState{i}(1:numOfSat,2:3);
        y = reshape(pre_y, [p,1]);
        x = A*x;
        
        yhat = C*xhat + Cl;
        xhat = A*xhat + L*(y-yhat);
        
        X = [X, x];
        Y = [Y, y];
        Xhat = [Xhat, xhat];
        Yhat = [Yhat, yhat];
        clear Cl;
        clear Const;
    end
    Xhat = Xhat';
    X = X';
    Y = Y';
    Yhat = Yhat';
    
%     
%     figure
%     subplot(2,1,1)
%     plot(1:400, X(:,1), 1:400, Xhat(:,1));
%     legend('Clock Bias generated from physical state ','Luenberger Estimate of Clock Bias')
%     grid on;
%     subplot(2,1,2)
%     plot(1:400, X(:,2), 1:400, Xhat(:,2));
%     legend('Clock Drift generated from physical state ','Luenberger Estimate of Clock Drift')
%     grid on;
end
