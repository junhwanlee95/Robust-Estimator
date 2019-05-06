function [Xbarhat, Dhat, Yhat] = H_observer_modified(Observation_Y, Constant_L)

    deltat = 1;
    eps = 1e-20;
    numOfSat = 4;
    elapsedtime = 400;

    n = 2;
    p = 2*numOfSat;

    A = [1 deltat; 0 1];
    B = 0;
    B2 = [1 0;0 1];
    C = [ones(numOfSat,1), zeros(numOfSat,1); ...
         zeros(numOfSat,1), ones(numOfSat,1)];
    D = [0;0];
    H = [1 0;0 1];

    %% CVX Programming
    cvx_begin sdp quiet

    variable P(n,n) symmetric
    variable Q(n,n) symmetric
    variable M(n,n) symmetric
    variable G(n,p) 

    [-P, zeros(n,n), (P*A-G*C)', (-M*B2'*C'*C)'; ...
     zeros(n,n), -Q, B2'*(P*A-G*C)', Q-B2'*C'*C*B2*M'; ...
     P*A-G*C, (P*A-G*C)*B2, -P, zeros(n,n); ...
     -M*B2'*C'*C, Q'-M*B2'*C'*C*B2, zeros(n,n), -Q] <= -eps*eye(4*n,4*n)

    P >= eps*eye(n)
    Q >= eps*eye(n)
    M >= eps*eye(n)

    cvx_end

    L = P\G;
    Gamma = Q\M;    

    %% Initializing the Estimated State Values
    
    xbarhat0 = zeros(2,1);
    xbarhat1 = zeros(2,1);
    xbarhat2 = [27;-92];
    dhat0 = zeros(2,1);
    dhat1 = zeros(2,1);
    yhat0 = zeros(2*numOfSat,1);
    yhat1 = zeros(2*numOfSat,1);

    Xbarhat = [xbarhat1 xbarhat2];
    Yhat = [yhat1];
    Dhat = [dhat1];
    Cl = Constant_L;
    Y = Observation_Y;
    
    %% iteration statrs here
    for i = 2:elapsedtime        
            yhat = C*Xbarhat(:,i) + C*Dhat(:,i-1) + Cl(:,i);
            epsilon = Y(:,i)-yhat;
            dhat = Dhat(:,i-1) + Gamma*C'*epsilon; 
            xbarhat = A*Xbarhat(:,i) + A*Dhat(:,i-1) + L*epsilon; 
            
            % Store esimated values in vector form
            Xbarhat = [Xbarhat xbarhat];
            Dhat = [Dhat dhat];
    end
    [Xbarhat,Dhat] = correctingXState(Xbarhat',Dhat');
end