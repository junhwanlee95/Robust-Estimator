function [Cl,Y] = creatingConstantVector(SatStateOutlier,Pu)
    numOfSat = 4;
    elapsedtime = 400;
    Cl = [zeros(2*numOfSat,1)];
    Y = [zeros(2*numOfSat,1)];
    
    for i = 1:elapsedtime             
            SatPos=SatStateOutlier{i}(1:numOfSat,4:6);
            SatVel=SatStateOutlier{i}(1:numOfSat,8:10);
            SatBias=SatStateOutlier{i}(1:numOfSat,7)*GpsConstants.LIGHTSPEED;
            SatDrift=SatStateOutlier{i}(1:numOfSat,11)*GpsConstants.LIGHTSPEED;

            for ii=1:size(SatPos,1)

                Const(ii,1)= norm(SatPos(ii,:)-Pu,2)-SatBias(ii);

            end
            for ii=1:size(SatVel,1)

                Const(ii+size(SatPos,1),1)= dot(SatVel(ii,:), SatPos(ii,:)-Pu)/norm(Pu-SatPos(ii,:),2)-SatDrift(ii);

            end
    pre_y = SatStateOutlier{i}(1:numOfSat,2:3);
    y = reshape(pre_y, [numOfSat*2,1]);
    
    Cl=[Cl Const];
    Y = [Y y];
    end
end
