function [Y] = ValidatingObservationData(prMdata, X, Cl)
   numOfSat = 4;
   C = [ones(numOfSat,1), zeros(numOfSat,1); ...
        zeros(numOfSat,1), ones(numOfSat,1)];
   Y = zeros(numOfSat*2,1);
   for i = 1:400
       x = [X(i,4);X(i,8)];
       cl = Cl(:,i);
       y = C*x+cl;
       
       Y = [Y y];
   end
end