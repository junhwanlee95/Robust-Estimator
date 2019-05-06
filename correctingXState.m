function [Xcorrect,D_Sum] = correctingXState(X,D)
    M = size(D,1);
    N = size(X,1);
    D_Sum = D(1,:);
    for i = 1:M-1
        D_Sum(i+1,2) = D_Sum(i,2)+D(i+1,2);
        D_Sum(i+1,1) = D_Sum(i,1)+D_Sum(i,2)+D(i+1,1);
    end
    Xcorrect = X(2:end,:)-D_Sum; 
end