function [D] = creatingClockDisturbance(x)
    N = size(x,1);
    D = [0 0];
    for i = 1:N-1
        d1 = x(i+1,1)-x(i,1)-x(i,2);
        d2 = x(i+1,2)-x(i,2);
        d = [d1 d2];
        D = [D;d]; 
    end
end