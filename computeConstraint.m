function [Aeq, beq] = computeConstraint(order, m, k_r, t, keyframe,acrob_v,acrob_accel,acrob_jerk)

n = 2;      %State number
computeMat = eye(order+1);      % Required for computation of polynomials

%% Pose constraints
A1 = zeros(2*m*n,n*(order+1)*m);
b1 = zeros(2*m*n,1);

% Start Point
start = keyframe(:,1);       % start, transition, end point

ts = zeros(1,order+1);
for j=1:order+1
    ts(j) = polyval(computeMat(j,:),t(1));
end

% update A1
for k=1:n       %  y_T1, z_T1
    a = zeros(1,n*(order+1)*m);     %1*56
    a( ((k-1)*(order+1) + 1) : ((k-1)*(order+1))+order+1 ) = ts;
    A1(k, :) = a;
end
% update b1
b1(1 : n) = start;

% End Point
trans = keyframe(:,2);       % start, transition, end point

ts = zeros(1,order+1);
for j=1:order+1
    ts(j) = polyval(computeMat(j,:),t(2));
end

% update A1
for k=1:n       %  y_T1, z_T1
    a = zeros(1,n*(order+1)*m);     %1*56
    a( ((k-1)*(order+1) + 1) : ((k-1)*(order+1))+order+1 ) = ts;
    A1(k+n, :) = a;
end
% update b1
b1(n+1 : 2*n) = trans;


%% Derivative constraints
A2 = zeros(2*m*n*k_r,n*(order+1)*m);
b2 = ones(2*m*n*k_r,1)*eps;
velocity = acrob_v;
accel = acrob_accel;
jerk = acrob_jerk;

for h=1:k_r
    % start point, all derivatives set to zero
    i=1;
    ts = zeros(1,order+1);
    for j=1:order+1
        tempCoeffs = computeMat(j,:);
        for k=1:h
            tempCoeffs = polyder(tempCoeffs);
        end
        ts(j) = polyval(tempCoeffs,t(i));
    end
    for k=1:n         % set the h-th derivatives of y/z to zeros
        a = zeros(1,n*(order+1)*m);
        a( ((k-1)*(order+1)+1) : ((k-1)*(order+1))+order+1) = ts;
        A2( (h-1)*n + k,:) = a;
        b2( (h-1)*n + k) = 0; 
    end

    % transition point, \dot y = v, \ddot z = a
    i=2;
    ts = zeros(1,order+1);
    for j=1:order+1
        tempCoeffs = computeMat(j,:);
        for k=1:h
            tempCoeffs = polyder(tempCoeffs);
        end
        ts(j) = polyval(tempCoeffs,t(i));
    end

    for k=1:n    % k=1 -->y ; k=2 -->z
        % end of trajectory i-1
        a = zeros(1,n*(order+1)*m);
        a( ((k-1)*(order+1)+1) : ((k-1)*(order+1))+order+1) = ts;
        A2(n*k_r + (h-1)*n + k,:) = a;
        b2(n*k_r + (h-1)*n + k) = 0;

        if (h==1 & k==1)        % \dot y = -v
            b2(n*k_r + (h-1)*n + k) = -velocity;
        elseif (h==2 & k==2)    % \ddot z = a
            b2(n*k_r + (h-1)*n + k) = accel;
        elseif (h==3 & k==1)    % \ddot y = j
            b2(n*k_r + (h-1)*n + k) = jerk;
        end

    end

end

%% Outputs
Aeq = [A1; A2];
beq = [b1; b2];

end