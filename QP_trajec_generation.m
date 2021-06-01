
global mu_r k_r 
mu_r = 1;        %constants that makes the integrand nondimensional
k_r =4; 

%Compute the cost function matrix, H
H = computeH(order, m, mu_r, k_r, t);

%Compute the constraint function matrix
[Aeq, beq] = computeConstraint(order, m, k_r, t, keyframe,acrob_v,acrob_accel,acrob_jerk);

% solve the problem
options = optimoptions('quadprog', 'MaxIterations',5000);   %% choose display mode, maximum iterations.
tic;
solution = quadprog(2*H, [], [],[], Aeq, beq, [], [], [], options);
toc;