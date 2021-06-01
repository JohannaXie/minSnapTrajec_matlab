%% Parameters
global m order;
n = 2;          %number of flat outputs (y, z)
t_f = 1.5;        %final time of the simulation
t = [0 t_f];
order = 14;     %order of polynomial functions
m = 1;          % number of trajectories

% acrob
g = 9.8066;

acrob_center = [0, -2, 2];
acrob_r = 0.8;
acrob_k = 1.1;

acrob_v = acrob_k * sqrt(g*acrob_r);
acrob_T = 2*pi/acrob_k*sqrt(acrob_r/g);
acrob_omega = acrob_v/acrob_r;
acrob_accel = acrob_v*acrob_v/acrob_r;
acrob_jerk = acrob_accel*acrob_omega;

%% Keyframes
global keyframe; 
keyframe = zeros(2,m);
keyframe(:,1) = [0.0,               acrob_center(3)+1         ];     % initial position
keyframe(:,2) = [acrob_center(2),   acrob_center(3)-acrob_r ];     % transition position

