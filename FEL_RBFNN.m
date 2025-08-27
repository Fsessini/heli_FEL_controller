%% angular rate FEL controller parameters
lambda = diag([0.08,0.05,1.5]);  
k      = diag([3.7,5,10]);

%% RBF Neural Network parameters
neurons  = 15; % number of neurons
x0       = linspace(-1,1,neurons)'; % centers
sigma    = linspace(1.2,4.2,neurons)'; % widths
W0       = zeros(neurons,3); % initial value of weights
eta      = 1; % learning rate

%% switches
att_sp   = 0; % 0 for outer loop setpoint, 1 for step input, 2 for sinewave
wind     = 0; % 0 for no external wind
vel_sp   = 1; % 0 for position setpoint, 1 for hover hold
NN_state = 1; % 0 to turn off network 