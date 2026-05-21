clear, clc, tic
disp("START")

% DEFINE SIM TYPE
sim_type = 4;
%{
    
    satellite with control = 1
    satellite (not controlled) with arm controlled = 2
    satellite (controlled) with arm and no arm control = 3
    satellite (controlled) with arm controlled = 4
    satellite with no control = 0 or num >= 5

%}

% Time Parameters
startTime = 0; % seconds
endTime = 120; 

% Satellite Control Parameters
kp = 0.005; % higher = faster response, but more overshoot
kd = 0.005; % higher = less oscillation, but slower to reach target

% Reference Angular Velocity (wf)
wf = [0; 0; 0]; 

% Target Euler Angles
eul = [0 0 90];

qc = eul2quat(deg2rad(eul), "XYZ")';
s = qc(1); 
v = qc(2:4);
zeta_qc = [ -v(1), -v(2), -v(3);
             s,    -v(3),  v(2);
             v(3),  s,    -v(1);
            -v(2),  v(1),  s   ];

% Load SPART stuff
addpath(genpath('SPART'));

[robot, robot_keys] = urdf2robot('3LINKARM_description/urdf/3LINKARM.urdf');
robot.base_type = 'floating'; 
disp(['Robot loaded with ', num2str(robot.n_q), ' joints.']);

theta = zeros(3,1); % Initial joint angles
thetaDot = zeros(3,1); % Initial joint velocities

% ARM CONTROL PARAMS
armKp = 0.001;
armKd = 0.001;
armGainVals = [armKp, armKd];

thetaGuidance = [20; -20; 30];
thetaDotGuidance = [0; 0; 0];

% RUN RBD SIMULATION INTEGRATION
vel = [0; 0; 0];
omega = [0; 0; 0];
q = [1; 0; 0; 0];
pos = [0; 0; 0];
h = [0; 0; 0];
MarmC = [0;0;0];
Farm = [0;0;0];

m = 5.32186;
I = [0.062191; 0.074922; 0.030348]; % Ixx, Iyy, Izz
gainVals = [kp 0 kd];
M = [0; 0; 0];
F = [0; 0; 0];

switch sim_type
    case 1
        initStateVec = [vel; omega; q; pos; h];
    case 2
        initStateVec = [vel; omega; q; pos; theta; thetaDot];
    case 3
        initStateVec = [vel; omega; q; pos; h; theta; thetaDot];
    case 4
        initStateVec = [vel; omega; q; pos; h; theta; thetaDot];
    otherwise
        initStateVec = [vel; omega; q; pos];
end

[t, sol] = runSimulation(sim_type, m, I, qc, gainVals, wf, zeta_qc, M, F, robot, MarmC, startTime, endTime, initStateVec, armGainVals, thetaGuidance, thetaDotGuidance);

[Tc, Marm, armTc] = reconstructVars(t, sol, qc, gainVals, wf, zeta_qc, MarmC, robot, M, F, thetaGuidance, thetaDotGuidance, armGainVals, sim_type);

plots = makePlots(t, sol, Tc, Marm, armTc, sim_type);

disp("END")
toc