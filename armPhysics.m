function [F, M, thetaDoubleDot] = armPhysics(Marm, vSat, qSat, omegaSat, theta, thetaDot, robot, MonArm, FonArm)
% PARAMETERS
R0 = quat2rotm(qSat');
r0 = [0.1; 0; 0];
qm = theta;
u0 = [omegaSat; vSat];
um = thetaDot;

% KINEMATICS
[RJ, RL, rJ, rL, e, g] = Kinematics(R0, r0, qm, robot);
[Bij, Bi0, P0, pm] = DiffKinematics(R0, r0, rL, e, g, robot);
[t0, tm] = Velocities(Bij, Bi0, P0, pm, u0, um, robot);
[J0n, Jmn] = Jacob(rL(1:3, end), r0, rL, P0, pm, robot.n_links_joints, robot);

% DYNAMICS
[I0, Im] = I_I(R0, RL, robot);
[M0_tilde, Mm_tilde] = MCB(I0, Im, Bij, Bi0, robot);
[H0, H0m, Hm] = GIM(M0_tilde, Mm_tilde, Bij, Bi0, P0, pm, robot);
[C0, C0m, Cm0, Cm] = CIM(t0, tm, I0, Im, M0_tilde, Mm_tilde, Bij, Bi0, P0, pm, robot);

% ZERO GRAVITY
g = 0;
wF0 = zeros(6,1);
wFm = zeros(6, robot.n_links_joints);
for i=1:robot.n_links_joints
    wFm(6,i) = -robot.links(i).mass*g;
end

% FORWARD DYNAMICS
tauq0 = [MonArm; FonArm];
tauqm = Marm;
[u0dot_FD, umdot_FD] = FD(tauq0, tauqm, wF0, wFm, t0, tm, P0, pm, I0, Im, Bij, Bi0, u0, um, robot);

%{
% INVERSE DYNAMICS
umdot = zeros(6,1);
[taum_floating, u0dot_floating] = Floating_ID(wF0, wFm, Mm_tilde, H0, t0, tm, P0, pm, I0, Im, Bij, Bi0, u0, um, umdot, robot);
%}

tau0 = H0m*umdot_FD + (C0*u0+C0m*um);

M = tau0(1:3);
F = tau0(4:6);
thetaDoubleDot = umdot_FD;

end