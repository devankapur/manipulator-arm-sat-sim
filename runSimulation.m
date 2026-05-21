function [t, sol] = runSimulation (sim_type, m, I, qc, gainVals, wf, zeta_qc, M, F, robot, MarmC, startTime, endTime, initStateVec, armGainVals, thetaGuidance, thetaDotGuidance)
tspan = [startTime, endTime];

switch sim_type
    case 1
        [t, sol] = ode45(@(t, s) simDynamics1(t, s, m, I, qc, gainVals, wf, zeta_qc, M, F), tspan, initStateVec);
    case 2
        [t, sol] = ode45(@(t, s) simDynamics2(t, s, m, I, M, F, robot, armGainVals, thetaGuidance, thetaDotGuidance), tspan, initStateVec);
    case 3
        [t, sol] = ode45(@(t,s) simDynamics3(t, s, m, I, qc, gainVals, wf, zeta_qc, M, F, robot, MarmC), tspan, initStateVec);
    case 4
        [t, sol] = ode45(@(t, s) simDynamics4(t, s, m, I, qc, gainVals, wf, zeta_qc, M, F, robot, armGainVals, thetaGuidance, thetaDotGuidance), tspan, initStateVec);
    otherwise
end

% DYNAMICS FUNCTIONS FOR EACH CASE
    function [dStateVec] = simDynamics1(t, stateVec, m, I, qc, gainVals, wf, zeta_qc, M, F)
        % UNPACK STATE VECTOR
        vel = stateVec(1:3);
        omega_b = stateVec(4:6);
        q = stateVec(7:10);
        pos = stateVec(11:13);
        h = stateVec(14:16);
        
        % SATELLITE CONTROL
        % pid controller
        [u] = satControlPID(q, qc, omega_b, gainVals, wf, zeta_qc);

        % reaction wheels
        [Tc, hDot] = reactionWheel(omega_b, u, h);

        % SATELLITE MOTION
        Mtot = M - Tc;
        Ftot = F;

        % 6 DoF block
        [dVdt, domegadt, dqdt, dPos] = satMotion(vel, omega_b, q, m, I, Ftot, Mtot);
    
        dStateVec = [dVdt; domegadt; dqdt; dPos; hDot];

        disp(t)
    end

    function [dStateVec] = simDynamics2(t, stateVec, m, I, M, F, robot, armGainVals, thetaGuidance, thetaDotGuidance)
        % UNPACK STATE VECTOR
        vel = stateVec(1:3);
        omega_b = stateVec(4:6);
        q = stateVec(7:10);
        pos = stateVec(11:13);
        theta = stateVec(14:16);
        thetaDot = stateVec(17:19);

        % ROBOT ARM
        % arm control
        [armTc] = armPIDcontrol(theta, thetaGuidance, thetaDot, thetaDotGuidance, armGainVals);
            
        MonArm = M;
        FonArm = F;
        
        % arm phyics
        [Farm, Marm, thetaDoubleDot] = armPhysics(armTc, vel, q, omega_b, theta, thetaDot, robot, MonArm, FonArm);

        % SATELLITE MOTION
        Mtot = M - Marm;
        Ftot = F - Farm;

        % 6 DoF block
        [dVdt, domegadt, dqdt, dPos] = satMotion(vel, omega_b, q, m, I, Ftot, Mtot);
    
        dStateVec = [dVdt; domegadt; dqdt; dPos; thetaDot; thetaDoubleDot];

        disp(t)

    end

    function [dStateVec] = simDynamics3(t, stateVec, m, I, qc, gainVals, wf, zeta_qc, M, F, robot, MarmC)
        % UNPACK STATE VECTOR
        vel = stateVec(1:3);
        omega_b = stateVec(4:6);
        q = stateVec(7:10);
        pos = stateVec(11:13);
        h = stateVec(14:16);
        theta = stateVec(17:19);
        thetaDot = stateVec(20:22);

        % SATELLITE CONTROL
        % pid controller
        [u] = satControlPID(q, qc, omega_b, gainVals, wf, zeta_qc);

        % reaction wheels
        [Tc, hDot] = reactionWheel(omega_b, u, h);

        % ROBOT ARM
        MonArm = M - Tc;
        FonArm = F;
        
        % arm phyics
        [Farm, Marm, thetaDoubleDot] = armPhysics(MarmC, vel, q, omega_b, theta, thetaDot, robot, MonArm, FonArm);
        % SATELLITE MOTION
        Mtot = M - Tc - Marm;
        Ftot = F - Farm;

        % 6 DoF block
        [dVdt, domegadt, dqdt, dPos] = satMotion(vel, omega_b, q, m, I, Ftot, Mtot);
    
        dStateVec = [dVdt; domegadt; dqdt; dPos; hDot; thetaDot; thetaDoubleDot];

        disp(t)
    end

    function [dStateVec] = simDynamics4(t, stateVec, m, I, qc, gainVals, wf, zeta_qc, M, F, robot, armGainVals, thetaGuidance, thetaDotGuidance)
        % UNPACK STATE VECTOR
        vel = stateVec(1:3);
        omega_b = stateVec(4:6);
        q = stateVec(7:10);
        pos = stateVec(11:13);
        h = stateVec(14:16);
        theta = stateVec(17:19);
        thetaDot = stateVec(20:22);

        % SATELLITE CONTROL
        % pid controller
        [u] = satControlPID(q, qc, omega_b, gainVals, wf, zeta_qc);

        % reaction wheels
        [Tc, hDot] = reactionWheel(omega_b, u, h);

        % ROBOT ARM
        % arm control
        [armTc] = armPIDcontrol(theta, thetaGuidance, thetaDot, thetaDotGuidance, armGainVals);

        MonArm = M - Tc;
        FonArm = F;
        
        % arm phyics
        [Farm, Marm, thetaDoubleDot] = armPhysics(armTc, vel, q, omega_b, theta, thetaDot, robot, MonArm, FonArm);

        % SATELLITE MOTION
        Mtot = M - Tc - Marm;
        Ftot = F - Farm;

        % 6 DoF block
        [dVdt, domegadt, dqdt, dPos] = satMotion(vel, omega_b, q, m, I, Ftot, Mtot);
    
        dStateVec = [dVdt; domegadt; dqdt; dPos; hDot; thetaDot; thetaDoubleDot];

        disp(t)
    end

end