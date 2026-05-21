function [Tc_recon, Marm_recon, armTc_recon] = reconstructVars(t, sol, qc, gainVals, wf, zeta_qc, MarmC, robot, M, F, thetaGuidance, thetaDotGuidance, armGainVals, sim_type)

switch sim_type
    case 1
        % UNPACK SOL VARS
        q = sol(:, 7:10);
        omega_b = sol(:, 4:6);
        h = sol(:, 14:16);
        
        % RECONSTRUCT SATELLITE CONTROL TORQUE (Tc)
        [Tc_recon] = recon_Tc(t, q, omega_b, h, qc, gainVals, wf, zeta_qc);

        Marm_recon = zeros(length(t), 3);
        armTc_recon = zeros(length(t), 3);
    case 2
        % UNPACK SOL VARS
        vel = sol(:, 1:3);
        omega_b = sol(:, 4:6);
        q = sol(:, 7:10);
        theta = sol(:, 14:16);
        thetaDot = sol(:, 17:19);
        
        Tc_recon = zeros(length(t), 3);

        % RECONSTRUCT ARM CONTROL TORQUE (armTc)
        [armTc_recon] = recon_armTc(theta, thetaGuidance, thetaDot, thetaDotGuidance, armGainVals);
        
        % RECONSTRUCT ARM TORQUE ON BASE
        [Marm_recon] = recon_Marm(vel, theta, thetaDot, M, F, Tc_recon, MarmC, q, omega_b, robot);
    case 3
        % UNPACK SOL VARS
        vel = sol(:, 1:3);
        omega_b = sol(:, 4:6);
        q = sol(:, 7:10);
        h = sol(:, 14:16);
        theta = sol(:, 17:19);
        thetaDot = sol(:, 20:22);

        % RECONSTRUCT SATELLITE CONTROL TORQUE (Tc)
        [Tc_recon] = recon_Tc(t, q, omega_b, h, qc, gainVals, wf, zeta_qc);
        
        % RECONSTRUCT ARM TORQUE ON SAT (Marm)
        [Marm_recon] = recon_Marm(vel, theta, thetaDot, M, F, Tc_recon, MarmC, q, omega_b, robot);

        armTc_recon = zeros(length(t), 3);
    case 4
        % UNPACK SOL VARS
        vel = sol(:, 1:3);
        omega_b = sol(:, 4:6);
        q = sol(:, 7:10);
        h = sol(:, 14:16);
        theta = sol(:, 17:19);
        thetaDot = sol(:, 20:22);

        % RECONSTRUCT SATELLITE CONTROL TORQUE (Tc)
        [Tc_recon] = recon_Tc(t, q, omega_b, h, qc, gainVals, wf, zeta_qc);

        % RECONSTRUCT ARM TORQUE ON SAT (Marm)
        [Marm_recon] = recon_Marm(vel, theta, thetaDot, M, F, Tc_recon, MarmC, q, omega_b, robot);

        % RECONSTRUCT ARM CONTROL TORQUE (armTc)
        [armTc_recon] = recon_armTc(theta, thetaGuidance, thetaDot, thetaDotGuidance, armGainVals);
    otherwise
end

% RECONSTRUCTION FUNCTION DECLARATIONS 
    function [Tc_recon] = recon_Tc(t, q, omega_b, h, qc, gainVals, wf, zeta_qc)
        Tc_recon = zeros(length(t), 3);
        
        for i = 1:length(t)
            q_current = q(i, :)';
            omega_b_current = omega_b(i, :)';
            h_current = h(i, :)';
        
            % pid controller
            [u] = satControlPID(q_current, qc, omega_b_current, gainVals, wf, zeta_qc);
        
            % reaction wheels
            [Tc, ~] = reactionWheel(omega_b_current, u, h_current);
            
            Tc_recon(i,:) = Tc';
        end
    end

    function [Marm_recon] = recon_Marm(vel, theta, thetaDot, M, F, Tc_recon, MarmC, q, omega_b, robot)
        Marm_recon = zeros(length(t), 3);
        
        for i = 1:length(t)
            vel_current = vel(i, :)';
            omega_b_current = omega_b(i, :)';
            q_current = q(i, :)';
            theta_current = theta(i, :)';
            thetaDot_current = thetaDot(i, :)';
        
            MonArm = M - Tc_recon(i, :)';
            FonArm = F;
        
            [~, Marm, ~] = armPhysics(MarmC, vel_current, q_current, omega_b_current, theta_current, thetaDot_current, robot, MonArm, FonArm);
            
            Marm_recon(i,:) = Marm';
        end
    end

    function [armTc_recon] = recon_armTc(theta, thetaGuidance, thetaDot, thetaDotGuidance, armGainVals)
        armTc_recon = zeros(length(t), 3);

        for i = 1:length(t)
            theta_current = theta(i, :)';
            thetaDot_current = thetaDot(i, :)';
                    
            [armTc] = armPIDcontrol(theta_current, thetaGuidance, thetaDot_current, thetaDotGuidance, armGainVals);

            armTc_recon(i,:) = armTc';
        end
    end

end
