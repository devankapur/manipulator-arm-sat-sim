function [u] = satControlPID(q, qc, omega_b, gainVals, wf, zeta_qc)

kp = -gainVals(1);
kd = -gainVals(3);

omega = omega_b - wf;
omegaTuned = kd * omega;
omegaTuned = reshape(omegaTuned, 1, []);

dq = zeta_qc' * q;
dq = reshape(dq, 1,[]);
dq1 = q' * qc;

dqTuned = kp * dq;

dq1Signed = sign(dq1);
dqProduct = dqTuned .* dq1Signed;

u = omegaTuned + dqProduct;
u = reshape(u, [], 1);

end