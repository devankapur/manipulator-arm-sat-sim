function [dVdt, domegadt, dqdt, dPos] = satMotion(vel, omega, q, m, I, F, M)

dVdt = (F/m) - cross(omega, vel);

I_mat = [I(1) 0 0; 0 I(2) 0; 0 0 I(3)];
domegadt = I_mat \ (M-cross(omega, I_mat*omega));

Omega_mat = [0, -omega(1), -omega(2), -omega(3);
             omega(1), 0, omega(3), -omega(2);
             omega(2), -omega(3), 0, omega(1);
             omega(3), omega(2), -omega(1), 0];
dqdt = 0.5 * Omega_mat * q;

R = quat2rotm(q');
dPos = R * vel;

end