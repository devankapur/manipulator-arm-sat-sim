function [Tc, hDot] = reactionWheel(omega_b, u, h)

hDot = u;

crossProduct = cross(omega_b, h);

Tc = -hDot - crossProduct; 
%{
if T > 0.001
    Tc = 0.001;
elseif T < -0.001
    Tc = -0.001;
else
    Tc = T;
end
%}
end