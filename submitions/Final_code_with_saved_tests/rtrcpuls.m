function [y, t] = rtrcpuls(a,tau,fs,span)

t_positive = eps:(1/fs):span*tau;  
t = [-fliplr(t_positive(2:end)) t_positive];
tpi = pi/tau; amtpi = tpi*(1-a); aptpi = tpi*(1 + a);
ac = 4*a/tau; at = 16*a^2/tau^2;
y = (sin(amtpi*t) + (ac*t).*cos(aptpi*t))./(tpi*t.*(1-at*t.^2));
y = y/norm(y);
