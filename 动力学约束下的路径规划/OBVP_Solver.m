syms T i_px i_py i_pz i_vx i_vy i_vz f_px f_py f_pz f_vx f_vy f_vz
Tans = [-12/T^3 0 0 6/T^2 0 0;  0 -12/T^3 0 0 6/T^2 0; 0  0 -12/T^3 0 0 6/T^2; 
    6/T^2 0 0 -2/T 0 0; 0 6/T^2 0 0 -2/T 0; 0 0 6/T^2 0 0 -2/T;];

% initial state and final state
s_init = [i_vx*T+i_px; i_vy*T+i_py; i_vz*T+i_pz; i_vx; i_vy; i_vz];
% s_init = [0.2*T; 0.2*T; 0; 0.2; 0.2; 0];
s_final = [f_px; f_py; f_pz; f_vx; f_vy; f_vz];
% s_final = [10 5 5 0 0 0]';
delta_s = s_final-s_init
 
lam = Tans*delta_s

alpha1 = lam(1);
alpha2 = lam(2);
alpha3 = lam(3);
belta1 = lam(4);
belta2 = lam(5);
belta3 = lam(6);

J = T+(1/3*alpha1^2*T^3 + alpha1*belta1*T^2+belta1^2*T) + (1/3*alpha2^2*T^3 + alpha2*belta2*T^2+belta2^2*T) + (1/3*alpha3^2*T^3 + alpha3*belta3*T^2+belta3^2*T)

dJ = diff(J,T);

% a = vpasolve(dJ, T)
[a] = solve(dJ, T)
