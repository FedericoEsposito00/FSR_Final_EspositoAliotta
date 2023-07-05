%% Allocation matrix

z_disp = -0.037;
l = 0.215;
b = deg2rad(20);

% W_des = [-24; 26; 9; -10; -16; 15];

gamma = 0.016;
ct = 8.54858e-6;
cq = ct*gamma;

xi0 = [l*cosd(-30); l*sind(-30); z_disp];
xi1 = [l*cosd(-90); l*sind(-90); z_disp];
xi2 = [l*cosd(-150); l*sind(-150); z_disp];
xi3 = [l*cosd(150); l*sind(150); z_disp];
xi4 = [l*cosd(90); l*sind(90); z_disp];
xi5 = [l*cosd(30); l*sind(30); z_disp];

R0 = Rz(-30)*Rx(-160);

R1 = Rz(-90)*Rx(160);

R2 = Rz(-150)*Rx(-160);

R3 = Rz(150)*Rx(160);

R4 = Rz(90)*Rx(-160);

R5 = Rz(30)*Rx(160);

z = [0;0;1];

M = [-gamma*R0*z+skew(xi0)*R0*z, gamma*R1*z+skew(xi1)*R1*z, -gamma*R2*z+skew(xi2)*R2*z, gamma*R3*z+skew(xi3)*R3*z, -gamma*R4*z+skew(xi4)*R4*z, gamma*R5*z+skew(xi5)*R5*z;
    R0*z, R1*z, R2*z, R3*z, R4*z, R5*z];

M_bar = [M(4:6, :); M(1:3, :)];

% lambda = M_bar\W_des;
% omega2 = lambda/ct;
% motor_velocity = sqrt(abs(omega2)).*sign(omega2);

function S = skew(v)
    if(numel(v)~= 1)
        S= [0 -v(3) v(2); 
            v(3) 0 -v(1);
            -v(2) v(1) 0];
    else
        S= zeros(3);
    end
end

function S = Rz(a) % In degrees
    S = [cosd(a) -sind(a) 0;
        sind(a) cosd(a) 0;
        0 0 1];
end

function S = Rx(a) % In degrees
    S = [1 0 0;
        0 cosd(a) -sind(a);
        0 sind(a) cosd(a)];
end