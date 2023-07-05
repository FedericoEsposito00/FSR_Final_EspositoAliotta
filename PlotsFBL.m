%% Plots

close all

tout = out.u.time;

figure % Control Errors
tiledlayout(3,2);

nexttile
plot(tout, out.err_p)
xlabel("t [s]")
ylabel("Position error [m]")
legend('e\_x', 'e\_y', 'e\_z', 'Orientation', 'horizontal','Location','northoutside');

err_eta = reshape(out.err_eta, [length(tout), 3]);

nexttile
plot(tout, err_eta)
ylabel("Orientation error [rad]")
xlabel("t [s]")
legend('e\_phi', 'e\_theta', 'e\_psi', 'Orientation', 'horizontal','Location','northoutside');

nexttile
plot(tout(1:501), out.err_p_dot(1:501, :))
ylabel("Linear velocity error [m/s]")
xlabel("t [s]")
legend('e\_x\_dot', 'e\_y\_dot', 'e\_z\_dot', 'Orientation', 'horizontal','Location','northoutside');


err_eta_dot = reshape(out.err_eta_dot, [length(tout), 3]);

nexttile
plot(tout(1:501), err_eta_dot(1:501, :))
ylabel("Angle derivative error [rad/s]")
xlabel("t [s]")
legend('e\_phi\_dot', 'e\_theta\_dot', 'e\_psi\_dot', 'Orientation', 'horizontal','Location','northoutside');


nexttile
plot(tout(501:end), out.err_p_dot(501:end, :))
ylabel("Linear velocity error [m/s]")
xlabel("t [s]")
legend('e\_x\_dot', 'e\_y\_dot', 'e\_z\_dot', 'Orientation', 'horizontal','Location','northoutside');

nexttile
plot(tout(501:end), err_eta_dot(501:end, :))
ylabel("Angle derivative error [rad/s]")
xlabel("t [s]")
legend('e\_phi\_dot', 'e\_theta\_dot', 'e\_psi\_dot', 'Orientation', 'horizontal','Location','northoutside');
%%
figure % Control inputs 
tiledlayout(3, 1)

nexttile
plot(tout, out.uD)
ylabel("uD [N]")
xlabel("t [s]")
legend('f\_x', 'f\_y', 'f\_z', 'Orientation', 'horizontal','Location','northoutside');

nexttile
plot(tout, out.tau_b)
ylabel("tau\_b [Nm]")
xlabel("t [s]")
legend('tau\_x', 'tau\_y', 'tau\_z', 'Orientation', 'horizontal','Location','northoutside');

velocities = reshape(out.velocities, [6, length(tout)])';
nexttile
plot(tout, velocities)
ylabel("Rotor speeds [rad/s]")
xlabel("t [s]")
legend('w\_1', 'w\_2', 'w\_3', 'w\_4', 'w\_5', 'w\_6', 'Orientation', 'horizontal','Location','northoutside');

% figure % Reference angles computed by the controller
% 
% plot(tout, out.phi_d)
% hold on
% plot(tout, out.theta_d)
% ylabel("Reference angles computed by the controller [rad]")
% xlabel("t [s]")
% legend('phi\_d', 'theta\_d', 'Orientation', 'horizontal','Location','northoutside');
% hold off

figure % External wrench estimate
tiledlayout(2, 1)

nexttile
plot(tout, out.estimate(:, 1:3))
ylabel("Estimated forces [N]")
xlabel("t [s]")
legend('f\_x', 'f\_y', 'f\_z', 'Orientation', 'horizontal','Location','northoutside');

nexttile
plot(tout, out.estimate(:, 4:6))
ylabel("Estimated torques [Nm]")
xlabel("t [s]")
legend('tau\_x', 'tau\_y', 'tau\_z', 'Orientation', 'horizontal','Location','northoutside');
