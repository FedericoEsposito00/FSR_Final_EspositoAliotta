%% Plots

close all

tout = out.u.time;

figure % Control Errors
tiledlayout(2,2);

nexttile
plot(tout, out.err_p)
xlabel("t [s]")
ylabel("Position error [m]")
legend('e\_x', 'e\_y', 'e\_z', 'Orientation', 'horizontal','Location','northoutside');


nexttile
plot(tout, out.err_p_dot)
ylabel("Linear velocity error [m/s]")
xlabel("t [s]")
legend('e\_x\_dot', 'e\_y\_dot', 'e\_z\_dot', 'Orientation', 'horizontal','Location','northoutside');


nexttile
plot(tout, out.err_R)
ylabel("Orientation error [rad]")
xlabel("t [s]")
legend('1^{st} component', '2^{nd} component', '3^{rd} component', 'Orientation', 'horizontal','Location','northoutside');

nexttile
plot(tout, out.err_W)
ylabel("Angular velocity error [rad/s]")
xlabel("t [s]")
legend('1^{st} component', '2^{nd} component', '3^{rd} component', 'Orientation', 'horizontal','Location','northoutside');

figure % Control Inputs (equal to the saturated version)
tiledlayout(3, 1)
nexttile
plot(tout, out.uD(:, 3))
ylabel("uT [N]")
xlabel("t [s]")

nexttile
plot(tout, out.tau_b)
ylabel("tau_b [Nm]")
xlabel("t [s]")
legend('tau\_x', 'tau\_y', 'tau\_z', 'Orientation', 'horizontal','Location','northoutside');

velocities = reshape(out.velocities, [6, length(tout)])';
nexttile
plot(tout, velocities)
ylabel("Rotor speeds [rad/s]")
xlabel("t [s]")
legend('omega\_1', 'omega\_2', 'omega\_3', 'omega\_4', 'omega\_5', 'omega\_6', 'Orientation', 'horizontal','Location','northoutside');

figure % Reference angles computed by the controller

eta_b_des = reshape(out.eta_b_des, [3, length(tout)])';

plot(tout, eta_b_des(:, 1))
hold on
plot(tout, eta_b_des(:, 2))
ylabel("Reference angles computed by the controller [rad]")
xlabel("t [s]")
legend('phi\_d', 'theta\_d', 'Orientation', 'horizontal','Location','northoutside');
hold off

figure % External wrench estimate
tiledlayout(3, 1)

nexttile
plot(tout, out.estimate(:, 1:3))
ylabel("Estimated forces [N]")
xlabel("t [s]")
legend('f\_x', 'f\_y', 'f\_z', 'Orientation', 'horizontal','Location','northoutside');

nexttile
plot(tout, out.estimate(:, 4:6))
ylabel("Estimated torques (body) [Nm]")
xlabel("t [s]")
legend('tau\_bx', 'tau\_by', 'tau\_bz', 'Orientation', 'horizontal','Location','northoutside');

nexttile
plot(tout, out.tau_est_world)
ylabel("Estimated torques (world) [Nm]")
xlabel("t [s]")
legend('tau\_x', 'tau\_y', 'tau\_z', 'Orientation', 'horizontal','Location','northoutside');

