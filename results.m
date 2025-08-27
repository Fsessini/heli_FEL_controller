clear; clc;
run('parameters.m')
run('FEL_RBFNN.m')
cm = 1/2.54; % cm-->in

%% comparison between linear and non linear model
% run the linear model for attitude step setpoint
% with and w/o the neural network
mdl = 'lin_att_model';
load_system(mdl)
att_sp = 1; % step input
runtime = '10';
NN_state = 0;
simOut = sim(mdl, 'StopTime', runtime);  
figure(1);
title('step input response from linear model'); 
xlabel('time'); 
ylabel('attitude angles [deg]'); 
ylim([-5,18])
grid on;
hold on;
h1 = plot(simOut.time(:),squeeze(simOut.euler_sp(1,:,:)),'DisplayName', 'phi_{sp}'); %phi_sp
h2 = plot(simOut.time(:),squeeze(simOut.euler_sp(2,:,:)),'DisplayName', 'theta_{sp}'); %theta_sp
h3 = plot(simOut.time(:),squeeze(simOut.euler_sp(3,:,:)),'DisplayName', 'psi_{sp}'); %psi_sp
h4 = plot(simOut.time(:), squeeze(simOut.euler(1,:,:)), 'DisplayName', '\phi'); % phi
h5 = plot(simOut.time(:), squeeze(simOut.euler(2,:,:)), 'DisplayName', '\theta'); % theta
h6 = plot(simOut.time(:), squeeze(simOut.euler(3,:,:)), 'DisplayName', '\psi'); % psi
color_phi = get(h4, 'Color');
color_theta = get(h5, 'Color');
color_psi = get(h6, 'Color');

NN_state = 1;
simOut = sim(mdl, 'StopTime', runtime);  
h7 = plot(simOut.time(:), squeeze(simOut.euler(1,:,:)), '--', 'Color', color_phi, 'HandleVisibility', 'off'); % phi NN
h8 = plot(simOut.time(:), squeeze(simOut.euler(2,:,:)), '--', 'Color', color_theta, 'HandleVisibility', 'off'); % theta NN
h9 = plot(simOut.time(:), squeeze(simOut.euler(3,:,:)), '--', 'Color', color_psi, 'HandleVisibility', 'off'); % psi NN

h_dummy = plot(nan, nan, '--k', 'DisplayName', 'NN On'); % Dashed black line in legend
legend([h1, h2, h3, h4, h5, h6, h_dummy], ...
       {'\phi_{sp}', '\theta_{sp}', '\psi_{sp}', ...
        '\phi', '\theta', '\psi', 'NN On'});
hold off
clear simOut h1 h2 h3 h4 h5 h6 h7 h8 h9 h_dummy

% run the non-linear model for attitude step setpoint, no disturbances
% with and w/o the neural network
mdl = 'non_lin_dyn_model';
load_system(mdl)
att_sp = 1; % step input
vel_sp = 1; % hover hold
wind = 0;
runtime = '10';
NN_state = 0;
simOut = sim(mdl, 'StopTime', runtime);  
figure(2);
title('step input response from non-linear model'); 
xlabel('time'); 
ylabel('attitude angles [deg]'); 
ylim([-5,18])
grid on;
hold on;
h1 = plot(simOut.time(:),squeeze(simOut.euler_sp(1,:,:)),'DisplayName', 'phi_{sp}'); %phi_sp
h2 = plot(simOut.time(:),squeeze(simOut.euler_sp(2,:,:)),'DisplayName', 'theta_{sp}'); %theta_sp
h3 = plot(simOut.time(:),squeeze(simOut.euler_sp(3,:,:)),'DisplayName', 'psi_{sp}'); %psi_sp
h4 = plot(simOut.time(:), squeeze(simOut.euler(1,:,:)), 'DisplayName', '\phi'); % phi
h5 = plot(simOut.time(:), squeeze(simOut.euler(2,:,:)), 'DisplayName', '\theta'); % theta
h6 = plot(simOut.time(:), squeeze(simOut.euler(3,:,:)), 'DisplayName', '\psi'); % psi
color_phi = get(h4, 'Color');
color_theta = get(h5, 'Color');
color_psi = get(h6, 'Color');

NN_state = 1;
simOut = sim(mdl, 'StopTime', runtime);  
h7 = plot(simOut.time(:), squeeze(simOut.euler(1,:,:)), '--', 'Color', color_phi, 'HandleVisibility', 'off'); % phi NN
h8 = plot(simOut.time(:), squeeze(simOut.euler(2,:,:)), '--', 'Color', color_theta, 'HandleVisibility', 'off'); % theta NN
h9 = plot(simOut.time(:), squeeze(simOut.euler(3,:,:)), '--', 'Color', color_psi, 'HandleVisibility', 'off'); % psi NN

h_dummy = plot(nan, nan, '--k', 'DisplayName', 'NN On'); % Dashed black line in legend
legend([h1, h2, h3, h4, h5, h6, h_dummy], ...
       {'\phi_{sp}', '\theta_{sp}', '\psi_{sp}', ...
        '\phi', '\theta', '\psi', 'NN On'});
hold off
clear simOut h1 h2 h3 h4 h5 h6 h7 h8 h9 h_dummy


%% test against disturbances, attitude setpoint
% run the non-linear model for attitude sinusoidal setpoint with disturbances
% with and w/o the neural network
mdl = 'non_lin_dyn_model';
load_system(mdl)
att_sp = 2; % sine input
vel_sp = 1; % hover hold
wind = 1;
runtime = '35';
NN_state = 0;
simOut = sim(mdl, 'StopTime', runtime);  
figure(3);
title('sinusoidal attitude setpoint following under wind shear'); 
xlabel('time'); 
ylabel('attitude angles [deg]'); 
%ylim([-5,18])
grid on;
hold on;
h1 = plot(simOut.time(:),squeeze(simOut.euler_sp(1,:,:)),'DisplayName', 'phi_{sp}'); %phi_sp
h2 = plot(simOut.time(:),squeeze(simOut.euler_sp(2,:,:)),'DisplayName', 'theta_{sp}'); %theta_sp
h3 = plot(simOut.time(:),squeeze(simOut.euler_sp(3,:,:)),'DisplayName', 'psi_{sp}'); %psi_sp
h4 = plot(simOut.time(:), squeeze(simOut.euler(1,:,:)), 'DisplayName', '\phi'); % phi
h5 = plot(simOut.time(:), squeeze(simOut.euler(2,:,:)), 'DisplayName', '\theta'); % theta
h6 = plot(simOut.time(:), squeeze(simOut.euler(3,:,:)), 'DisplayName', '\psi'); % psi
color_phi = get(h4, 'Color');
color_theta = get(h5, 'Color');
color_psi = get(h6, 'Color');

NN_state = 1;
simOut = sim(mdl, 'StopTime', runtime);  
h7 = plot(simOut.time(:), squeeze(simOut.euler(1,:,:)), '--', 'Color', color_phi, 'HandleVisibility', 'off'); % phi NN
h8 = plot(simOut.time(:), squeeze(simOut.euler(2,:,:)), '--', 'Color', color_theta, 'HandleVisibility', 'off'); % theta NN
h9 = plot(simOut.time(:), squeeze(simOut.euler(3,:,:)), '--', 'Color', color_psi, 'HandleVisibility', 'off'); % psi NN

h_dummy = plot(nan, nan, '--k', 'DisplayName', 'NN On'); % Dashed black line in legend
legend([h1, h2, h3, h4, h5, h6, h_dummy], ...
       {'\phi_{sp}', '\theta_{sp}', '\psi_{sp}', ...
        '\phi', '\theta', '\psi', 'NN On'});

% plot weights evolution
figure(4);
title('NN weights evolution in attitude following'); 
xlabel('time'); 
ylabel('weights'); 
ylim([-0.2,0.2])
grid on;
hold on;
for i=1:length(squeeze(simOut.NN_weights(:,:,1)))
    plot(simOut.time(:),squeeze(simOut.NN_weights(i,:,:)))
end
hold off
clear simOut h1 h2 h3 h4 h5 h6 h7 h8 h9 h_dummy


%% test against disturbances, trajectory tracking
% run the non-linear model to track an ascending turn trajectory with disturbances
% with and w/o the neural network
mdl = 'non_lin_dyn_model';
load_system(mdl)
att_sp = 0; % outer loop setpoint
vel_sp = 0; % trajectory follow
wind = 1;
runtime = '100';
NN_state = 0;
simOut = sim(mdl, 'StopTime', runtime);  

figure(5) % trajectory
view(3);
title('trajectory following under wind shear'); 
xlabel('x[m]'); 
ylabel('y[m]'); 
zlabel('z[m]'); 
grid on;
hold on;
g1 = plot3(simOut.X_sp(:,1)-Xe_0(1),-(simOut.X_sp(:,2)-Xe_0(2)),-simOut.X_sp(:,3));
g2 = plot3(simOut.X_e(:,1)-Xe_0(1),-(simOut.X_e(:,2)-Xe_0(2)),-simOut.X_e(:,3));
%color_traj = get(g2, 'Color');

NN_state = 1;
simOut = sim(mdl, 'StopTime', runtime);  
%g3 = plot3(simOut.X_e(:,1)-Xe_0(1),-(simOut.X_e(:,2)-Xe_0(2)),-simOut.X_e(:,3),'--','color',color_traj,'HandleVisibility', 'off');
g3 = plot3(simOut.X_e(:,1)-Xe_0(1),-(simOut.X_e(:,2)-Xe_0(2)),-simOut.X_e(:,3),'color','#77AC30');
%g_dummy = plot3(nan, nan, nan, '--k', 'DisplayName', 'NN On'); % Dashed black line in legend
%legend([g1,g2, g_dummy], ...
%       {'setpoint','real','NN On'});
legend([g1,g2,g3], ...
      {'setpoint','real','NN On'});

% plot weights evolution
figure(6);
title('NN weights evolution in trajectory following'); 
xlabel('time'); 
ylabel('weights'); 
ylim([-0.2,0.2])
grid on;
hold on;
for i=1:length(squeeze(simOut.NN_weights(:,:,1)))
    plot(simOut.time(:),squeeze(simOut.NN_weights(i,:,:)))
end

hold off
clear simOut g1 g2 %g_dummy


%% export figures
pause;
% exportgraphics(figure(1), 'step_response_linear.pdf', 'ContentType', 'vector');
% exportgraphics(figure(2), 'step_response_nonlinear.pdf', 'ContentType', 'vector');
% exportgraphics(figure(3), 'sinewave_attitude_setpoint.pdf', 'ContentType', 'vector');
% exportgraphics(figure(4), 'weights_evolution_attitude.pdf', 'ContentType', 'vector');
% exportgraphics(figure(5), 'trajectory_following.pdf', 'ContentType', 'vector');
% exportgraphics(figure(6), 'weights_evolution_trajectory.pdf', 'ContentType', 'vector');
close all;