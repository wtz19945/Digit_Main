close all; clear; clc
% System Parameters
Tstep = 0.4; % step time is 0.4, fixed for now
dt = 0.1; % MPC sample Time, 
Npred = 4; % Number of steps
z0 = .9; % walking height
g = 9.81; % gravity term;
Nodes = Npred * round(Tstep / dt) + 1;

% Add ros path
addpath('/home/orl/catkin_ws/devel/share');

% Specify the path to your custom ROS bag file
filename_mpc = "../data/Sim_Data/mpc_sim_test_2024_09_12_15_00_26.bag";
filename_main = "../data/Sim_Data/sim_test_2024_09_12_15_00_26.bag";

% Create a rosbag object
bag_mpc = rosbag(filename_mpc);
bag_main = rosbag(filename_main);

% Display information about the bag file
% bagInfo = rosbag('info', filename);

% Display the topics available in the bag file
disp('Topics in the MPC bag file:');
disp(bag_mpc.AvailableTopics);

disp('Topics in the Main Controller bag file:');
disp(bag_main.AvailableTopics);

% Read message
MaintopicName = '/digit_state';
main_msgs = readMessages(select(bag_main, 'Topic', MaintopicName), 'DataFormat','struct');
MPCtopicName = '/mpc_info';
mpc_msgs = readMessages(select(bag_mpc, 'Topic', MPCtopicName), 'DataFormat','struct');
% Display the number of messages read
main_msgs{2}
mpc_msgs{2}

% Animate the MPC data
fig_name = 'mpc_animation.gif'; % Name of the output GIF file
delayTime = 0.0001;                      % Delay time between frames in seconds
loopCount = Inf;                      % Number of times the GIF should loop (Inf for infinite)

theta = 0:0.1:2*pi;
x_cir = cos(theta);
y_cir = sin(theta);

figure(1);
for k = 1:numel(mpc_msgs) - 20
    % Get the mpc data
    MPC_sol = mpc_msgs{k}.MpcSolution;
    pel_pos = mpc_msgs{k}.MpcPelPos;
    f_init = mpc_msgs{k}.FInit;
    obs_pos = mpc_msgs{k}.ObsInfo;

    x_offset = pel_pos(1);
    y_offset = pel_pos(2);

    x_sol = MPC_sol(1:2:Nodes * 2);
    y_sol = MPC_sol(3*Nodes + Npred + 1:2:5*Nodes + Npred);

    dPx = MPC_sol(3*Nodes + 1:3*Nodes + Npred);
    dPy = MPC_sol(6*Nodes + Npred + 1:6*Nodes + Npred + Npred);

    actual_foot_x = [f_init(1);f_init(1) + cumsum(dPx)] + x_offset;
    actual_foot_y = [f_init(2);f_init(2) + cumsum(dPy)] + y_offset;


    % Plot the data
    plot(x_offset + x_sol, y_offset + y_sol, 'LineWidth', 2);
    hold on
    plot(actual_foot_x(1),actual_foot_y(1),'o','MarkerSize',5,    'MarkerEdgeColor','b',...
        'MarkerFaceColor','g')
    plot(actual_foot_x(2),actual_foot_y(2),'o','MarkerSize',5,    'MarkerEdgeColor','b',...
        'MarkerFaceColor','g')
    plot(actual_foot_x(3),actual_foot_y(3),'o','MarkerSize',5,    'MarkerEdgeColor','b',...
        'MarkerFaceColor','g')
    plot(actual_foot_x(4),actual_foot_y(4),'o','MarkerSize',5,    'MarkerEdgeColor','b',...
        'MarkerFaceColor','g')
    plot(x_offset + obs_pos(5) + x_cir * 0.14, y_offset + obs_pos(6) + y_cir * 0.14)
    hold off
    xlim([-4 4])
    ylim([-1 1])
    frame = getframe(1);
    im = frame2im(frame);
    [imind,cm] = rgb2ind(im,256);
    
    % Write to the GIF file
    if k == 1
        % For the first frame, create the GIF file
        imwrite(imind, cm, fig_name, 'gif', 'Loopcount', loopCount, 'DelayTime', delayTime);
    else
        % For subsequent frames, append to the GIF file
        imwrite(imind, cm, fig_name, 'gif', 'WriteMode', 'append', 'DelayTime', delayTime);
    end
end
% pel_vel_x_cur = zeros(size(msgs,1));
% pel_vel_x_des = zeros(size(msgs,1));
% 
% foot_pos_x_cur = zeros(size(msgs,1));
% foot_pos_x_des = zeros(size(msgs,1));
% 
% foot_pos_y_cur = zeros(size(msgs,1));
% foot_pos_y_des = zeros(size(msgs,1));
% 
% for i = 1:numel(msgs)
%     Pel_Vel = msgs{i}.PelVel;
%     Pel_Vel_Ref = msgs{i}.PelVelDes;
% 
%     Foot_Pos = msgs{i}.LeftToePos;
%     Foot_Pos_Ref = msgs{i}.LeftToePosRef;
% 
%     pel_vel_x_cur(i) = Pel_Vel(1);
%     pel_vel_x_des(i) = Pel_Vel_Ref(1);
% 
%     foot_pos_x_cur(i) = Foot_Pos(1);
%     foot_pos_x_des(i) = Foot_Pos_Ref(1) + 0.08;
% 
%     foot_pos_y_cur(i) = Foot_Pos(2);
%     foot_pos_y_des(i) = Foot_Pos_Ref(2);
% end
% 
% t = 0.001 * (1:length(pel_vel_x_des));
% 
% start_time = 2000;
% end_time   = 12000;
% 
% figure
% plot(t(start_time:end_time), foot_pos_x_cur(start_time:end_time))
% hold on
% plot(t(start_time:end_time), foot_pos_x_des(start_time:end_time))
% title("foot tracking x")
% 
% figure
% plot(t(start_time:end_time), foot_pos_y_cur(start_time:end_time))
% hold on
% plot(t(start_time:end_time), foot_pos_y_des(start_time:end_time))
% title("foot tracking y")
% 
% figure
% plot(t(start_time:end_time), pel_vel_x_cur(start_time:end_time))
% hold on
% plot(t(start_time:end_time), pel_vel_x_des(start_time:end_time))
% title("base tracking")