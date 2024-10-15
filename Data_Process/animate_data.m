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
filename_mpc = "../data/Hard_Data/Forward/mpc_hardware2.bag";
filename_main = "../data/Hard_Data/Forward/hardware2.bag";

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
delayTime = 0.1;                      % Delay time between frames in seconds
loopCount = Inf;                      % Number of times the GIF should loop (Inf for infinite)

theta = 0:0.1:2*pi;
x_cir = cos(theta);
y_cir = sin(theta);

%%
Fs = 1000; % Sampling frequency (Hz)
Fc = 400;   % Cutoff frequency (Hz)

left_traj = [];
right_traj = [];
stanceleg_prev = -1;
zleft = 0;
zright = 0;
for i = 25000:75:numel(main_msgs) - 11000
    stanceleg = main_msgs{i}.StanceLeg;
    if stanceleg ~= stanceleg_prev
        zleft = main_msgs{i}.LeftToePosRef(3);
        zright = main_msgs{i}.RightToePosRef(3);
    end
    stanceleg_prev = stanceleg;
    left_pos = main_msgs{i}.LeftToePos;
    right_pos = main_msgs{i}.RightToePos;
    pel_pos = main_msgs{i}.PelPosActual;
    obs_info = main_msgs{i}.ObsInfo;
    obs_info(5:6) = obs_info(5:6) + pel_pos(1:2) + [.4;0];
    left_traj = [left_traj [left_pos(1) + pel_pos(1);left_pos(2) + pel_pos(2);left_pos(3) - zleft]];
    right_traj = [right_traj [right_pos(1) + pel_pos(1);right_pos(2) + pel_pos(2);right_pos(3) - zright]];
end


foot_trajx = [];
foot_trajy = [];
foot_trajz = [];
[sx,sy,sz] = sphere;

index = 2388;
MPC_sol = mpc_msgs{index}.MpcSolution;
pel_pos = mpc_msgs{index}.MpcPelPos;
f_init = mpc_msgs{index}.FInit;
obs_pos = mpc_msgs{index}.ObsInfo;

swing_foot_start = mpc_msgs{index}.MpcSwfCur;

dPx = MPC_sol(3*Nodes + 1:3*Nodes + Npred);
dPy = MPC_sol(6*Nodes + Npred + 1:6*Nodes + Npred + Npred);

x_offset = pel_pos(1);
y_offset = pel_pos(2);

actual_foot_x = [f_init(1);f_init(1) + cumsum(dPx)] + x_offset + 0.08;
actual_foot_y = [f_init(2);f_init(2) + cumsum(dPy)] + y_offset;

swing_foot = MPC_sol(7*Nodes + 2 * Npred + 1 : 7*Nodes + 2 * Npred + 15);
swing_foot_x = swing_foot(1:5);
swing_foot_y = swing_foot(6:10);
swing_foot_z = swing_foot(11:15);

figure
hSurface = surf(0.14* sx + x_offset + obs_info(5) , 0.14 * sy + y_offset + obs_info(6) , 0.14* sz + obs_pos(7) , 'FaceColor', 'red');
set(hSurface,'FaceColor',[0 0 1], ...
  'FaceAlpha',1,'FaceLighting','gouraud')
hold on
plot3(actual_foot_x(1),actual_foot_y(1),obs_pos(7),'o','MarkerSize', 10,    'MarkerEdgeColor','b',...
    'MarkerFaceColor','g')
plot3(actual_foot_x(2),actual_foot_y(2),obs_pos(7),'o','MarkerSize', 10,    'MarkerEdgeColor','b',...
    'MarkerFaceColor','r')
plot3(swing_foot_start(1) + x_offset + 0.08,swing_foot_start(2)+ y_offset,swing_foot_start(3),'o','MarkerSize', 10,    'MarkerEdgeColor','b',...
    'MarkerFaceColor','y')
plot3(left_traj(1,:),left_traj(2,:),left_traj(3,:))
plot3(right_traj(1,:),right_traj(2,:),right_traj(3,:))
legend('obstacle', 'stance foot', 'swing foot goal', 'swing foot current', 'left foot trajectory', 'right foot trajectory')
view(15,90)
xlim([-1 1])
ylim([-1 1])
zlim([0 .4])


%%
figure(1);
start_index = 2250;
slack_var = [];
distance = [];
x_dis = [];
y_dis = [];
pel_vel = [];

% for k = 2388 + 44
for k = start_index : 10 : start_index + 300
    % Get the mpc data

    MPC_sol = mpc_msgs{k}.MpcSolution;
    pel_pos = mpc_msgs{k}.MpcPelPos;
    f_init = mpc_msgs{k}.FInit;
    obs_pos = mpc_msgs{k}.ObsInfo;
    swing_foot_start = mpc_msgs{k}.MpcSwfCur;
    pel_vel_ref = main_msgs{10.5 * k}.PelRef;
    pel_vel = [pel_vel;pel_vel_ref(2)];
    x_offset = pel_pos(1);
    y_offset = pel_pos(2);

    x_sol = MPC_sol(1:2:Nodes * 2);
    y_sol = MPC_sol(3*Nodes + Npred + 1:2:5*Nodes + Npred);

    dPx = MPC_sol(3*Nodes + 1:3*Nodes + Npred);
    dPy = MPC_sol(6*Nodes + Npred + 1:6*Nodes + Npred + Npred);

    actual_foot_x = [f_init(1);f_init(1) + cumsum(dPx)] + x_offset;
    actual_foot_y = [f_init(2);f_init(2) + cumsum(dPy)] + y_offset;

    slack_var = [slack_var;MPC_sol(end - 18)];
    x_dis = [x_dis;MPC_sol(end - 22)];
    distance = [distance; norm(obs_pos(5:6))];
    % Plot the data
    plot(x_offset + x_sol, y_offset + y_sol, 'LineWidth', 2);
    hold on
    plot(actual_foot_x(1),actual_foot_y(1),'o','MarkerSize',5,    'MarkerEdgeColor','b',...
        'MarkerFaceColor','g')
    plot(actual_foot_x(2),actual_foot_y(2),'o','MarkerSize',5,    'MarkerEdgeColor','b',...
        'MarkerFaceColor','r')
    plot(actual_foot_x(3),actual_foot_y(3),'o','MarkerSize',5,    'MarkerEdgeColor','b',...
        'MarkerFaceColor','m')
    plot(swing_foot_start(1) + x_offset,swing_foot_start(2)+ y_offset,'o','MarkerSize',5,    'MarkerEdgeColor','b',...
        'MarkerFaceColor','y')
    plot(x_offset + obs_pos(5) + x_cir * 0.14, y_offset + obs_pos(6) + y_cir * 0.14)
    legend('mpc com traj','stance foot','swing foot goal','third step', 'swing foot current', 'obstacle')
    hold off
    xlim([-4 4])
    ylim([-1 1])
    frame = getframe(1);
    im = frame2im(frame);
    [imind,cm] = rgb2ind(im,256);
    
    % Write to the GIF file
    if k == start_index
        % For the first frame, create the GIF file
        imwrite(imind, cm, fig_name, 'gif', 'Loopcount', loopCount, 'DelayTime', delayTime);
    else
        % For subsequent frames, append to the GIF file
        imwrite(imind, cm, fig_name, 'gif', 'WriteMode', 'append', 'DelayTime', delayTime);
    end
end

figure
plot(slack_var)
hold on
plot(distance)
plot(x_dis)
plot(pel_vel)
legend('slack', 'distance to obstacle', 'travel distance prediction','velocity cmd')