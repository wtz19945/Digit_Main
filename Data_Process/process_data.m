close all; clear; clc

% Add ros path
addpath('/home/orl/catkin_ws/devel/share');

% Specify the path to your custom ROS bag file
filename_mpc = "../data/Sim_Data/mpc_sim_test_2024_09_09_15_44_52.bag";
filename_main = "../data/Sim_Data/sim_test_2024_09_09_15_44_52.bag";

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