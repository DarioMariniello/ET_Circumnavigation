clear all
close all
clc
%% DYNAMICAL NETWORK WITH STATIONARY TARGET
bag = rosbag('/home/antonio/catkin_ws/src/circumnavigation_moving_target/bagfiles/Stationary.bag');
%% TOPICS
agent1_beta = select(bag, 'Time', [bag.StartTime bag.EndTime], 'Topic', '/agent1/beta');
agent2_beta = select(bag, 'Time', [bag.StartTime bag.EndTime], 'Topic', '/agent2/beta');
agent3_beta = select(bag, 'Time', [bag.StartTime bag.EndTime], 'Topic', '/agent3/beta');
agent4_beta = select(bag, 'Time', [bag.StartTime bag.EndTime], 'Topic', '/agent4/beta');
agent5_beta = select(bag, 'Time', [bag.StartTime bag.EndTime], 'Topic', '/agent5/beta');


msgsb1= readMessages(agent1_beta);
msgsb2= readMessages(agent2_beta);
msgsb3= readMessages(agent3_beta);
msgsb4= readMessages(agent4_beta);
msgsb5= readMessages(agent5_beta);
 

%% Counterclockwise angles
lenb1 = length(msgsb1);
lenb2 = length(msgsb2);
lenb3 = length(msgsb3);
lenb4 = length(msgsb4);
lenb5 = length(msgsb5);

 time_datab1 = zeros(lenb1,1);
 for i=1:lenb1
     time_datab1(i)= double(double(msgsb1{i,1}.Header.Stamp.Sec)+double(msgsb1{i,1}.Header.Stamp.Nsec)/10^9)-double(double(msgsb1{1,1}.Header.Stamp.Sec)+double(msgsb1{1,1}.Header.Stamp.Nsec)/10^9);
 end
 
 time_datab2 = zeros(lenb2,1);
 for i=1:lenb2
     time_datab2(i)= double(double(msgsb2{i,1}.Header.Stamp.Sec)+double(msgsb2{i,1}.Header.Stamp.Nsec)/10^9)-double(double(msgsb1{1,1}.Header.Stamp.Sec)+double(msgsb1{1,1}.Header.Stamp.Nsec)/10^9);
 end
 
time_datab3 = zeros(lenb3,1);
 for i=1:lenb3
     time_datab3(i)= double(double(msgsb3{i,1}.Header.Stamp.Sec)+double(msgsb3{i,1}.Header.Stamp.Nsec)/10^9)-double(double(msgsb1{1,1}.Header.Stamp.Sec)+double(msgsb1{1,1}.Header.Stamp.Nsec)/10^9);
 end
 
 time_datab4 = zeros(lenb4,1);
 for i=1:lenb4
     time_datab4(i)= double(double(msgsb4{i,1}.Header.Stamp.Sec)+double(msgsb4{i,1}.Header.Stamp.Nsec)/10^9)-double(double(msgsb1{1,1}.Header.Stamp.Sec)+double(msgsb1{1,1}.Header.Stamp.Nsec)/10^9);
 end
 
 time_datab5 = zeros(lenb5,1);
 for i=1:lenb5
     time_datab5(i)= double(double(msgsb5{i,1}.Header.Stamp.Sec)+double(msgsb5{i,1}.Header.Stamp.Nsec)/10^9)-double(double(msgsb1{1,1}.Header.Stamp.Sec)+double(msgsb1{1,1}.Header.Stamp.Nsec)/10^9);
 end 



beta1 = zeros(lenb1,1);
for i=1:lenb1
    beta1(i) =double(msgsb1{i,1}.Point.X);
end

beta2 = zeros(lenb2,1);
for i=1:lenb2
    beta2(i) =double(msgsb2{i,1}.Point.X);
end

beta3 = zeros(lenb3,1);
for i=1:lenb3
    beta3(i) =double(msgsb3{i,1}.Point.X);
end

beta4 = zeros(lenb4,1);
for i=1:lenb4
    beta4(i) =double(msgsb4{i,1}.Point.X);
end

beta5 = zeros(lenb5,1);
for i=1:lenb5
    beta5(i) =double(msgsb5{i,1}.Point.X);
end

