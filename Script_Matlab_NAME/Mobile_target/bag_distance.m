clear all
close all
clc
%% DYNAMICAL NETWORK WITH STATIONARY TARGET
bag = rosbag('/home/antonio/catkin_ws/src/circumnavigation_moving_target/bagfiles/Mobile.bag');
%% TOPICS

agent1_distance = select(bag, 'Time', [bag.StartTime bag.EndTime], 'Topic', '/agent1/distance');
agent2_distance = select(bag, 'Time', [bag.StartTime bag.EndTime], 'Topic', '/agent2/distance');
agent3_distance = select(bag, 'Time', [bag.StartTime bag.EndTime], 'Topic', '/agent3/distance');
agent4_distance = select(bag, 'Time', [bag.StartTime bag.EndTime], 'Topic', '/agent4/distance');


msgsd1= readMessages(agent1_distance);
msgsd2= readMessages(agent2_distance);
msgsd3= readMessages(agent3_distance);
msgsd4= readMessages(agent4_distance);
 




%% Distances
lend1 = length(msgsd1);
lend2 = length(msgsd2);
lend3 = length(msgsd3);
lend4 = length(msgsd4);

 time_datad1 = zeros(lend1,1);
 for i=1:lend1
     time_datad1(i)= double(double(msgsd1{i,1}.Header.Stamp.Sec)+double(msgsd1{i,1}.Header.Stamp.Nsec)/(10^9))-double(double(msgsd1{1,1}.Header.Stamp.Sec)+double(msgsd1{1,1}.Header.Stamp.Nsec)/10^9);
 end
 
 time_datad2 = zeros(lend2,1);
 for i=1:lend2
     time_datad2(i)= double(double(msgsd2{i,1}.Header.Stamp.Sec)+double(msgsd2{i,1}.Header.Stamp.Nsec)/(10^9))-double(double(msgsd1{1,1}.Header.Stamp.Sec)+double(msgsd1{1,1}.Header.Stamp.Nsec)/10^9);
 end
 
time_datad3 = zeros(lend3,1);
 for i=1:lend3
     time_datad3(i)= double(double(msgsd3{i,1}.Header.Stamp.Sec)+double(msgsd3{i,1}.Header.Stamp.Nsec)/(10^9))-double(double(msgsd1{1,1}.Header.Stamp.Sec)+double(msgsd1{1,1}.Header.Stamp.Nsec)/10^9);
 end
 
 time_datad4 = zeros(lend4,1);
 for i=1:lend4
     time_datad4(i)= double(double(msgsd4{i,1}.Header.Stamp.Sec)+double(msgsd4{i,1}.Header.Stamp.Nsec)/(10^9))-double(double(msgsd1{1,1}.Header.Stamp.Sec)+double(msgsd1{1,1}.Header.Stamp.Nsec)/10^9);
 end
 
 distance1 = zeros(lend1,1);
for i=1:lend1
    distance1(i) =double(msgsd1{i,1}.Point.X);
end

distance2 = zeros(lend2,1);
for i=1:lend2
    distance2(i) =double(msgsd2{i,1}.Point.X);
end

distance3 = zeros(lend3,1);
for i=1:lend3
    distance3(i) =double(msgsd3{i,1}.Point.X);
end

distance4 = zeros(lend4,1);
for i=1:lend4
    distance4(i) =double(msgsd4{i,1}.Point.X);
end

