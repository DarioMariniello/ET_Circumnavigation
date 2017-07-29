%% DYNAMICAL NETWORK WITH STATIONARY TARGET
bag = rosbag('/home/antonio/catkin_ws/src/circumnavigation_moving_target/bagfiles/Mobile.bag');

agent1_error = select(bag, 'Time', [bag.StartTime bag.EndTime], 'Topic', '/agent1/error');
agent2_error = select(bag, 'Time', [bag.StartTime bag.EndTime], 'Topic', '/agent2/error');
agent3_error = select(bag, 'Time', [bag.StartTime bag.EndTime], 'Topic', '/agent3/error');
agent4_error = select(bag, 'Time', [bag.StartTime bag.EndTime], 'Topic', '/agent4/error');

msgse1= readMessages(agent1_error);
msgse2= readMessages(agent2_error);
msgse3= readMessages(agent3_error);
msgse4= readMessages(agent4_error);

lene1 = length(msgse1);
lene2 = length(msgse2);
lene3 = length(msgse3);
lene4 = length(msgse4);

 time_datae1 = zeros(lene1,1);
 for i=1:lene1
     time_datae1(i)= double(double(msgse1{i,1}.Header.Stamp.Sec)+double(msgse1{i,1}.Header.Stamp.Nsec)/(10^9))-double(double(msgse1{1,1}.Header.Stamp.Sec)+double(msgse1{1,1}.Header.Stamp.Nsec)/10^9);
 end
 
 time_datae2 = zeros(lene2,1);
 for i=1:lene2
     time_datae2(i)= double(double(msgse2{i,1}.Header.Stamp.Sec)+double(msgse2{i,1}.Header.Stamp.Nsec)/(10^9))-double(double(msgse1{1,1}.Header.Stamp.Sec)+double(msgse1{1,1}.Header.Stamp.Nsec)/10^9);
 end
 
time_datae3 = zeros(lene3,1);
 for i=1:lene3
     time_datae3(i)= double(double(msgse3{i,1}.Header.Stamp.Sec)+double(msgse3{i,1}.Header.Stamp.Nsec)/(10^9))-double(double(msgse1{1,1}.Header.Stamp.Sec)+double(msgse1{1,1}.Header.Stamp.Nsec)/10^9);
 end
 
 time_datae4 = zeros(lene4,1);
 for i=1:lene4
     time_datae4(i)= double(double(msgse4{i,1}.Header.Stamp.Sec)+double(msgse4{i,1}.Header.Stamp.Nsec)/(10^9))-double(double(msgse1{1,1}.Header.Stamp.Sec)+double(msgse1{1,1}.Header.Stamp.Nsec)/10^9);
 end
 
error1 = zeros(lene1,1);
for i=1:lene1
    error1(i) =double(msgse1{i,1}.Point.X);
end

error2 = zeros(lene2,1);
for i=1:lene2
    error2(i) =double(msgse2{i,1}.Point.X);
end

error3 = zeros(lene3,1);
for i=1:lene3
    error3(i) =double(msgse3{i,1}.Point.X);
end

error4 = zeros(lene4,1);
for i=1:lene4
    error4(i) =double(msgse4{i,1}.Point.X);
end
