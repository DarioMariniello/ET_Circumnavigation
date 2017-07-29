%% PLOT COUNTERCLOCKWISE ANGLES
figure(1)
%title('Counterclocwise angles')
plot(time_datab1,beta1,'b')
hold on
plot(time_datab2,beta2,'r')
hold on
plot(time_datab3,beta3,'g')
hold on
plot(time_datab4,beta4,'k')
grid on
xlabel('time [s]')
legend('\beta_1(t)','\beta_2(t)','\beta_3(t)','\beta_4(t)');

%% PLOT DISTANCES FROM THE TARGET
% figure(2)
% title('Distances from the target')
% plot(time_datad1,distance1,'b')
% hold on
% plot(time_datad2,distance2,'r')
% hold on
% plot(time_datad3,distance3,'y')
% hold on
% plot(time_datad4,distance4,'k')
% grid on
% xlabel('time [ns]')
% legend('D_1(t)','D_2(t)','D_3(t)','D_4(t)');
