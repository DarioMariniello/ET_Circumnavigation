%% PLOT ESTIMATE ERROR NORMS
figure(1)
plot(time_datad1,distance1,'b')
hold on
plot(time_datad2,distance2,'r')
hold on
plot(time_datad3,distance3,'g')
hold on
plot(time_datad4,distance4,'k')
grid on
xlabel('time [s]')
legend('D_1(t)','D_2(t)','D_3(t)','D_4(t)');
