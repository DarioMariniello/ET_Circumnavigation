%% PLOT ESTIMATE ERROR NORMS
figure(1)
plot(time_datae1,error1,'b')
hold on
plot(time_datae2,error2,'r')
hold on
plot(time_datae3,error3,'g')
hold on
plot(time_datae4,error4,'k')
hold on
plot(time_datae5,error5,'m')
grid on
xlabel('time [s]')
l=legend('$\Vert \tilde{x}_1(t)$','$\Vert \tilde{x}_2(t)$','$\Vert \tilde{x}_3(t)$','$\Vert \tilde{x}_4(t)$','$\Vert \tilde{x}_5(t)$');
set(l,'Interpreter','latex')