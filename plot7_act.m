close all
% figure
% subplot(3,1,1)
% hold on
% mov=abs(act(1).movement)*180/pi;
% plot(mov,act(1).M_aero(1,:),'k-.')
% plot(mov,act(1).M_g(1,:),'k--')
% plot(mov,act(1).M(1,:),'k-')
% xlabel('\theta_{rot} (deg)')
% ylabel('M_x (N.m)')
% 
% 
% subplot(3,1,2)
% hold on
% mov=abs(act(1).movement)*180/pi;
% plot(mov,act(1).M_aero(2,:),'k-.')
% plot(mov,act(1).M_g(2,:),'k--')
% plot(mov,act(1).M(2,:),'k-')
% xlabel('\theta_{rot} (deg)')
% ylabel('M_y (N.m)')
% 
% 
% subplot(3,1,3)
% hold on
% mov=abs(act(1).movement)*180/pi;
% plot(mov,act(1).M_aero(3,:),'k-.')
% plot(mov,act(1).M_g(3,:),'k--')
% plot(mov,act(1).M(3,:),'k-')
% xlabel('\theta_{rot} (deg)')
% ylabel('M_z (N.m)')
% legend('M_{aero}','M_{g}','M_{total}')

% figure
% subplot(3,1,1)
% hold on
% mov=abs(act(1).movement);
% plot(mov,act(1).F_aero(1,:),'k-.')
% plot(mov,act(1).F_g(1,:),'k--')
% plot(mov,act(1).F(1,:),'k-')
% xlabel('x (m)')
% ylabel('F_x (N)')
% 
% 
% subplot(3,1,2)
% hold on
% mov=abs(act(1).movement);
% plot(mov,act(1).F_aero(2,:),'k-.')
% plot(mov,act(1).F_g(2,:),'k--')
% plot(mov,act(1).F(2,:),'k-')
% xlabel('x (m)')
% ylabel('F_y (N)')
% 
% 
% subplot(3,1,3)
% hold on
% mov=abs(act(1).movement);
% plot(mov,act(1).F_aero(3,:),'k-.')
% plot(mov,act(1).F_g(3,:),'k--')
% plot(mov,act(1).F(3,:),'k-')
% xlabel('x (m)')
% ylabel('F_z (N)')
% legend('F_{aero}','F_{g}','F_{total}')

figure
hold on
mov=abs(act(1).movement*180/pi);
plot(mov,act(1).M_scalar(1,:),'k-.')
xlabel('\theta_{rot} (deg)')
ylabel('M (N.m)')

