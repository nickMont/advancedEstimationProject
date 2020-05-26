%here
tt=0
thetaOffset=atan2(.1,1)
xP2d=xStore(1:2,indsamp);
xE2d=xStore(5:6,indsamp);
thetaOffset=thetaOffset+tt;
R=[cos(thetaOffset) -sin(thetaOffset);sin(thetaOffset) cos(thetaOffset)];
mx=(xP2d(2,1)-xE2d(2,1))/(xP2d(1,1)-xE2d(1,1));
xPlin_x=-5:0.3:1;
xPlin_y=xPlin_x*mx;
xElin_x=-6:0.3:0;
xElin_y=xElin_x*mx;
xP2d=R*xP2d;
xE2d=R*xE2d;

figure(1);clf;
figset
plot(xP2d(1,:),xP2d(2,:),'-*b');
hold on
plot(xE2d(1,:),xE2d(2,:),'-Or');
% hold on
% plot(xPlin_x,xPlin_y,'-.*k');
% hold on
% plot(xElin_x,xElin_y,'-.og');
axis([-6 2 -6 2])
xlabel('x-position (m)')
ylabel('y-position (m)')
legend('Pursuer trajectory, quad dynamics','Evader trajectory, quad dynamics','Pursuer trajectory, point mass','Evader trajectory, point mass');
figset