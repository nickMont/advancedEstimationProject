[a,k]=size(xStore);
% for ik=1:k
%     if plotFlag==1
%         figure(1)
%         pause(.1)
%         delete(f1); delete(f2)
%         f1=scatter(xStore(1),xStore(2),'b');
%         hold on
%         f2=scatter(xStore(5),xStore(6),'r');
%         axis(axisveck)
%     end
% end

figure(1);clf;
plot(xStore(7,:),xStore(8,:),'o-r')
hold on
plot(xStore(19,:),xStore(20,:),'o-b')
hold on
p=[1 -9;0.1 -0.9]; hold on; plot(p(1,:),p(2,:),'*-y')
p=[0 -10;0 -1]; hold on; plot(p(1,:),p(2,:),'*-c')
axis([-2.5 1.5 -2.5 1.5])
legend('Quad,Pur','Quad,Eva','PM,Pur','PM,Eva')
figset

figure(2);clf;
t=(1:k-1)*0.1-0.1;
plot(t,uPhist(1,:)*10,'--b')
hold on
plot(t,uPhist(2,:)*10,':b')
hold on
plot(t,uPhist(3,:)*10,'--r')
hold on
plot(t,uPhist(4,:)*10,':r')
ylabel('Control input for each pursuer rotor (rad/s)')
xlabel('Time (s)')
figset

figure(3);clf;
t=(1:k-1)*0.1-0.1;
plot(t,uEhist(1,:)*10,'--b')
hold on
plot(t,uEhist(2,:)*10,':b')
hold on
plot(t,uEhist(3,:)*10,'--r')
hold on
plot(t,uEhist(4,:)*10,':r')
ylabel('Control input for each evader rotor (rad/s)')
xlabel('Time (s)')
figset

