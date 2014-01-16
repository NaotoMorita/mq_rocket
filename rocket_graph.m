figure(1)
if t>=parashoot_time
	plot3(log_r(1,1:round(parashoot_time/dt))',log_r(2,1:round(parashoot_time/dt))',-log_r(3,1:round(parashoot_time/dt))','r')
	hold on
	plot3(log_r(1,round(parashoot_time/dt):columns(log_r)-1)',log_r(2,round(parashoot_time/dt):columns(log_r)-1)',-log_r(3,round(parashoot_time/dt):columns(log_r)-1)')
	axis 'equal'
else
	plot3(log_r(1,1:columns(log_r))',log_r(2,1:columns(log_r))',-log_r(3,1:columns(log_r))','r')
	hold on
end

axis 'equal'
xlabel('X')
ylabel('Y')
zlabel('Z')
legend('coasting','parashoot')
hold off

figure(2)
%機体姿勢を含めた3D軌跡

hb=blen/2;
for i=1:step:columns(log_r)
	
	rocket_pose=log_dcm{i}*[blen;0;0];
	
	plot3([log_r(1,i);log_r(1,i)-rocket_pose(1)],[log_r(2,i);log_r(2,i)-rocket_pose(2)],[-log_r(3,i);-log_r(3,i)+rocket_pose(3)]);
	plot3([log_r(1,i)],[log_r(2,i)],[-log_r(3,i)],'or')
	hold on
end
axis 'equal'
xlabel('X')
ylabel('Y')
zlabel('Z')

hold off
	
figure(3)
hold on
%時間と高度
plot(log_t,-log_r(3,:),"r")
%時間と位置
plot(log_t,log_r(2,:))
plot(log_t,log_r(1,:),"g")

xlabel("t [s]")
ylabel("position [m]")
legend("atitude","y","x")
hold off

figure(4)
%時間と慣性座標上での速度
hold on
plot(log_t,-log_v(1,:),"g")
%時間と位置
plot(log_t,log_v(2,:))
plot(log_t,-log_v(3,:),"r")

xlabel("t [s]")
ylabel("velocity(inatial) [m/s]")
legend("x","y","z")
hold off

figure(5)
%時間と迎え角

