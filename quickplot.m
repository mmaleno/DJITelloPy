% figure(100)
% for i = 1:length(STATE_ESTIMATES(1,:))
%     plot(STATE_ESTIMATES(1,1:i),STATE_ESTIMATES(2,1:i),'o')
%     hold on
%     pause(.01)
% end

xt = [0,0,1.5,1.5,0]
yt = [0,0,0,.5,0]
zt = [0,-.8,-.8,-.8,-.8]

figure(145)
plot3(STATE_ESTIMATES(1,:),STATE_ESTIMATES(2,:),STATE_ESTIMATES(3,:))
hold on
plot3(xt*100,yt*100,zt*100)
hold on
title('Estimated Position')
xlabel('x Position (m)')
ylabel('y Position (m)')
zlabel('z Position (m)')

