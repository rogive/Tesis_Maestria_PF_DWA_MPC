%% 
mapp1 = occupancyMap(obstaculos);
% map2 = occupancyMap(10,10,2);
% obstacles = [4 10; 3 5; 7 7];
% setOccupancy(map,obstacles,ones(length(obstacles),1))
% inflate(map,0.25)
show(mapp1)

maxrange = 500;
angles = [pi/4,-pi/4,0,-pi/8];
vehiclePose = [Pos0(1),Pos0(2),pi/2];
intsectionPts = rayIntersection(mapp1,vehiclePose,angles,maxrange,0.7)

hold on
plot(intsectionPts(:,1),intsectionPts(:,2),'*r') % Intersection points
plot(Pos0(1),Pos0(2),'ob') % Vehicle pose
for i = 1:3
    plot([Pos0(1),intsectionPts(i,1)],...
        [Pos0(2),intsectionPts(i,2)],'-b') % Plot intersecting rays
end
plot([Pos0(1),Pos0(1)-6*sin(angles(4))],...
    [Pos0(2),Pos0(2)+6*cos(angles(4))],'-b') % No intersection ray

legend('Collision Points','Vehicle Position','Rays','Location','SouthEast')