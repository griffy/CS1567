
weold=0;
leftDeltaX = turnLeftGetWheelencoderRight_we_global(:,1)
rightDeltaX = turnLeftGetWheelencoderRight_we_global(:,2)
rearDeltaX = turnLeftGetWheelencoderRight_we_global(:,3)
currX=0;
currY=0;

sumX=sum(leftDeltaX)
sumY=sum(rightDeltaX)
sumTheta=sum(rearDeltaX)

radius=29/2.0;

for i = 1:length(turnLeftGetWheelencoderRight_we_global)

w1=leftDeltaX(i)
w2=rightDeltaX(i)
w3=rearDeltaX(i)

lengthArc=(-w1+w2+w3)/4.0
thetaNew=(lengthArc/(radius));

dy=((w2*sind(30) + w1*sind(-30))/2)*sind(weold)
dx=((w2*cosd(30) + w1*cosd(-30))/2)*cosd(weold)


currX=currX+dx
currY=currY+dy
weold=weold+thetaNew

end