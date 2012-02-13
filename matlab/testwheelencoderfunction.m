
data=turnLeftGetWheelencoderRight_we_global;
nsData=turnLeftGetWheelencoderRight_ns_global(:,3)

weold=0;
leftDeltaX = data(:,1)
rightDeltaX = data(:,2)
rearDeltaX = data(:,3)


currX=0;
currY=0;

sumX=sum(leftDeltaX)
sumY=sum(rightDeltaX)
sumTheta=sum(rearDeltaX)

radius=29/2.0;

for i = 1:length(data)

w1=leftDeltaX(i)
w2=rightDeltaX(i)
w3=rearDeltaX(i)

lengthArc=(w1-w2+w3)/4.0
%thetaNew=(lengthArc/(radius));
%thetaNew=-((w3+w1+-w2)/3)/(pi*radius*2)
thetaNew=(w3)/(radius)

dy=((w2*sind(30) + w1*sind(-30))/2)*sind(weold)
dx=((w2*cosd(30) + w1*cosd(-30))/2)*cosd(weold)


currX=currX+dx
currY=currY+dy
weold=weold+thetaNew

end


changeinNS=nsData(length(nsData))-nsData(1)

differenceFromNS=radtodeg(changeinNS-weold)