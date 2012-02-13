data=turnLeftGetWheelencoderLeft_we_global;
nsData=turnLeftGetWheelencoderLeft_ns_global(:,3)

weold=0;
leftDeltaX = data(:,1)
rightDeltaX = data(:,2)
rearDeltaX = data(:,3)


currX=0;
currY=0;

sumX=sum(leftDeltaX)
sumY=sum(rightDeltaX)
sumTheta=sum(rearDeltaX)

circumferenceOfWheel=23.9; %in centimeters 3 inch wheel
radius=29/2.0;

ticksperRev=103;

ticksPerCm=ticksperRev/(circumferenceOfWheel)
ticksPerCm=4;

for i = 1:length(data)

w1=leftDeltaX(i)/ticksPerCm
w2=rightDeltaX(i)/ticksPerCm
w3=rearDeltaX(i)/ticksPerCm

lengthArc=-(w1-w2+w3)
thetaNew=(lengthArc/(pi*radius));
thetaNew=((-w3))/(radius)

dy=((w2*sind(30) + w1*sind(-30))/2)*sind(weold)
dx=((w2*cosd(30) + w1*cosd(-30))/2)*cosd(weold)


currX=currX+dx
currY=currY+dy
weold=weold+thetaNew

end


changeinNS=nsData(length(nsData))-nsData(1)

ticksPerCm

differenceFromNS=radtodeg(changeinNS-weold)