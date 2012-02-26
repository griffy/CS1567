%fft test

close all;

x=sit_ns_raw(:,2)
x2=spin_ns_raw(:,2)
x3=move_ns_raw(:,2)

Fs = 1;
T=1/Fs;
L=length(x);
t=(0:L-1)*T;

L2=length(x2);
t2=(0:L2-1)*T;

L3=length(x3);
t3=(0:L3-1)*T;

subplot(3,3,1);
plot(Fs*t(1:L),x(1:L))
title('original data 1');

y=filter(Hlp,x)
yprime=filter(Hlp2,x)
subplot(3,3,4);
plot(Fs*t(8:L),y(8:length(y)),Fs*t(8:L),yprime(8:length(yprime)),Fs*t(1:L),x(1:L));   
title('filtered data 1');

NFFT = 2^nextpow2(L);
y=fft(x,NFFT)/L;

f=Fs/2*linspace(0,1,NFFT/2+1);

subplot(3,3,7);
plot(f,2*abs(y(1:NFFT/2+1)))
title('fft 1');


subplot(3,3,2);
plot(Fs*t2(1:L2),x2(1:L2))
title('original data 2');

y2=filter(Hlp,x2);
y2prime=filter(Hlp2,x2);
subplot(3,3,5);
plot(Fs*t(8:L2),y2(8:length(y2)),Fs*t(8:L2),y2prime(8:length(y2)),Fs*t2(1:L2),x2(1:L2));   
title('filtered data 2');

NFFT2 = 2^nextpow2(L2);
y2=fft(x2,NFFT2)/L2;
f2=Fs/2*linspace(0,1,NFFT2/2+1);

subplot(3,3,8);
plot(f2,2*abs(y2(1:NFFT2/2+1)))
title('fft 2');


subplot(3,3,3);
plot(Fs*t3(1:L3),x3(1:L3))
title('original data 3');

y3=filter(Hlp,x3);
y3prime = filter(Hlp2,x3);
subplot(3,3,6);
plot(Fs*t(8:L3), y3(8:length(y3)), Fs*t(8:L3), y3prime(8:length(y3prime)), Fs*t3(1:L3),x3(1:L3));   
title('filtered data 3');

NFFT3 = 2^nextpow2(L3);
y3=fft(x3,NFFT3)/L3;
f3=Fs/2*linspace(0,1,NFFT3/2+1);

subplot(3,3,9);
plot(f3,2*abs(y3(1:NFFT3/2+1)))
title('fft 3');
