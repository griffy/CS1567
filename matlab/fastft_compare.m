%fft test

close all;

x=sit_ns_raw(:,3)
x2=spin_ns_raw(:,3)

Fs = 1;
T=1/Fs;
L=length(x);
t=(0:L-1)*T;

subplot(3,2,1);
plot(Fs*t(1:L),x(1:L))
title('original data 1');

y=filter(Hlp,x);
subplot(3,2,3);
plot(y(8:length(y)));   
title('filtered data 1');

NFFT = 2^nextpow2(L);
y=fft(x,NFFT)/L;
f=Fs/2*linspace(0,1,NFFT/2+1);

subplot(3,2,5);
plot(f,2*abs(y(1:NFFT/2+1)))
title('fft 1');


subplot(3,2,2);
plot(Fs*t(1:L),x2(1:L))
title('original data 2');

y2=filter(Hlp,x2);
subplot(3,2,4);
plot(y2(8:length(y2)));   
title('filtered data 2');

NFFT = 2^nextpow2(L);
y2=fft(x2,NFFT)/L;
f2=Fs/2*linspace(0,1,NFFT/2+1);

subplot(3,2,6);
plot(f2,2*abs(y2(1:NFFT/2+1)))
title('fft 2');

