%fft test

close all;

x=sit_ns_raw(:,1)

Fs = 1;
T=1/Fs;
L=length(x);
t=(0:L-1)*T;

subplot(3,1,1);
plot(Fs*t(1:L),x(1:L))
title('original');

y=filter(Hlp2,x);
subplot(3,1,2);
plot(y(8:length(y)));
title('filtered');

NFFT = 2^nextpow2(L);
y=fft(x,NFFT)/L;
f=Fs/2*linspace(0,1,NFFT/2+1);

subplot(3,1,3);
plot(f,2*abs(y(1:NFFT/2+1)))
title('fft');
