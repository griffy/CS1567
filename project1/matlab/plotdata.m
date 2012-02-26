clc;
display('start');

t=1:length(holdstill_middle_raw);

subplot(2,1,1);
plot(t,holdstill_middle_raw);

subplot(2,1,2);
plot(t,holdstill_middle_filtered);

% %change the name to the name of the file
% holdstill_middle_rawaccum=driveholdstillwheels_origin;
% 
% 
% for i=2:709
%     holdstill_middle_rawaccum(i,1)=holdstill_middle_rawaccum(i-1,1)+holdstill_middle_rawaccum(i,1);
%     holdstill_middle_rawaccum(i,2)=holdstill_middle_rawaccum(i-1,2)+holdstill_middle_rawaccum(i,2);
% end
% 
% subplot(2,1,2);
% plot(t,holdstill_middle_rawaccum,'m*');