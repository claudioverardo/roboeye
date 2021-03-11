clear

load('./data/test_trajectory');
% trajectory = uint8(rand([MAXPOINTS,QNUM])*255);
trajectory = uint8(Q_final);

MAXPOINTS = size(Q_final,1);
QNUM = size(Q_final,2);

M_TX = reshape(trajectory.',1,[]);
M_RX = zeros(size(M_TX));

fprintf('---- CONNECTION -----\n');
s = serialport('COM3',115200);
% seriallist
pause(11);
fprintf('DONE\n');

fprintf('-------- TX --------\n');
write(s, M_TX, 'uint8')
fprintf('DONE\n');
% pause(1);

fprintf('-------- RX --------\n');
M_RX = uint8(read(s,MAXPOINTS*QNUM,'uint8'));
fprintf('DONE\n');           

if isequal(M_RX,M_TX)
   fprintf('DATA OK\n'); 
else
   fprintf('DATA ERROR!\n'); 
end

delete(s);