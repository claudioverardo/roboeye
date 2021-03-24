%test_servoangles

npoints=150;

start=[90 84 99 95 90 73];

finish=[90 90 90 90 90 73];

Qroba=zeros(npoints,6);

for i=1:6
    Qroba(:,i)=linspace(start(i),finish(i),npoints);
end

print_for_arduino(Qroba,4);