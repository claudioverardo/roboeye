% test_touchdown
num=20;
r=linspace(150,360,num);
maxEX=zeros(num,6);
for i=1:num
    [qex,cvecex,maxEX(i,:)]=touchdown(r(i),0,0);
end