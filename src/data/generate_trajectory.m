%%%% TEST TRAJECTORY

global A_target

%define starting position of solver
%startingpos=[0 90 -90 -180 0];
startingpos=[0 0 0 0 0];
startingpos=[89.95285    90.37269   -78.13193  -192.18695   -45.09429];
startingpos=[90.00001     -42.05283     -57.27621      11.09545      91.25148];
npoints=20;
npoints_first=150;

%Posizioni end effector



diam=4;
traslazione=50;
espans=8;
var=linspace(0,1,npoints)*2*pi;


%Position end effector

%time %x %y %z %theta %psi

%test 1
%X=[4*linspace(0,10,npoints)' 4*linspace(20,30,npoints)' linspace(20,30,npoints)' linspace(10,20,npoints)' ones(npoints,1)*180 linspace(0,90,npoints)'];

%test 2 (ASTROID)
X=[linspace(0,10,npoints)' (espans*(diam*cos(var)+cos(-diam*var))+ones(1,npoints)*traslazione)' (espans*(diam*sin(var)+sin(-diam*var))+ones(1,npoints)*traslazione)' ones(npoints,1)*30' ones(npoints,1)*180 linspace(0,90,npoints)'];

%test 3
X=[linspace(0,3,npoints)' ones(npoints,1)*310 ones(npoints,1)*1 ones(npoints,1)*220+linspace(0,40,npoints)' ones(npoints,1)*(91) zeros(npoints,1)];

%test 4
X=[linspace(0,3,npoints)' linspace(120,130,npoints)' ones(npoints,1)*1 ones(npoints,1)*30 ones(npoints,1)*180 zeros(npoints,1)];

%test 5
X=[linspace(0,3,npoints)' ones(npoints,1)*(0) linspace(120,130,npoints)'+ones(npoints,1) ones(npoints,1)*30 ones(npoints,1)*180 ones(npoints,1)*45];

%test 6
X=[linspace(0,3,npoints)' ones(npoints,1)*250 ones(npoints,1)*(0) linspace(40,-6,npoints)'  ones(npoints,1)*150 linspace(0,90,npoints)'*0];

%test 7
X=[linspace(0,3,npoints)' ones(npoints,1)*250 linspace(0,40,npoints)' linspace(40,-6,npoints)'  ones(npoints,1)*150 linspace(0,90,npoints)'*0];

grabberalgle=ones(npoints,1)*72; %angle position of the grabber

Q=zeros(size(X)+[0 1]);
Qrob=zeros(size(X)+[0 1]);
%Q_first=zeros(npoints_first,7);
Qrob_first=zeros(npoints_first,7);
Q(:,1)=X(:,1);

%contains information about the quality of the solver's output (i.e convergence)
controlvec=zeros(length(X(:,1)),2);

tic   

for i=1:length(X(:,1))
  
  transl=X(i,[2 3 4]);
  eulr=[atan2(transl(2),transl(1)) X(i,[5 6])*pi/180]; %input in radiants
  
  [qloc, fval, info] = inverse_kin(transl,eulr,startingpos);
  
  %Q(i,[2 3 4 5 6])=qloc+[0 90 0 -90 0]; %test for different angles rf
  Q(i,[2 3 4 5 6])=qloc;
  
  Q(i,7)=grabberalgle(i);

  Qrob(i,:)=[Q(i,1) braccio_angles(Q(i,[2 3 4 5 6])) Q(i,7)]; %adapt output to braccio's convention
  
  controlvec(i,:)=[sum(sum(fval.^2)) info];
  
  startingpos=qloc; %define starting potition for the solver used in the next step
  
end

%%%%    COMPUTE FIRST PART TRAJECTORY

Q_first=[zeros(npoints_first,1)...            %time (does not matters)
         linspace(0,Q(1,2),npoints_first)'...  %M1
         linspace(0,Q(1,3),npoints_first)'...  %M2
         linspace(0,Q(1,4),npoints_first)'...  %M3
         linspace(0,Q(1,5),npoints_first)'...  %M4
         linspace(0,Q(1,6),npoints_first)'...  %M5
         linspace(10,Q(1,7),npoints_first)'...  %M6
         ];
     
for i=1:npoints_first
    Qrob_first(i,:)=[Q_first(i,1) braccio_angles(Q_first(i,[2 3 4 5 6])) Q_first(i,7)];
end

toc

Qtot = [Q_first; Q];

Q_final = [Qrob_first(:,[2:1:end]); Qrob(:,[2:1:end])];
Q_final = uint8(mod(round(Q_final),360));

%save('./data/test_trajectory.mat','Q_final');

jointpos=plot_config(Qtot);

print_for_arduino(Q_final,2);


