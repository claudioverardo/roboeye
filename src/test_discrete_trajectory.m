%%%% TEST TRAJECTORY

cd ./trajectory_planning

global A_target

%define starting position of solver
%startingpos=[0 90 -90 -180 0];
startingpos=[0 0 0 0 0];
startingpos=[89.95285    90.37269   -78.13193  -192.18695   -45.09429];
startingpos=[90.00001     -42.05283     -57.27621      11.09545      91.25148];
npoints=100;

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
X=[linspace(0,3,npoints)' ones(npoints,1)*(0) ones(npoints,1)*270 linspace(60,130,npoints)'  ones(npoints,1)*135 ones(npoints,1)*45];

grabberalgle=ones(npoints,1)*50;

Q=zeros(size(X));
Qrob=zeros(size(X)+[0 1]);
Q(:,1)=X(:,1);

%contains information about the quality of the solver's output (i.e convergence)
controlvec=zeros(length(X(:,1)),2);

tic

##for i=1:length(X(:,1))
##  
##  transl=X(i,[2 3 4]);
##  eulr=[atan2(transl(2),transl(1)) X(i,[5 6])*pi/180]; %input in radiants
##  A_target=roto_transl_mat(transl,eulr);
##  
##  [qloc, fval, info] = fsolve ("inv_kin_prob", startingpos);
##  
##  Q(i,[2 3 4 5 6])=qloc%+[0 90 0 -90 0];
##  %Q(i,[2 3 4 5 6])=qloc;
##
##  Qrob(i,:)=[braccio_angles(Q(i,[2 3 4 5 6])) 50];
##  
##  controlvec(i,:)=[sum(sum(fval.^2)) info];
## 
##  startingpos=qloc;
##end
   

for i=1:length(X(:,1))
  
  transl=X(i,[2 3 4]);
  eulr=[atan2(transl(2),transl(1)) X(i,[5 6])*pi/180]; %input in radiants
  
  [qloc, fval, info] = inverse_kin(transl,eulr,startingpos);
  
  %Q(i,[2 3 4 5 6])=qloc+[0 90 0 -90 0]; %test for different angles rf
  Q(i,[2 3 4 5 6])=qloc;

  Qrob(i,:)=[Q(i,1) braccio_angles(Q(i,[2 3 4 5 6])) grabberalgle(i)]; %adapt output to braccio's convention
  
  controlvec(i,:)=[sum(sum(fval.^2)) info];
  
  startingpos=qloc; %define starting potition for the solver used in the next step
  
end

toc

fig=plot_config(Q);

print_for_arduino(Qrob,1);


