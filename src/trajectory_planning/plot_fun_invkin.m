%%%%% PLOT INVS KIN FUNCTION %%%%%%


[X,Y] = meshgrid(linspace(-150,150,100),linspace(-150,150,100));
Z=zeros(size(X));
qplot=[0.0000  -11.9499  -80.6664 -101.2703   -0.0000];
for i=1:length(X(1,:))
    for j=1:length(X(:,1))
        qplot(2)=X(i,j);
        qplot(3)=Y(i,j);
        vecsol=inv_kin_prob(qplot);
        Z(i,j)=sqrt(vecsol*vecsol');
    end
end
figure
s=surf(X,Y,Z)
s.EdgeColor = 'none';
xlabel('x')
ylabel('y')