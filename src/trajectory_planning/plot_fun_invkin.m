
function plot_fun_invkin(qplot,q1s,q2s,q1s_min,q1s_max,q2s_min,q2s_max)
    %function that plot %inv_kin_prob square residual
    
    % function takes a joint position qtot and plot the function 
    % varying joints n. q1s and n. q2s 
    % within the intervals [q1s_min,q1s_max] [q2s_min,q2s_max]

    %%% DEF VALUES %%%

    %qplot=[0.0000  -11.9499  -80.6664 -101.2703   -0.0000];
    % q1s =2;
    % q2s=3;
    % q1s_min=-150;
    % q1s_max=150;
    % q2s_min=-150;
    % q2s_max=150;

    [X,Y] = meshgrid(linspace(q1s_min,q1s_max,100),linspace(q2s_min,q2s_max,100));
    Z=zeros(size(X));
    for i=1:length(X(1,:))
        for j=1:length(X(:,1))
            qplot(q1s)=X(i,j);
            qplot(q2s)=Y(i,j);
            vecsol=inv_kin_prob(qplot);
            Z(i,j)=sqrt(vecsol*vecsol');
        end
    end
    figure
    s=surf(X,Y,Z);
    s.EdgeColor = 'none';
    xlabel('x')
    ylabel('y')

end