function [Q_def,error_flag,singularity,Q_sing] = parabolic_traj(p1,p2,z_ap,roll_in,npoints,braccio_params,grasp,offset,VERBOSE)
    % FUNCTION THAT COMPUTE PARABOLIC TRAJECTORY FROM P1 TO P2 WITH APEX AT
    % Z_AP
    
    %box coordinates: [90 250 120];

    % z_ap-auto margin 
    margin_z=100;
    Q_sing=[];

    if nargin <= 8
        VERBOSE=0;
    end
    
    if nargin <= 7
        offset=0;
    end
    
    if nargin <= 6
        grasp=73;
    end
    
    if nargin <= 5 || strcmp(braccio_params,'def')
        braccio_params=[71 125 125 195 0];
    end
    
    if nargin <= 4
        npoints=8;
    end

    error_flag=0;

    Q=zeros(npoints+1,6);
    ef=zeros(npoints+1,1);
    singflag=false(npoints,1);

    theta1 = atan2(p1(2),p1(1));
    theta2 = atan2(p2(2),p2(1));

    r1=norm(p1([1 2]));
    r2=norm(p2([1 2]));
    rm=(r1+r2)/2;
    
    if strcmp(z_ap,'auto')
        z_ap_max = sqrt((braccio_params([2 3 4])*[1 1 1]')^2-rm^2)+braccio_params(1);
        z_ap=z_ap_max-margin_z;
    end

    z1=p1(3);
    z2=p2(3);

    param_parab=tr_params_parab(theta1,theta2,z1,z2,z_ap);
    param_line=tr_params_line(theta1,theta2,r1,r2);

    theta_discr=linspace(theta1,theta2,npoints+1);

    z_discr=param_parab'*[theta_discr.^2; theta_discr; ones(size(theta_discr))];
    r_discr=param_line'*[theta_discr; ones(size(theta_discr))];

    x_discr=r_discr.*cos(theta_discr);
    y_discr=r_discr.*sin(theta_discr);
    
    if VERBOSE >0
        fig=figure;
        arc_plot=linspace(0,pi);
        hold on;
        scatter3(x_discr,y_discr,z_discr);
        plot3(r1*sin(arc_plot),r1*cos(arc_plot),zeros(size(arc_plot)));
        plot3(r2*sin(arc_plot),r2*cos(arc_plot),zeros(size(arc_plot)));
        xlabel('x');
        ylabel('y');
        zlabel('z');
        hold off
    end

    roll_vec=linspace(roll_in,90,npoints+1);
    
    i=1;
    [Q(i,:), ef(i)]=gothere(braccio_params,x_discr(i),y_discr(i),z_discr(i),roll_vec(i),grasp,offset,[]);
    for i=2:npoints+1
        [Q(i,:), ef(1)]=gothere(braccio_params,x_discr(i),y_discr(i),z_discr(i),roll_vec(i),grasp,offset,Q(i-1,:));
    end
    
    %%% check singularities
    for i=1:length(Q(:,1))-1
        [singflag(i),sub_flag,sub_Q]=check_sing(Q(i,[1:1:5]),Q(i+1,[1:1:5]));
        Q_sing=[Q_sing; sub_Q(find(sub_flag),:)];
    end
    
    singularity=any(singflag == true(size(singflag)));

    if (any(ef == true(size(ef)))) || any(z_discr < zeros(size(z_discr))) || singularity
        error_flag=1;
    end
    
    if VERBOSE >0
        for i=1:length(Q(:,1))
            jp=plot_config_rob(Q(i,:),braccio_params);
        end
    end
    disp(singflag)
    
    %truncate first line
    Q_def=Q([2:1:end],:);
end

function params = tr_params_parab(theta1,theta2,z1,z2,z_ap)
    thetam=(theta1+theta2)/2;
    
    A=[theta1^2 theta1 1;...
       theta2^2 theta2 1;...
       thetam^2 thetam 1];
    b=[z1 z2 z_ap]';

    params=A\b;
   
end

function params = tr_params_line(theta1,theta2,r1,r2)

      A=[theta1 1;...
         theta2 1];
     
      b=[r1 r2]';
      
      params=A\b;
end

