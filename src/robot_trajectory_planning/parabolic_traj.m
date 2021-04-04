function [Q,error_flag] = parabolic_traj(p1,p2,z_ap,roll_in,npoints,braccio_params,grasp,offset,varargin)
    % FUNCTION THAT COMPUTE PARABOLIC TRAJECTORY FROM P1 TO P2 WITH APEX AT
    % Z_AP
    
    %box coordinates: [90 250 120];
    
    default_verbose = 0;
    
    % Input parser
    p = inputParser;
    addParameter(p, 'verbose', default_verbose, @(x) x>=0);
    parse(p, varargin{:});
    
    % Parse function parameters
    VERBOSE = p.Results.verbose;

    if nargin <= 7
        offset=0;
    end
    
    if nargin <= 6
        grasp=73;
    end
    
    if nargin <= 5
        braccio_params=[71 125 125 195 0];
    end
    
    if nargin <= 4
        npoints=8;
    end

    error_flag=0;

    Q=zeros(npoints,6);
    ef=zeros(6,1);

    theta1 = atan2(p1(2),p1(1));
    theta2 = atan2(p2(2),p2(1));

    r1=norm(p1([1 2]));
    r2=norm(p2([1 2]));

    z1=p1(3);
    z2=p2(3);

    param_parab=tr_params_parab(theta1,theta2,z1,z2,z_ap);
    param_line=tr_params_line(theta1,theta2,r1,r2);

    theta_discr=linspace(theta1,theta2,npoints+1);

    z_discr=param_parab'*[theta_discr.^2; theta_discr; ones(size(theta_discr))];
    r_discr=param_line'*[theta_discr; ones(size(theta_discr))];

    x_discr=r_discr.*cos(theta_discr);
    y_discr=r_discr.*sin(theta_discr);
    
    if VERBOSE > 0
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

    for i=2:npoints+1
        [Q(i-1,:), ef(i-1)]=gothere(braccio_params,x_discr(i),y_discr(i),z_discr(i),roll_vec(i),grasp,offset);
    end

    if (any(ef == true(size(ef)))) || any(z_discr < zeros(size(z_discr)))
        error_flag=1;
    end
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

