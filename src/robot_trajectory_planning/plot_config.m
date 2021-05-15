function jointpos = plot_config(Q, braccio_params,disp_kpts,plot_dual)
% PLOT_CONFIG Given a input trajectory in joints space (model convention), 
% plot the position and orientation of the end effector for each point of 
% the trajectory. Moreover, plot the final robot configuration.
%
%   [jointpos, Aloc_out] = PLOT_CONFIG(Q, braccio_params)
%
%   Input arguments:
%   ------------------
%   Q:                  NxQNUM-1 array, trajectory in joints space
%   braccio_params:     1xQNUM-1 array, real parameters of the Braccio robot,
%                       cf. direct_kin(...)
%   disp_kpts:          vector with indexes of keypoints to display
%   plot_dual:          boolean, if true display with dashed line even the
%                       dual solution (the other sol if IK)
%
%   Output arguments:
%   ------------------
%   jointpos:           (QNUM-1)x3 array, final 3D position of joints
%
% See also PLOT_CONFIG_ROB
  
  % if nargin <= 1
  %     braccio_params=[71 125 125 195 0];
  % end
  
  if nargin <=2 
      disp_kpts = [];
  end
  if nargin <=3 
      plot_dual = false;
  end

  npoints=length(Q(:,1));
  lungax=30;

  xax=zeros(npoints,3);
  yax=zeros(npoints,3);
  zax=zeros(npoints,3);

  XX=zeros(npoints,6);
  XX(:,1)=Q(:,1);

  %compute EF's reference frame axes
  for i=1:npoints
    
    Qcorr=Q(i,:)+[0 90 0 -90 0]; %convert angles to DH convenction
    
    Aloc=direct_kin(Qcorr,5,braccio_params);
    Rloc=Aloc([1 2 3],[1 2 3]);
    XX(i,[2 3 4])=Aloc([1 2 3],4);
    
    xax(i,:)=Rloc*[1; 0; 0]*lungax;
    yax(i,:)=Rloc*[0; 1; 0]*lungax;
    zax(i,:)=Rloc*[0; 0; 1]*lungax;
    
    
  end
  
  % Compute dual solution (corrected) for final config if needed
  if plot_dual == true
      Qcorr_dual = dualsol(Q(end,:))+[0 90 0 -90 0];
  end

  jointpos=zeros(6,3);
  jointpos_dual=zeros(6,3);
  
  %compute joints' positions
  for i=1:5
    %Afin=direct_kin(Q(1,[2 3 4 5 6]),i);
    Afin=direct_kin(Qcorr,i,braccio_params);
    jointpos(i+1,:)=Afin([1 2 3],4)';
    if plot_dual == true
         Afin_dual = direct_kin(Qcorr_dual,i,braccio_params);
         jointpos_dual(i+1,:)=Afin_dual([1 2 3],4)';
    end
  end

    fig=figure;
    set(gca,'DataAspectRatio',[1 1 1]);
    hold on
    
    if isempty(disp_kpts) == false
        scatter3(XX(disp_kpts,2), XX(disp_kpts,3), XX(disp_kpts,4),250,'xk','linewidth',2);
        for j=1:length(disp_kpts)
            text(XX(disp_kpts(j),2), XX(disp_kpts(j),3), XX(disp_kpts(j),4),['   Keypoint ' ...
            num2str(j)],'HorizontalAlignment','left','FontSize',16,'Color','black');
        end
    end


    quiver3(XX(:,2), XX(:,3), XX(:,4), zax(:,1), zax(:,2), zax(:,3),'b');
    quiver3(XX(:,2), XX(:,3), XX(:,4), xax(:,1), xax(:,2), xax(:,3),'r');
    quiver3(XX(:,2), XX(:,3), XX(:,4), yax(:,1), yax(:,2), yax(:,3),'g');
    
    %global RF
    quiver3(0, 0, 0, lungax*1.5, 0, 0,'r','linewidth',3);
    quiver3(0, 0, 0, 0, lungax*1.5, 0,'g','linewidth',3);
    quiver3(0, 0, 0, 0, 0, lungax*1.5,'b','linewidth',3);
    
    pbaspect([1 1 1])
    
    plot3(jointpos(:,1),jointpos(:,2),jointpos(:,3),'k','LineWidth',2);
    
    if plot_dual == true
        % Plot dual sol
        plot3(jointpos_dual(:,1),jointpos_dual(:,2),jointpos_dual(:,3),'--k','LineWidth',2);
        scatter3(jointpos_dual(:,1),jointpos_dual(:,2),jointpos_dual(:,3),'d','k');
    else
        % Plot starting position
        plot3([0 0]',[0 0]',[0 sum(braccio_params([1:4]))]','--k');
    end
    
    scatter3(jointpos(:,1),jointpos(:,2),jointpos(:,3),'k');
    hold off
    xlabel('x');
    ylabel('y');
    zlabel('z');
    title('Trajectory in model convention');

    %Aloc_out=Aloc;
    
end