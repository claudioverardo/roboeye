function jointpos = plot_config_rob(Q_rob, braccio_params, post_corr, home, disp_kpts, plot_dual)
% PLOT_CONFIG_ROB Given a input trajectory in joints space (robot convention),
% plot the position and orientation of the end effector for each point of 
% the trajectory. Moreover, plot the final robot configuration.
%
%   jointpos = PLOT_CONFIG_ROB(Q, braccio_params, post_corr, home, disp_kpts,
%   plot_dual)
%
%   Input arguments:
%   ------------------
%   Q:                  NxQNUM-1 array, trajectory in joints space
%   braccio_params:     1xQNUM-1 array, real parameters of the Braccio robot,
%                       cf. direct_kin(...)
%   post_corr:          1xQNUM-1 array, offsets to be applied a posteriori,
%                       cf. braccio_angles(...)
%   home:               1xQNUM array, home position of the robot
%   disp_kpts:          vector with indexes of keypoints to display
%   plot_dual:          boolean, if true display with dashed line even the
%                       dual solution (the other sol if IK)
%
%   Output arguments:
%   ------------------
%   jointpos:           (QNUM-1)x3 array, final 3D position of joints
%
% See also PLOT_CONFIG
  
  if nargin <= 4 
      disp_kpts = [];
  end
  
  if nargin <= 5 
      plot_dual = false;
  end
  
  %convert into algorithm angle convention
  %Q=Q_rob;
  %for i=1:length(Q_rob(:,1))
  %    Q(i,[1 2 3 4 5])=braccio_angles_inv(Q_rob(i,[1 2 3 4 5]),post_corr,home(1:end-1));
  %end
  Q=braccio_angles_inv(Q_rob(:,[1 2 3 4 5]),post_corr,home(1:end-1));
  
  jointpos = plot_config(Q, braccio_params, disp_kpts, plot_dual);
  title('Trajectory in robot convention');

end