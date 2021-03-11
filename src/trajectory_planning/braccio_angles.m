% convert angles from model convention to robot conv 
function out=braccio_angles(in)

in(4)=-in(4);

out=in+[0 90 90 90 180];


end