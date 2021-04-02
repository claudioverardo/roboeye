function done=print_for_arduino(Q,numtraj)
  %PRINT ON A TXT FILE TRAJECTORY ARRAY TO PASTE INTO ARDUINO SCRIPT
  
  [npoints,L]=size(Q);
  
  directory='';
  
  file = fopen(strcat(directory,sprintf('Arduino_trajectory_%d.txt',numtraj)),'w');
  
  if L==6
      for i=1:npoints
         printpart=round(Q(i,:));
         fprintf(file,'{%d, %d, %d, %d, %d, %d},\n',printpart);
      end
  else
      for i=1:npoints
         printpart=round(Q(i,[2 3 4 5 6 7]));
         fprintf(file,'{%d, %d, %d, %d, %d, %d},\n',printpart);
      end
  end
  
  fclose(file);
  
  done=1;
  
end
