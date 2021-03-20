function sigma_svd = check_svd(X)
% CHECK_SVD SVD test to check if arrays are (numerically) linearly dependent 
%
%   sigma_svd = CHECK_SVD(X)
%
%   Input arguments:
%   ------------------
%   X:          cell array of candidated linearly dependent arrays            
%
%   Output arguments:
%   ------------------
%   sigma_svd:  singolar values of the concatenated arrays
%
%   See also SVD


    X_cat = [];
    for i = 1:length(X)
        X_cat = [X_cat, vec(X{i})];
    end
    
    sigma_svd = svd(X_cat);

end