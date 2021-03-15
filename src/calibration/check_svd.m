function sigma_svd = check_svd(X)
% SVD test to check if arrays are (numerically) linearly independent 

    X_cat = [];
    for i = 1:length(X)
        X_cat = [X_cat, vec(X{i})];
    end
    
    sigma_svd = svd(X_cat);

end