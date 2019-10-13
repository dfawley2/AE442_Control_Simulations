function N = normVectorColumn(A)

N = sqrt(sum( A.^2, 1));

end