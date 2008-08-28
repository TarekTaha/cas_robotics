function [Y] = dirichlet(X,A)

total = 0 ;
for i=1:length(A)
    total = total + A(i);
end

Y= zeros(1,length(X));

for i=1:length(X)
    Y(i) = (A(i) -1)/(total - length(X));
end

end