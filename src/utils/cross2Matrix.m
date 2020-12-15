function M = cross2Matrix(x)
%CROSS2MATRIX Matrix representing the cross product with x.
%
%M = CROSS2MATRIX(x) If x is 3x1, it returns the matrix representing the 
%linear operator of the cross product with x. 
%M * y is equivalent to cross(x,y).
%If x is a 3xN matrix, with N > 1, M is a 3x3xN matrix where 
%M(:,:,i) = CROSS2MATRIX(x(:,i)).

arguments
    x {mustBe3xN(x)}
end

x = reshape(x(:),3,1,[]);

z = zeros(1,1,size(x,3));
M = [z         -x(3,1,:)  x(2,1,:);
     x(3,1,:)   z        -x(1,1,:);
    -x(2,1,:)   x(1,1,:)  z];
end

function mustBe3xN(x)
    if size(x,1) ~= 3
        eid = 'Size:notValid';
        msg = 'x must be a 3xN matrix.';
        throwAsCaller(MException(eid,msg))
    end
end

