function q4 = ramt2q(r1,r2,r3,t)
%ramt2q ‰ñ“]ŠpA‰ñ“]Ž²‚©‚çƒNƒI[ƒ^ƒjƒIƒ“‚ðŒvŽZ
%   ‰ñ“]ŠpA‰ñ“]Ž²‚©‚çƒNƒI[ƒ^ƒjƒIƒ“‚ðŒvŽZ
ram = [r1 r2 r3];
q4 = zeros(4,1);
% q = zeros(3,1);
if norm(ram) > 0.0000000001
    ram = ram/norm(ram);
else
    ram(1) = 1;
    ram = ram/norm(ram);
end


q4(4) = cos(t/2); 
q4(1) = ram(1)*sin(t/2);
q4(2) = ram(2)*sin(t/2);
q4(3) = ram(3)*sin(t/2);
end

