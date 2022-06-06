function c = rpy2c(rpy)
%rpy2c ロールピッチヨーから方向余弦行列を算出
%   ロールピッチヨーから方向余弦行列を算出
sr = sin(rpy(1));
sp = sin(rpy(2));
sy = sin(rpy(3));

cr = cos(rpy(1));
cp = cos(rpy(2));
cy = cos(rpy(3));

c = [cp*cy          cp*sy          -sp  ;
     sr*sp*cy-cr*sy sr*sp*sy+cr*cy sr*cp;
     cr*sp*cy+sr*sy cr*sp*sy-sr*cy cr*cp];
end

