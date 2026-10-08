-- Audit-only reconstruction; original CC0 tutorial remains unchanged.
module EscardoVariants where
import Escardo (I,mid,digitMul,zero,compl,addOne,trisection01,sqr)

-- Identical mul_version2 recurrence, with the missing a0 contribution restored.
fixedMul :: I -> I -> I
fixedMul (0:x) y = 0 : fixedMul x y
fixedMul x (0:y) = 0 : fixedMul x y
fixedMul (a0:0:x) (b0:0:y) = mid (0:mid (digitMul b0 x) (digitMul a0 y))
                                              ((a0*b0):0:0:fixedMul x y)
fixedMul (a0:0:x) (b0:1:y) = mid (mid (a0:0:x) (mid (digitMul b0 x) (digitMul a0 y)))
                                              ((a0*b0):0:0:fixedMul x y)
fixedMul (a0:0:x) (b0:b1:y) = mid (mid ((a0*b1):0:digitMul b1 x)
                                             (mid (digitMul b0 x) (digitMul a0 y)))
                                              ((a0*b0):0:0:fixedMul x y)
fixedMul (a0:a1:x) (b0:0:y) = mid (mid (0:0:digitMul a1 y)
                                             (mid (digitMul b0 x) (digitMul a0 y)))
                                              ((a0*b0):(a1*b0):0:fixedMul x y)
fixedMul (a0:a1:x) (b0:b1:y) = mid (mid ((a0*b1):mid (digitMul b1 x) (digitMul a1 y))
                                             (mid (digitMul b0 x) (digitMul a0 y)))
                                              ((a0*b0):(a1*b0):(a1*b1):fixedMul x y)

domainRoot :: I -> I
domainRoot y = trisection01 (\x -> mid (sqr x) (compl y))

-- Historical always-two-digit normalizer, for scoped lookahead/performance A/B.
twoDigitDivide2 :: [Int] -> I
twoDigitDivide2 (a:b:x) = let d=2*a+b in
  if d < -2 then -1:twoDigitDivide2(d+4:x)
  else if d > 2 then 1:twoDigitDivide2(d-4:x)
  else 0:twoDigitDivide2(d:x)

naiveBigMid :: [I] -> I
naiveBigMid (x:xs) = mid x (naiveBigMid xs)
