module SinBorwein (module SinBorwein
                  ,module Data.CDAR.BR) where

import Data.CDAR.Approx
import Data.CDAR.BR

f0 :: (BR Approx,BR Approx,Integer)
f0 = (6-4*sqrt 2,sqrt 2-1,1)
f :: (BR Approx,BR Approx,Integer) -> (BR Approx,BR Approx,Integer)
f (a,y,k) = let t = sqrt . sqrt $ 1-y^4
                y' = (1-t)/(1+t)
                a' = a*(1+y')^4-2^(2*k+1)*y'*(1+y'+y'^2)
            in (a',y',k+1)

extract :: (BR Approx,BR Approx,Integer) -> BR Approx
extract (a,_,_) = 1/a

