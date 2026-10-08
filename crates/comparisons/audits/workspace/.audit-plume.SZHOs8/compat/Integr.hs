----------------------------------------------------------------------
--   Much of the code 
--
--
----------------------------------------------------------------------


module Integr (realintegrate, fnmax, fnmin) where

import SBinStream
import SBinFloat
import SBinDec -- Testing
import X_Repn -- Testing
import DyDigit
import DyStream
import DyFloat
import Convert







------------------------------------------------------------------------
-- The following code is a modification of work by Reinhold Heckmann. --
------------------------------------------------------------------------

forall  :: (SBinStream -> Bool) -> Bool
forall predicate  =  predicate (notwitness predicate)

notwitness :: (SBinStream -> Bool) -> SBinStream
-- both consider only  DReal's  made from 0 and 1
-- forall checks whether the given predicate holds for all such DReal's
-- notwitness tries to find a counterexample,
-- i.e. a sequence where the predicate does not hold

notwitness pred  = if  pred notwit1  then  0 : notwitness (pred . (0:))
		                     else  notwit1
  where notwit1 = (1 : notwitness (pred . (1:)))




common :: (SBinStream -> Int) -> Maybe Int
-- The common result digit for all arguments made up of 0's and 1'
-- (if it exists)

common f = if forall (\r -> (f r) == d)  then Just d else Nothing
  where d = (f sbZero)


-----------------------------------------------------------------------

sfnmax1, sfnmax2 :: (SBinStream -> SBinStream) -> SBinStream
-- sfnmax f over the interval [0,1]

sfnmax1 f  =  case  common (head . f)  of
            Just d  ->  d : sfnmax1 (tail . f)
            Nothing    ->  sbMax (sfnmax1 (f . (0:))) (sfnmax1 (f . (1:)))





sfnmin1, sfnmin2 :: (SBinStream -> SBinStream) -> SBinStream
-- sfnmin f over the interval [0,1]

sfnmin1 f  =  case  common (head . f)  of
            Just d  ->  d : sfnmin1 (tail . f)
            Nothing    ->  sbMin (sfnmin1 (f . (0:))) (sfnmin1 (f . (1:)))

-----------------------------------------------------------------------

sfnmaxd :: (SBinStream -> Int) -> Int

sfnmaxd g = case  common g  of
            Just d     ->  d 
            Nothing    ->  max (sfnmaxd (g . (0:))) (sfnmaxd (g . (1:)))


sfnmax2 f = (d : sfnmax2 ((p d) . f))
  where d = sfnmaxd (head . f)




sfnmind :: (SBinStream -> Int) -> Int

sfnmind g = case  common g  of
            Just d  ->  d 
            Nothing    ->  min (sfnmaxd (g . (0:))) (sfnmind (g . (1:)))


sfnmin2 f = (d : sfnmin2 ((p d) . f))
  where d = sfnmind (head . f) 

----------------------------------------------------------------------

intq1, intq2 :: (SBinStream -> SBinStream) -> DyStream
int1, int2  :: (SBinStream -> SBinStream) -> SBinStream
-- The integral  int f  over the interval [0,1]

intq1 f  =  case  common (head . f)  of
             Just d  ->  (dyd (toInteger d,0): intq1 (tail . f))
             Nothing -> dysAv (intq1 (f . (0:))) (intq1 (f . (1:)))

int1  =  dysToSbs . intq1

----------------------------------------------------------------------

meas :: (SBinStream -> Int) -> DyDigit

meas g = case common g  of
             Just 1    -> dyd_One
             Just 0    -> dyd_Zero
             Just (-1) -> dyd_minusOne
             Nothing   -> dydAv (meas (g . (0:))) (meas (g . (1:)))

expand :: (SBinStream -> SBinStream) -> [SBinStream -> Int]

expand f =
  (\r -> head (f r)) : (expand (tail . f))

intq2 f = (map meas (expand f))

int2  =  dysToSbs . intq2

----------------------------------------------------------------------
-- End of work by Reinhold Heckmann 
----------------------------------------------------------------------



square r = sbMul r r
cube r = sbMul r (square r)
halve x = (0:x)

--------------------




-----------------------------------------------------

-- Integration on the whole real line stuff

nextwitness :: (SBinStream -> Bool) -> SBinStream -> SBinStream
nextwitness pred (d:w) =
	if (pred(d:w')) then (d:w')
	else if (d==0) then nextwitness pred sbHalf
	else sbminusOne
	where w' = nextwitness (\v -> pred (d:v)) w

findbiggest :: (SBinStream -> Integer) -> SBinStream -> Integer -> Integer
findbiggest f w n =
	let {w' = nextwitness (\v -> f v > n) w;
	     n' = f w'}
	in  if (n' <= n) then n else findbiggest f w' n'

biggest :: (SBinStream -> Integer) -> Integer
biggest f = findbiggest f sbZero (f sbZero)



inormalize :: SBinFloat -> Integer -> SBinStream
inormalize (e,m) n = if (e == n) then m 
		     else (0:inormalize (e,m) (n-1))


realintegrate' :: (SBinStream -> SBinFloat) -> SBinFloat
realintegrate' f = (e, int2 (\v -> inormalize (f v) e))
	where e = biggest (\v -> sbfExp (f v))



realintegrate :: (SBinFloat -> SBinFloat) -> SBinFloat -> SBinFloat 
		 -> SBinFloat
realintegrate f a b =
	       sbfMul bma 
		      (realintegrate' (\m -> f (sbfAdd a (sbfMul bma (0,m)))))
        where bma = sbfSub b a



------------------------------------------------------------

-- Max/Min over whole real line:



fnmax' :: (SBinStream -> SBinFloat) -> SBinFloat
fnmax' f = (e, sfnmax1 (\v -> inormalize (f v) e))
	where e = biggest (\v -> sbfExp (f v))



fnmax :: (SBinFloat -> SBinFloat) -> SBinFloat -> SBinFloat 
		 -> SBinFloat
fnmax f a b = (fnmax' (\m -> f (sbfAdd a (sbfMul bma (0,m)))))
        where bma = sbfSub b a



fnmin' :: (SBinStream -> SBinFloat) -> SBinFloat
fnmin' f = (e, sfnmin1 (\v -> inormalize (f v) e))
	where e = biggest (\v -> sbfExp (f v))



fnmin :: (SBinFloat -> SBinFloat) -> SBinFloat -> SBinFloat 
		 -> SBinFloat
fnmin f a b = (fnmin' (\m -> f (sbfAdd a (sbfMul bma (0,m)))))
        where bma = sbfSub b a

