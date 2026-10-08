module Data.Number.IReal.IReal where

import Data.Number.IReal.IntegerInterval
import Data.Number.IReal.UnsafeMemo
import Data.Number.IReal.Scalable
import Data.Bits

-- The type IReal, its smart constructor and selector ----------------

type Precision = Int

-- | A real number/interval is a function from required precision to an integer interval;
-- for numbers the interval is thin (has radius 1).
newtype IReal = IR (Precision -> IntegerInterval) 

-- | Smart constructor; uses (unfortunately unsafe) memoizing techniques for efficiency.
ir :: (Precision -> IntegerInterval) -> IReal
ir  = IR . unsafeMemo

-- | Selector; computes integer interval for given precision.
appr :: IReal -> Precision -> IntegerInterval
appr (IR f) = f

-- | prec n x is an interval of width at least 10^(-n) containing x.
instance VarPrec IReal where
  precB b x = seq l (seq u (IR (\p -> scale xb (p-b))))
      where xb@(I (l,u)) = appr x b


