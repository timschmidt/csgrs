module Data.Number.IReal.FoldB where

import Data.Number.IReal.IReal
import Data.Number.IReal.Auxiliary
import Data.Number.IReal.IntegerInterval
import Data.Number.IReal.Scalable
import Data.Number.IReal.IRealOperations
import Data.List

-- Balanced fold -----------------------------------------------

-- | Balanced fold, minimizing depth of call tree. Assumes associative operator.
-- Often much more efficient than foldl/foldr when type @a@ is 'IReal' and the list is long.
foldb :: (a -> a -> a) -> a -> [a] -> a
foldb f u []        = u
foldb f u [x]       = f u x
foldb f u xs        = foldb f u (g xs)
  where g (x:y:xs)  = f x y : g xs
        g xs        = xs
-- | Balanced fold for associative operator over non-empty list.
foldb1 f (x : xs)   = foldb f x xs

-- | Balanced sum, reorganized for (much) better efficiency when type @a@ is 'IReal' and the list is long.
bsum :: Num a => [a] -> a
bsum                = foldb (+) 0 

foldb' ::  (a -> a -> a) -> a -> [a] -> a
foldb' f u xs       = foldl' sum u (foldl' add [Nothing] xs)
   where add st x   = seq x $ case st of
                                [] -> [Just x]
                                Nothing : st' -> Just x : st'
                                Just y : st' -> seq st' (Nothing : add st' (f x y))
         sum s m    = maybe s (f s) m

isum' :: [IReal] -> IReal
isum' xs            = isumN' (fromIntegral (length xs)) xs

-- | 1st arg should be length of 2nd arg.
isumN' :: Integer -> [IReal] -> IReal
isumN' _ []         = 0
isumN' _ [x]        = x
isumN' n xs         = ir (\p -> scale (foldl' pl 0 (map (flip appr (p+p')) xs)) (-p'))
   where p'         = lg2 n + 2
         pl (I (a,b)) (I (c,d)) 
                    = seq e (seq f (I (e,f)))
            where e = a+c
                  f = b+d

