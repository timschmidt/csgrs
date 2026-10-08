{-# LANGUAGE FlexibleContexts, TypeSynonymInstances, MultiParamTypeClasses, FlexibleInstances #-}

{- | We implement real numbers as the completion of dyadic intervals. The whole construction is
   parametrized by an approximate field, an example of which is "Dyadic".
-}

module Data.Reals.Reals (
             RealNum, approximate, ClosedInterval (..), forall
) where

import Data.Approximate.ApproximateField
import Debug.Trace
import Data.Ratio
import Data.Approximate.Interval
import Data.Reals.Space
import Data.Reals.Staged
import Data.Approximate.Floating.MPFR

-- | A real number is implemented as a staged dyadic interval @'Interval' q@ where @q@ is the
-- underlying approximate field (in practice these are dyadic rationals). @'RealNumQ' q@ can be used
-- to represent not only real numbers but also the elements of the interval domain, including the
-- back-to-front intervals.

type RealNumQ q = StagedWithList (Interval q)
type RealNum = RealNumQ Rounded

-- | We implement a very simple show instance for reals which computes the 20th approximation
-- and shows it as an interval, together with a floating point approximation.
instance ApproximateField q => Show (RealNumQ q) where
   show x = let i = approximate x (prec RoundDown 20)
            in show i

-- | Linear order on real numbers
instance Ord (Interval q) => LinearOrder (RealNumQ q) where
    less = lift2 (\s -> (<))

-- | The Hausdorff property
instance Ord (Interval q) => Hausdorff (RealNumQ q) where
     x `apart` y = (x `less` y) `sor` (y `less` x)

-- | It is a bad idea to use Haskell-style inequality @/=@ on reals because it either returns @True@
-- or it diverges. Similarly, using Haskell equality @==@ is bad. Nevertheless, we define @==@ and @/=@
-- because Haskell wants them for numeric types.
instance Ord (Interval q) => Eq (RealNumQ q) where
    x /= y = force $ x `apart` y

-- | Real numbers are an ordered type in the sense of Haskells 'Ord', although a comparison never
-- returns @EQ@ (instead it diverges). This is a fact of life, comparison of reals is not decidable.
instance Ord (Interval q) => Ord (RealNumQ q) where
  compare x y = if force (x `less` y) then LT else GT

-- | The ring structure fo the reals.
instance (DyadicField q, ApproximateField (Interval q)) => Num (RealNumQ q) where
    (+) = lift2 appAdd
    (-) = lift2 appSub
    (*) = lift2 appMul

    abs = lift1 appAbs

    signum _ = error "could diverge"
    {- signum x = do i <- x
                      s <- getStage
                      return Interval { lower = app_signum s (lower i),
                                          upper = app_signum (anti s) (upper i) --}

    fromInteger k = limit $ \s -> i
                      where i = appFromInteger k

-- | Division and reciprocals.
instance (DyadicField q, ApproximateField (Interval q)) => Fractional (RealNumQ q) where
    (/) = lift2 appDiv

    recip = lift1 appInv

    fromRational r = limit $ \s ->
                                ({-traceShow ("fr",r, s)-} appFromRational s r )--, fromRational r)

-- | The value @ClosedInterval(a,b)@ represents the closed interval [a,b] as a subspace of the reals.
newtype ClosedInterval q = ClosedInterval (q, q)

-- | Compactness of the closed interval
instance (DyadicField q) => Compact (ClosedInterval q) (RealNumQ q) where
   forall (ClosedInterval(a,b)) p =
     limit (\s ->
       let r = rounding s
           n = precision s
           test_interval u v = case r of
                                 RoundDown -> Interval {lower = u, upper = v}
                                 RoundUp   -> let w = midpoint u v in Interval {lower = w, upper = w}
           sweep [] = True
           sweep ((k,a,b):lst) = let x = limit $ \s -> test_interval a b
                                    in case (r, approximate (p x) (prec r k)) of
                                      (RoundDown, False) -> (k < n) &&
                                                            (let c = midpoint a b in sweep (lst ++ [(k+1,a,c), (k+1,c,b)]))
                                      (RoundDown, True)  -> sweep lst
                                      (RoundUp,   False) -> False
                                      (RoundUp,   True)  -> (k >= n) ||
                                                            (let c = midpoint a b in sweep (lst ++ [(k+1,a,c), (k+1,c,b)]))
       in sweep [(0,a,b)]
     )
{-
-- | Missing: overtness of reals, open interval (a,b) and closed interval [a,b]
instance Overt (ClosedInterval q) (RealNumQ q) where
     exists (ClosedInterval (a,b)) p = error "Not implemented"

-- | Reals form a complete space, which means that every Cauchy sequence of reals has
-- a limit. In the implementation this is manifested by the existence of an operator
-- which computes the limit of a Cauchy sequence. The error bounds for the sequence are
-- given explicitly.
lim :: (Int -> (RealNumQ q, q)) -> RealNumQ q
lim x =
    error "Limits of Cauchy sequences are not implemented"
    {- there are several ways to implement 'lim', see the end of Section 7 of

       http://math.andrej.com/2008/01/31/a-constructive-theory-of-domains-suitable-for-implementation/

    -}



-- | Reals form an Archimedean field. Topologically speaking, this means that the
-- underlying approximate field @q@ is dense in the reals. Computationally this means
-- that we can compute arbitrarily good @q@-approximations to real numbers. The
-- function 'approxTo x k' computes an approximation @a@ of type @q@ which is within
-- @2^-k@ of @x@.
approxTo :: RealNumQ q -> Int -> q
approxTo x k =
    error "approxTo not implemented"
    {- the simplest way to implement approxTo is to keep calling
       @approximate x (prec RoundDown n)@ until @n@ is so large that the returned
       interval has width at most @2^(-n-1)@. The center of the
       interval is the approximation we seek.

       There are several possibilities for optimization. Here we describe one. Let
       @a_n@ be the midpoint of the interval @approximate x (prec RoundDown n)@ and
       let @r_n@ be its width. We are looking for @n@ such that @r_n < 2^{ -n-1}@.
       One heuristic is to assume that @r_n@'s form a geometric series. From this we
       can look at three terms of the sequence, say @r_10@, @r_20@, and @r_30@ and
       estimate an @n@ which should give @r_n < 2^{ -n-1}@. If the estimate fails,
       we try something else. The drawback is that we might end up over-estimating
       the needed precision @n@. -}
-}
