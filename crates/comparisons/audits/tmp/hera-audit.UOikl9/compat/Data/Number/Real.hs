module Data.Number.Real ( 
              -- | show x will output as much decimalas as
              -- a standard IEEE 754 double if possible.


              -- | (==) and (/=) should not be used as x == y will diverge if
              -- two reals should be equal.

              CReal(), Nat, Chain,
              PBool (..),
              min, max, 
              lim, limRec, limRat, infSum, infSumRec,
              approx,
              pCompare, (<.), (>.), sqrt, exp, log,
              fromDyadic, fromInt, fromWord, fromString, toString, toStringDec
            ) where

import qualified Data.Number.DyadicInterval as DI
import qualified Data.Number.Ball as B
import qualified Data.Number.Dyadic as D

import Data.Order

import Data.Word(Word)
import Prelude hiding (min, max, sqrt, log, exp)

import Data.IORef(IORef, newIORef, writeIORef, readIORef)
import System.IO.Unsafe(unsafePerformIO)

import Data.Maybe(isNothing, fromMaybe)

import Data.Ratio(numerator, denominator)

type Nat = Word

type Chain = Nat -> DI.Interval

-- | Real number is represented as a chain of dyadic intervals which
-- are neither necessarily nested nor bounded away from 0.
--
-- On n-th stage computations are performed with precision of n bits.
data CReal = CReal { state :: IORef (Nat, DI.Interval),
                     eval :: Nat -> CReal -> DI.Interval }

{-# INLINE make #-}
make   :: Chain -> CReal
make c = CReal { state = unsafePerformIO $ newIORef (D.minPrec, c D.minPrec) ,
                 eval = \n (CReal s _) -> unsafePerformIO $
                                           do (n', i) <- readIORef s
                                              if n' == n then return i
                                                else do let i' = c n
                                                        writeIORef s (n, i')
                                                        return i'
               } 

{-# INLINE represent #-}
represent     :: (D.Precision -> DI.Interval -> DI.Interval) -> CReal -> CReal     
represent f r = make (\n -> f n (eval r n r))

{-# INLINE represent2 #-}
represent2        :: (D.Precision -> DI.Interval -> DI.Interval -> DI.Interval)
                     -> CReal -> CReal -> CReal
represent2 f r r' =  make (\n -> f n (eval r n r) (eval r' n r'))


max :: CReal -> CReal -> CReal
max = represent2 DI.maxI

min :: CReal -> CReal -> CReal
min = represent2 DI.minI

instance Eq CReal where
    r /= r' = or (map (isNothing . (\n -> DI.intersect (eval r n r) (eval r' n r'))) [1..])  

instance Show CReal where
    show = toStringDec 16 

instance Read CReal where
    readsPrec _ s = [(fromString s, "")]

instance Num CReal where
    (+) = represent2 DI.add
    (-) = represent2 DI.sub
    (*) = represent2 DI.mul
    negate = represent DI.neg
    abs r = max r (negate r)
    signum r = make $ \n -> case DI.compareI (eval r n r) (eval 0 n 0) of
                              Less    -> DI.fromInt D.minPrec (negate 1)
                              Greater -> DI.fromInt D.minPrec 1
                              _       -> DI.fromBall (B.Ball 0 1)
    fromInteger = fromDyadic . fromInteger

instance Fractional CReal where
    (/) = represent2 DI.div
    recip r = 1 / r
    fromRational r = fromIntegral (numerator r) / fromIntegral (denominator r)

sqrt :: CReal -> CReal
sqrt = represent DI.sqrt

exp :: CReal -> CReal
exp = represent DI.exp

log :: CReal -> CReal
log = represent DI.log

              
-- | A basic general limit which takes as arguments a sequence of reals and a sequence of 
-- error bounds. 
lim       :: (Nat -> CReal) -- ^ Sequence
            -> (Nat -> CReal) -- ^ Error bounds
            -> CReal
lim am rm = make limStage
    where limStage n =  foldl1 DI.intersect lst
              where lst = take (fromIntegral n) .
                          map (\k -> let n' = if k < div n 2 then k else n
                                         (a, r) = (am k, rm k) -- get k-th element of the sequence 
                                         (an, rn) = (eval a n' a, eval r n' r) -- get the n-th approximation of the k-th element
                                         i = case (an, rn) of
                                                 (Just b, Just b') -> DI.fromBall (B.Ball (B.center b) (B.radius b + B.upper_ b'))
                                                 _                 -> Nothing
                                     in i) $ [1..]

-- | Similar to lim, but can sometimes be more convenient for some sequences
limRec      :: CReal -- ^ initial value
               -> (CReal -> Nat -> (CReal, CReal)) -- ^ a function which produces a pair, (next element, error estimate)
                                                -- from previous one and location
               -> CReal
limRec st f = make limStage
    where limStage n = limStage' 1 st (eval st n st)
              where limStage' k st' acc = 
                        let (an, rn) = f st' k -- n-th element of the sequence
                            (ak, rk) = (eval an n an, eval rn n rn) -- k-th approximation
                            i = case (ak, rk) of
                                  (Just b, Just b') -> DI.fromBall (B.Ball (B.center b) (B.radius b + B.upper_ b'))
                                  _                 -> Nothing
                        in if k == n then DI.intersect acc i
                             else limStage' (succ k) an (DI.intersect acc i)

-- | Limit of a sequence of rationals.
limRat :: (Nat -> D.Dyadic) -- ^ Sequence of dyadics
          -> (Nat -> D.Dyadic) -- ^ Sequence of (dyadic) error bounds
          -> CReal
limRat an rn = make (\n -> DI.fromBall (B.Ball (an n) (rn n)))


-- | Computes an infinite sum of a series         
infSum      :: (Nat -> CReal) -- ^ Sequence of reals
               -> (Nat -> CReal) -- ^ Sequence of series remainders
               -> CReal
infSum am rm = make partialsum
    where partialsum k = psum 1 (eval a0 k a0) Nothing
            where psum n acc res = 
                      let (an,rn) = (am n, rm n)
                          err = eval rn k rn
                          acc' = DI.add k acc (eval an k an)
                          (res', p) = case (acc', err) of
                                        (Just  b, Just b') -> 
                                            let (cac,rac) = (B.center b, B.radius b)
                                                (ler, uer) = (B.lower_ b', B.upper_ b')
                                            in (DI.intersect res (DI.fromBall (B.Ball cac (rac + uer))), rac <= ler)
                                        (Nothing, _) -> (Nothing, False)
                                        (_, Nothing) -> (res, True)
                      in if p then psum (succ n) acc' res'
                           else res'
          a0 = am 0
           

-- | Similar to infSum but can sometimes be more convenient
-- Second argument is a_0
infSumRec      :: CReal
               -> (CReal -> Nat -> (CReal, CReal)) 
               -> CReal
infSumRec st f = make partialsum
    where partialsum k = psum 1 (eval a0 k a0) Nothing a0
            where psum n acc res t = 
                      let (an, rn) = f t n
                          err = eval rn k rn
                          acc' = DI.add k acc (eval an k an)
                          (res', p) = case (acc', err) of
                                        (Just  b, Just b') -> 
                                            let (cac,rac) = (B.center b, B.radius b)
                                                (ler, uer) = (B.lower_ b', B.upper_ b')
                                            in (DI.intersect res (DI.fromBall (B.Ball cac (rac + uer))), rac <= ler)
                                        (Nothing, _) -> (Nothing, False)
                                        (_, Nothing) -> (res, True)
                      in if p then psum (succ n) acc' res' an
                         else res'
          a0 = st

-- comparison functions

-- | @ pCompare x y @ returns a function @ Nat -> POrdering @ which
-- when applied to some @ n @ computes approximates with precision @ n @
-- and then compares the resulting intervals
pCompare      :: CReal -> CReal -> Nat -> POrdering
pCompare r r' = \n -> DI.compareI (eval r n r) (eval r' n r')

-- | @ x \<. y @ is a function @ Nat -> PBool @ which, when
-- applied to some @ n @, computes the approximation with precision @ n @ 
-- and then compares the intervals. If intervals are disjoint then result is 
-- either PTrue or PFalse, otherwise result is Indeterminate.
infix 4 <.
(<.) :: CReal -> CReal -> Nat -> PBool
(<.) r r' = \n -> case pCompare r r' n of
                    Less    -> PTrue
                    Greater -> PFalse
                    _       -> Indeterminate

-- | Similar to (<.)
infix 4 >.
(>.) :: CReal -> CReal -> Nat -> PBool
(>.) r r' = \n -> case pCompare r r' n of
                    Less    -> PFalse
                    Greater -> PTrue
                    _       -> Indeterminate

-- | @ approx x n @ tries to compute a dyadic approximation to x so than @ |x - d| <= 10^(-n) @
-- If it succeeds it returns @ Right d @ where d is a dyadic rational, otherwise it returns
-- Left (d, n) where d is a dyadic rational and n is the number of accurate decimal places
--
-- Approx succeeds if result can be computed with precision less than the square of the number
-- of required bits of precision.
approx     :: CReal -> Nat -> Either (D.Dyadic, Word) D.Dyadic
approx r k = approx' n
             where approx' :: Nat -> Either (D.Dyadic, Word) D.Dyadic
                   approx' n' | cp >= fromIntegral n = Right c
                              | threshold = Left (c, floor (logBase 10 2 * fromIntegral cp :: Double))
                              | otherwise = approx' $ 2 * n'
                              where cp = if r' == 0 then fromIntegral n 
                                           else let t = negate . D.getExp $ r'
                                                in if t >= 0 then t else 0
                                    B.Ball c r' = fromMaybe (B.Ball 0 (D.pow2 31)) (eval r n' r)
                                    threshold = n * n < n'
                   n = ceiling ((logBase 2 10 :: Double) * fromIntegral k) + 1

fromDyadic   :: D.Dyadic -> CReal
fromDyadic d = make $ \_ -> DI.fromBall (B.Ball d $ 0)

-- | fromInt should be preferred over fromIntegral where applicable
fromInt   :: Int -> CReal
fromInt i = make $ \_ -> DI.fromBall $ B.Ball (D.fromInt D.Near 32 i) $ 0

-- | fromWord should be preferred over fromIntegral where applicable
fromWord   :: Word -> CReal
fromWord i = make $ \_ -> DI.fromBall $ B.Ball (D.fromWord D.Near 32 i) $ 0


fromString   :: String -> CReal
fromString s = make (\_ -> let l = length s
                               n = ceiling (logBase 2 10 * fromIntegral (if elem '.' s then pred l else l) :: Double)
                               cen = D.fromString s n 10 in 
                           DI.fromBall (B.Ball cen 0))


-- | toStringDec tries to compute the result to the number of specified significand digits
toStringDec     :: Nat -> CReal ->  String 
toStringDec n r = inf ++ s
    where (inf, s) = case approx r n of
                       Right d    -> ("",D.toString n d)
                       Left (d,k) -> ("Could not compute to desired accuracy, only to " ++ show k ++ " significand digits : ",
                              D.toString k d) 

-- | toString computes the result with specified precision.
toString     ::  Nat -> CReal -> String
toString n r = DI.toString (eval r n r)