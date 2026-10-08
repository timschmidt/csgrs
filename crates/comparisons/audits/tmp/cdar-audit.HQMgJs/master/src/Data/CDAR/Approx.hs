{-# LANGUAGE BangPatterns,GADTs,TypeSynonymInstances,FlexibleInstances #-}
{-|
= Computable Real Arithmetic
This module provides the data type 'CR' that implements the real closed field of computable real numbers.

== Centred Dyadic Approximations
The computable reals are realised as lists of rapidly shrinking intervals. The intervals used here are centred dyadic intervals, implemented here as the data type 'Approx'.

For more information on the theoretical aspects see <http://cs.swan.ac.uk/~csjens/pdf/centred.pdf>.
-}
module Data.CDAR.Approx (Approx(..)
                        ,CR(..)
--                        ,errorBits
--                        ,errorBound
--                        ,defaultPrecision
                        ,Precision
                        ,showA
                        ,showInBaseA
                        ,endToApprox
                        ,lowerBound
                        ,upperBound
                        ,lowerA
                        ,upperA
                        ,centre
                        ,centreA
                        ,radius
                        ,diameter
                        ,exact
                        ,approximatedBy
                        ,better
                        ,fromDyadic
                        ,toApprox
                        ,recipA
                        ,divAInteger
                        ,modA
                        ,divModA
                        ,toDoubleA
                        ,precision
                        ,significance
                        ,boundErrorTerm
                        ,limitSize
                        ,checkPrecisionLeft
                        ,limitAndBound
                        ,unionA
                        ,intersectionA
                        ,consistentA
                        ,poly
                        ,pow
                        ,powers
                        ,sqrtHeronA
                        ,sqrtA
                        ,sqrtRecA
                        ,findStartingValues
                        ,sqrtD
                        ,shiftD
                        ,sqrA
                        ,log2Factorials
                        ,taylorA
                        ,expA
                        ,expBinarySplittingA
                        ,expTaylorA
                        ,expTaylorA'
                        ,logA
                        ,logInternal
                        ,logBinarySplittingA
                        ,logTaylorA
                        ,sinTaylorA
                        ,sinTaylorRed1A
                        ,sinTaylorRed2A
                        ,sinA
                        ,cosA
                        ,atanA
                        ,sinBinarySplittingA
                        ,cosBinarySplittingA
                        ,atanBinarySplittingA
                        ,atanTaylorA
                        ,piRaw
                        ,piA
                        ,piMachinA
                        ,piBorweinA
                        ,piAgmA
                        ,log2A
                        ,lnSuperSizeUnknownPi
                        ,logAgmA
                        ,agmLn
                        ,showCRN
                        ,showCR
                        ,ok
                        ,require
                        ,toDouble
                        ,fromDouble
                        ,fromDoubleAsExactValue
                        ,polynomial
                        ,taylorCR
                        ,atanCR
                        ,piCRMachin
                        ,piMachinCR
                        ,piBorweinCR
                        ,piBinSplitCR
                        ,ln2
                        ,sinCRTaylor
                        ,sinCR
                        ,cosCR
                        ,sqrtCR
                        ,expCR
                        ,compareA
                        ,compareCR
                        ,caseA
                        ,caseCR
                        ,Select (..)
                        ,selectA
                        ,selectCR
                        ,floorA
                        ,ceilingA
                        ,floorCR
                        ,ceilingCR
                        ,limCR
                        ,unitError) where

import           Control.Applicative (ZipList (..))
import           Control.DeepSeq
import           Control.Exception
import           Data.Bits
import           Data.Char (isDigit)
import           Data.CDAR.Classes
import           Data.CDAR.Dyadic
import           Data.CDAR.Extended
import           Data.CDAR.IntegerLog
import           Data.Char (intToDigit)
import           Data.List (findIndex, intersperse, transpose, unfoldr, zipWith4)
import           Data.Ratio

--import Debug.Trace

-- |A type synonym. Used to denote number of bits after binary point.
type Precision = Int

{-|
= Centred Dyadic Approximations
There are two constructors for approximations:

- 'Approx' is encodes some finite interval with dyadic endpoints. A real
  number is /approximated/ by the approximation is it belongs to the interval.
- 'Bottom' is the trivial approximation that approximates all real numbers.

The three fields of an @Approx m e s@ should be thought of as:

[@m@] the midpoint
[@e@] the error term
[@s@] the exponent

Thus, a value @Approx m e s@ is to be interpreted as the interval
[(m-e)*2^s, (m+e)*2^s].

== Centred intervals
We have opted for a centred representation of the intervals. It is also
possible to represent the endpoints as 'Dyadic' numbers. The rationale for a
centred repersentation is that we often normalise an approximation @Approx m e
s@ so that @e@ is limited in size. This allows many operation to only work on
one large number @m@.

== Potential for overflow
Since the third field (the exponent) is only an 'Int' it may overflow. This is
an optimisation that was adopted since it seems unlikely that overflow in a 64
bit Int exponent would occur. In a 32 bit system, this is potentially an
issue.

The 'Integer' data type is unbonded, but is, of course, bounded by the
available memory available in the computer. No attempt has been made to check
for exhausted memory.

== Approximations as a Domain

Ordered by reverse inclusion the dyadic intervals encoded by the 'Approx'
approximations (including 'Bottom') constitute the compact elements of a Scott
domain /D/. (It is a substructure of the (algebraic) interval domain.)
We will identify our approximations with the compact elements of /D/.

Increasing sequences in /D/ have suprema. A sequence /converges/ if the length
of the approximations tend to zero. The supremum of a converging sequence is a
singleton set containing a real number. Let ρ be the map taking a converging
sequence to the unique real number in the supremum. The computations on
(computable) real numbers is via this representation map ρ.

There is no check that the sequences we have are in fact increasing, but we
are assuming that all sequences are pairwise consistent. We can thus create an
increasing sequence by considering the sequence of finite suprema. For
correctness, we have to ensure that all operations done on consistent
sequences result in consistent sequences. If non-consistent sequences are
somehow input we can make no guarantees at all about the computed value.

Note, that we cannot ensure that converging sequences are mapped to converging
sequences because of properties of computable real arithmetic. In particular,
at any discuntinuity, it is impossible to compute a converging sequence.
-}
data Approx = Approx Integer Integer Int
            | Bottom
              deriving (Read,Show)

instance NFData Approx where
    rnf Bottom = ()
    rnf (Approx m e s) = rnf m `seq` rnf e `seq` rnf s

instance Scalable Approx where
  scale Bottom _ = Bottom
  scale (Approx m e s) n = Approx m e (s+n)

instance Scalable CR where
  scale (CR x) n = CR $ flip scale n <$> x

{-|
=The Computable Real data type

Computable reals are realised as infinite sequences of centred dyadic
representations.

All approximations in such a sequence should be pairwise consistent, i.e.,
have a non-empty intersection. However, there is no check that this is
actually the case.

If the diameter of the approximations tend to zero we say that the sequences
converges to the unique real number in the intersection of all intervals.
Given the domain /D/ of approximations described in 'Approx', we have a
representation (a retraction) ρ from the converging sequences in /D/ to ℝ.
Some operations on computable reals are partial, notably equality and
ordering. A consequence of this is that there is no guarantee that a
computable real will converge.

For the /n/-th element in the sequence there is a bound on how much effort is
put into the computation of the approximation. For involved computations it is
possible that several of the leading approximations are trivial, i.e.,
'Bottom'. If the computation will eventually converge, it will generate proper
approximation after a modest number of initial trivial approximations.

The amount of added effort in each iteration is rather substantial so the
expected precision of approximations increase very quickly.

==The actual data type

In fact, the type 'CR' is a newtype of 'ZipList' 'Approx' in the
implementation of infinite sequences of approximations, as that allows for
applicative style. Hopefully, it is not needed to access the internal
representation of 'CR' directly.
-}
newtype CR = CR {unCR :: ZipList Approx}

-- |Number of bits that error term is allowed to take up. A larger size allows
-- for more precise but slightly more costly computations. The value here is
-- suggested by test runs.
errorBits :: Int
errorBits = 10

errorBound :: Integer
errorBound = 2^errorBits

-- |The default cutoff for diverging computations. May well be chosen much
-- smaller. 31 corresponds to about 10 decimal places.
defaultPrecision :: Precision
defaultPrecision = 31

{-|

Gives a decimal representation of an approximation. It tries to give as many
decimal digits as possible given the precision of the approximation. The
representation may be wrong by 1 ulp (unit in last place). If the value is not
exact the representation will be followed by @~@.

The representation is not always intuitive:

>>> showA (Approx 1 1 0)
"1.~"

The meaning of the above is that it is 1, but then the added @~@ (which must
be after the decimal point) means that the last position may be off by 1,
i.e., it could be down to 0 or up to 2. And [0,2] is indeed the range encoded
by the above approximation.
-}
showA :: Approx -> String
showA = showInBaseA 10

-- |Similar to 'showA' but can generate representations in other bases (<= 16).
{- am is the absolute value of the significand
   b corresponds to the value 1 with respect to the shift s -- this is used to find the digits in the auxiliary functions
   i is the integral part of am
   f is the fractional part of am
   i' and f' are the integral and fractional parts relevant for near zero approximations
   e' is the error term shifted appropriately when s positive, also set to at least 1
     (otherwise odd bases will yield infinite expansions)
-}
showInBaseA :: Int -> Approx -> String
showInBaseA _ Bottom = "⊥"
showInBaseA base (Approx m e s)
    | e == 0 && (even base || s >= 0)
                     = sign ++ showExactA base b i f
    | am < e         = "±" ++ showNearZeroA base b i' f'
    | otherwise      = sign ++ showInexactA base b i f e'
    where b = bit (max 0 (-s))
          am = abs m
          i = shift am s
          e' = max 1 $ shift e (max 0 s)
          f = am .&. (b-1)
          i' = shift (am+e) s
          f' = (am+e) .&. (b-1)
          sign = if m < 0 then "-" else ""

showExactA :: Int -> Integer -> Integer -> Integer -> String
showExactA base b i f = 
    let g i' = let (q,r) = quotRem i' (fromIntegral base)
               in if i' == 0 then Nothing
                  else Just (intToDigit (fromIntegral r), q)
        ip = reverse (unfoldr g i)
        h f' = let (q,r) = quotRem ((fromIntegral base)*f') b
               in if f' == 0 then Nothing
                  else Just (intToDigit (fromIntegral q), r)
        fp = unfoldr h f
    in (if null ip then "0" else ip)
       ++ (if null fp then "" else ".")
       ++ fp

showNearZeroA :: Int -> Integer -> Integer -> Integer -> String
showNearZeroA base b i f =
    let s = showExactA base b i f
        t = takeWhile (flip elem "0.~") s
        u = takeWhile (/= '.') s
    in if null t
       then replicate (length u) '~'
       else t ++ "~"

showInexactA :: Int -> Integer -> Integer -> Integer -> Integer -> String
showInexactA base b i f e =
    let g (0,b',f') = if b' < f'+e
                      then Just ('1', (0, (fromIntegral base)*b', f'))
                      else Nothing
        g (n,b',f') = let (q,r) = quotRem n (fromIntegral base)
                          z = (q, (fromIntegral base)*b', r*b'+f')
                      in if e+f' <= b'
                         then Just (intToDigit (fromIntegral r), z)
                         else if e <= min f' b'
                              then Just (intToDigit ((fromIntegral r + 1) `rem` (fromIntegral base)), z)
                              else Just ('~', z)
        intRev = unfoldr g (i,b,f)
        noFrac = case intRev of
                   [] -> False
                   (x:_) -> x == '~'
        int = if null intRev then "0" else reverse intRev
        h (f',err) = let (q,r) = quotRem ((fromIntegral base)*f') b
                         err' = (fromIntegral base)*err
                         z = (r, err')
                     in if err' + r <= b
                        then Just (intToDigit (fromIntegral q), z)
                        else if err' <= min r b
                             then Just (intToDigit ((fromIntegral q + 1) `rem` (fromIntegral base)), z)
                             else Nothing
        frac = unfoldr h (f,e)
    in int ++ if noFrac
              then ""
              else "." ++ frac ++ "~"

-- |Construct a centred approximation from the end-points.
endToApprox :: Extended Dyadic -> Extended Dyadic -> Approx
endToApprox (Finite l) (Finite u)
  | u < l = Bottom -- Might be better with a signalling error.
  | otherwise =
    let a@(m:^s) = scale (l + u) (-1)
        (n:^t)   = u-a
        r        = min s t
        m'       = unsafeShiftL m (s-r)
        n'       = unsafeShiftL n (t-r)
    in (Approx m' n' r)
endToApprox _ _ = Bottom

-- Interval operations
-- |Gives the lower bound of an approximation as an 'Extended' 'Dyadic' number.
lowerBound :: Approx -> Extended Dyadic
lowerBound (Approx m e s) = Finite ((m-e):^s)
lowerBound Bottom = NegInf

-- |Gives the upper bound of an approximation as an 'Extended' 'Dyadic' number.
upperBound :: Approx -> Extended Dyadic
upperBound (Approx m e s) = Finite ((m+e):^s)
upperBound Bottom = PosInf

-- |Gives the lower bound of an 'Approx' as an exact 'Approx'.
lowerA :: Approx -> Approx
lowerA Bottom = Bottom
lowerA (Approx m e s) = Approx (m-e) 0 s

-- |Gives the upper bound of an 'Approx' as an exact 'Approx'.
upperA :: Approx -> Approx
upperA Bottom = Bottom
upperA (Approx m e s) = Approx (m+e) 0 s

-- |Gives the mid-point of an approximation as a 'Maybe' 'Dyadic' number.
centre :: Approx -> Maybe Dyadic
centre (Approx m _ s) = Just (m:^s)
centre _ = Nothing

-- |Gives the centre of an 'Approx' as an exact 'Approx'.
centreA :: Approx -> Approx
centreA Bottom = Bottom
centreA (Approx m _ s) = Approx m 0 s

-- |Gives the radius of an approximation as a 'Dyadic' number. Currently a
-- partial function.
radius :: Approx -> Extended Dyadic
radius (Approx _ e s) = Finite (e:^s)
radius _ = PosInf

-- |Gives the lower bound of an approximation as an 'Extended' 'Dyadic' number.
diameter :: Approx -> Extended Dyadic
diameter (Approx _ e s) = Finite $ 2 * (e:^s)
diameter _ = PosInf

-- |Returns 'True' if the approximation is exact, i.e., it's diameter is 0.
exact :: Approx -> Bool
exact (Approx _ 0 _) = True
exact _ = False

-- |Checks if a number is approximated by an approximation, i.e., if it
-- belongs to the interval encoded by the approximation.
approximatedBy :: Real a => a -> Approx -> Bool
_ `approximatedBy` Bottom = True
r `approximatedBy` d =
    let r' = toRational r
    in toRational (lowerBound d) <= r' && r' <= toRational (upperBound d)

-- |A partial order on approximations. The first approximation is better than
-- the second if it is a sub-interval of the second.
better :: Approx -> Approx -> Bool
d `better` e = lowerBound d >= lowerBound e &&
               upperBound d <= upperBound e

-- |Turns a 'Dyadic' number into an exact approximation.
fromDyadic :: Dyadic -> Approx
fromDyadic (m:^s) = Approx m 0 s

-- |Two approximations are equal if they encode the same interval.
instance Eq Approx where
    (Approx m e s) == (Approx n f t)
        | s >= t = let k = s-t
                   in unsafeShiftL m k == n && unsafeShiftL e k == f
        | s <  t = let k = t-s
                   in m == unsafeShiftL n k && e == unsafeShiftL f k
    Bottom == Bottom = True
    _ == _ = False

-- |Not a sensible instance. Just used to allow to allow enumerating integers
-- using \'..\' notation.
instance Enum Approx where
    toEnum n = Approx (fromIntegral n) 0 0
    fromEnum (Approx m _ s) = fromIntegral $ shift m s
    fromEnum Bottom = 0

instance Num Approx where
    (Approx m e s) + (Approx n f t)
        | s >= t = let k = s-t
                   in Approx (unsafeShiftL m k + n) (unsafeShiftL e k + f) t
        | s <  t = let k = t-s
                   in Approx (m + unsafeShiftL n k) (e + unsafeShiftL f k) s
    _ + _ = Bottom
    (Approx m e s) * (Approx n f t)
        | am >= e && an >= f && a > 0           = Approx (a+d) (ab+ac) u
        | am >= e && an >= f && a < 0           = Approx (a-d) (ab+ac) u
        | am < e && n >= f                      = Approx (a+b) (ac+d) u
        | am < e && -n >= f                     = Approx (a-b) (ac+d) u
        | m >= e && an < f                      = Approx (a+c) (ab+d) u
        | -m >= e && an < f                     = Approx (a-c) (ab+d) u
        | a == 0                                = Approx (0) (ab+ac+d) u
        | am < e && an < f && a > 0 && ab > ac  = Approx (a+ac) (ab+d) u
        | am < e && an < f && a > 0 && ab <= ac = Approx (a+ab) (ac+d) u
        | am < e && an < f && a < 0 && ab > ac  = Approx (a-ac) (ab+d) u
        | am < e && an < f && a < 0 && ab <= ac = Approx (a-ab) (ac+d) u
      where am = (abs m)
            an = (abs n)
            a = m*n
            b = m*f
            c = n*e
            d = e*f
            ab = (abs b)
            ac = (abs c)
            u = s+t
    _ * _ = Bottom
    negate (Approx m e s) = Approx (-m) e s
    negate Bottom = Bottom
    abs (Approx m e s)
        | m' < e    = let e' = m'+e
                      in Approx e' e' (s-1)
        | otherwise = Approx m' e s
      where m' = abs m
    abs Bottom = Bottom
    signum (Approx m e _)
        | e == 0 = Approx (signum m) 0 0
        | abs m < e = Approx 0 1 0
        | abs m == e = Approx (signum m) 1 (-1)
        | otherwise = Approx (signum m) 0 0
    signum Bottom = Approx 0 1 0
    fromInteger i = Approx i 0 0

-- |Convert a rational number into an approximation of that number with
-- 'Precision' bits correct after the binary point.
toApprox :: Precision -> Rational -> Approx
toApprox t r = Approx (2 * round (r*2^^t)) 1 (-t - 1)

-- |Not a proper Fractional type as Approx are intervals.
instance Fractional Approx where
    fromRational = toApprox defaultPrecision
    recip = recipA defaultPrecision

-- |Compute the reciprocal of an approximation. The number of bits after the
-- binary point is bounded by the 'Precision' argument if the input is exact.
-- Otherwise, a good approximation with essentially the same significance as
-- the input will be computed.
recipA :: Precision -> Approx -> Approx
recipA _ Bottom = Bottom
recipA t (Approx m e s)
    | e == 0      = let s' = integerLog2 (abs m)
                    in Approx
                         (round (bit (s'+t+2) % m))
                         1
                         (-s-s'-t-2)
    | (abs m) > e = let d = m*m-e*e
                        d2 = unsafeShiftR d 1
                        s' = integerLog2 d + 2 * errorBits
                    in boundErrorTerm $ Approx
                           ((unsafeShiftL m s' + d2) `div` d)
                           ((unsafeShiftL e s' + d2+1) `div` d + 1)
                           (-s-s')
    --  (abs m) > e = let d = m*m-e*e
    --                     s' = 2 * (integerLog2 m + errorBits)
    --                 in boundErrorTerm $ Approx
    --                        (round (unsafeShiftL m s'%(d)))
    --                        (ceiling (1%2 + unsafeShiftL e s'%(d)))
    --                        (-s-s')
    | otherwise   = Bottom

-- |Divide an approximation by an integer.
divAInteger :: Approx -> Integer -> Approx
divAInteger Bottom _ = Bottom
divAInteger (Approx m e s) n =
  let t = integerLog2 n
  in Approx (round (unsafeShiftL m t % n))
             (ceiling (unsafeShiftL e t % n))
             s

-- |Compute the modulus of two approximations.
modA :: Approx -> Approx -> Approx
modA (Approx m e s) (Approx n f t) =
    let r = min s t
        (d,m') = divMod (unsafeShiftL m (s-r)) (unsafeShiftL n (t-r))
        e' = scale e (s-r) + abs d * scale f (t-r)
    in Approx m' e' r
modA _ _ = Bottom

-- |Compute the integer quotient (although returned as an 'Approx' since it
-- may be necessary to return 'Bottom' if the integer quotient can't be
-- determined) and the modulus as an approximation of two approximations.
divModA :: Approx -> Approx -> (Approx, Approx)
divModA (Approx m e s) (Approx n f t) =
    let r = min s t
        (d,m') = divMod (unsafeShiftL m (s-r)) (unsafeShiftL n (t-r))
        e' = e + abs d * f
    in (fromIntegral d, Approx m' e' r)
divModA _ _ = (Bottom, Bottom)

-- | Compare 'Approx' with a partial result.
compareA :: Approx -> Approx -> Maybe Ordering
compareA (Approx m e s) (Approx n f t)
  | abs ((m:^s)-(n:^t)) > (e:^s)+(f:^t) = Just $ compare (m:^s) (n:^t)
  | otherwise                           = Nothing
compareA _ _ = Nothing

-- |Not a proper Ord type as Approx are intervals.
instance Ord Approx where
  compare x y = maybe (error "compare: comparisons are partial on Approx") id $ compareA x y

-- |The 'toRational' function is partial since there is no good rational
-- number to return for the trivial approximation 'Bottom'.
--
-- Note that converting to a rational number will give only a single rational
-- point. Do not expect to be able to recover the interval from this value.
instance Real Approx where
    toRational (Approx m e s) = approxRational
                                  (toRational (m:^s))
                                  (toRational (e:^s))
    toRational _ = undefined

-- |Convert the centre of an approximation into a 'Maybe' 'Double'.
toDoubleA :: Approx -> Maybe Double
toDoubleA = fmap (fromRational . toRational) . centre


-- |Computes the precision of an approximation. This is roughly the number of
-- correct bits after the binary point.
precision :: Approx -> Extended Precision
precision (Approx _ 0 _) = PosInf
precision (Approx _ e s) = Finite $ - s - (integerLog2 e) - 1
precision Bottom         = NegInf

-- |Computes the significance of an approximation. This is roughly the number
-- of significant bits.
significance :: Approx -> Extended Int
significance (Approx _ 0 _) = PosInf
significance (Approx 0 _ _) = NegInf
significance (Approx m 1 _) =  Finite $ integerLog2 (abs m) - 1
significance (Approx m e _) =
    Finite $ (integerLog2 (abs m)) - (integerLog2 (e-1)) - 1
significance Bottom         = NegInf

{-|
This function bounds the error term of an 'Approx'.

It is always the case that @x `'better'` boundErrorTerm x@.

Consider an approximation @Approx m e s@. If @e@ has /k/ bits then that
essentially expresses that the last /k/ bits of @m@ are unknown or garbage. By
scaling both @m@ and @e@ so that @e@ has a small number of bits we save on
memory space and computational effort to compute operations. On the other
hand, if we remove too many bits in this way, the shift in the mid-point of the
interval becomes noticable and it may adversely affect convergence speed of
computations. The number of bits allowed for @e@ after the operation is
determined by the constant 'errorBits'.

== Domain interpretation and verification

For this implementation to be correct it is required that this function is
below the identity on the domain /D/ of 'Approx'. For efficiency it is
desirable to be as close to the identity as possible.

This function will map a converging sequence to a converging sequence.
-}
boundErrorTerm :: Approx -> Approx
boundErrorTerm Bottom = Bottom
boundErrorTerm a@(Approx m e s)
    | e < errorBound = a
    | otherwise =
        let k = integerLog2 e + 1 - errorBits
            t = testBit m (k-1)
            m' = unsafeShiftR m k
            -- may overflow and use errorBits+1
            e' = unsafeShiftR (e + bit (k-1)) k + 1 
        in if t
           then Approx (m'+1) e' (s+k)
           else Approx m'     e' (s+k)

{-|
Limits the size of an approximation by restricting how much precision an
approximation can have.

It is always the case that @x `'better'` limitSize x@.

This is accomplished by restricting the exponent of the approximation from
below. In other words, we limit the precision possible.

It is conceivable to limit the significance of an approximation rather than
the precision. This would be an interesting research topic.

== Domain interpretation and verification

For this implementation to be correct it is required that this function is
below the identity on the domain /D/ of 'Approx'. For efficiency it is
desirable to be as close to the identity as possible.

This function will NOT map a converging sequence to a converging sequence for
a fixed precision argument. However, if the function is applied with
increasing precision for a converging sequence, then this will give a
converging sequence.
-}
limitSize :: Precision -> Approx -> Approx
limitSize _ Bottom = Bottom
limitSize l a@(Approx m e s)
    | k > 0     = Approx
                  ((if testBit m (k-1) then (+1) else id) (unsafeShiftR m k))
                  (1 + (unsafeShiftR (e + bit (k-1)) k))
                  (-l)
    | otherwise = a
    where k = (-s)-l

-- |Throws an exception if the precision of an approximation is not larger
-- than the deafult minimum.
checkPrecisionLeft :: Approx -> Approx
checkPrecisionLeft a
        | precision a > pure defaultPrecision = a
        | otherwise = throw $ LossOfPrecision

-- |Bounds the error term and limits the precision of an approximation.
--
-- It is always the case that @x `'better'` limitAndBound x@.
limitAndBound :: Precision -> Approx -> Approx
limitAndBound limit =
    limitSize limit . boundErrorTerm

-- | Find the hull of two approximations.
unionA :: Approx -> Approx -> Approx
unionA Bottom _ = Bottom
unionA _ Bottom = Bottom
unionA a b = endToApprox (lowerBound a `min` lowerBound b) (upperBound a `max` upperBound b)

-- | Find the intersection of two approximations.
intersectionA :: Approx -> Approx -> Approx
intersectionA Bottom a = a
intersectionA a Bottom = a
intersectionA a b = if l <= u then endToApprox l u else error "Trying to take intersection of two non-overlapping Approx."
  where l = (lowerBound a `max` lowerBound b)
        u = (upperBound a `min` upperBound b)

-- | Determine if two approximations overlap.
consistentA :: Approx -> Approx -> Bool
consistentA Bottom _ = True
consistentA _ Bottom = True
consistentA a b = (lowerBound a `max` lowerBound b) <= (upperBound a `min` upperBound b)

-- |Given a list of polynom coefficients and a value this evaluates the
-- polynomial at that value.
--
-- Should give a tighter bound on the result since we reduce the dependency
-- problem.
poly :: [Approx] -> Approx -> Approx
poly [] _ = 0
poly _ Bottom = Bottom
poly as x =
    let --poly' :: [Dyadic] -> Dyadic -> Dyadic
        poly' as' x' = sum . zipWith (*) as' $ pow x'
        ms = map ((maybe (error "Can't compute poly with Bottom coefficients") id) . centre) as
        (Just c) = centre x
        (m':^s) = poly' ms c
        ds = zipWith (*) (tail as) (map fromIntegral ([1,2..] :: [Int]))
        (Finite b) = upperBound . abs $ poly' ds x
        (Finite (e':^_)) = fmap (b*) $ radius x
        -- exponent above will be same as s
    in Approx m' e' s

-- |Gives a list of powers of a number, i.e., [1,x,x^2,...].
pow :: (Num a) => a -> [a]
pow x = iterate (* x) 1

-- |Computes lists of binomial coefficients. [[1], [1,1], [1,2,1], [1,3,3,1], ...]
binomialCoefficients :: (Num a) => [[a]]
binomialCoefficients =
    let f ss = 1 : zipWith (+) ss (tail ss) ++ [1]
    in iterate f [1]

-- |Computes powers of approximations. Should give tighter intervals than the
-- general 'pow' since take the dependency problem into account. However, so
-- far benchmarking seems to indicate that the cost is too high, but this may
-- depend on the application.
powers :: Approx -> [Approx]
powers (Approx m e s) =
    let ms = pow m
        es = pow e
        f = reverse . zipWith (*) ms . reverse . zipWith (*) es
        sumAlt [] = (0,0)
        sumAlt (x:[]) = (x,0)
        sumAlt (x:y:xs) = let (a,b) = sumAlt xs in (a+x,b+y)
        g s' (m', e') = Approx m' e' s'
    in zipWith g (iterate (+s) 0) $ map (sumAlt . f) binomialCoefficients
powers _ = repeat Bottom

{-|
Old implementation of sqrt using Heron's method. Using the current method
below proved to be more than twice as fast for small arguments (~50 bits) and
many times faster for larger arguments.
-}
sqrtHeronA :: Precision -> Approx -> Approx
sqrtHeronA _ Bottom = Bottom
sqrtHeronA k a@(Approx m e s)
    | -m > e    = error "Attempting sqrt of Approx containing only negative numbers."
    | m < e     = Bottom
    | e == 0    = let (n:^t) = shiftD (-k) $ sqrtD (-k-2) (m:^s)
                  in Approx n 1 t
    | m == e    = let (n:^t) = sqrtD (s `quot` 2 -errorBits) ((m+e):^s)
                      n' = (n+2) `quot` 2
                  in Approx n' n' t
    | otherwise = let (Finite p) = significance a
                      s' = s `quot` 2 - p - errorBits
                      l@(n:^t) = sqrtD s' ((m-e):^s)
                      (n':^t') = sqrtD' s' ((m+e):^s) l
                  in endToApprox (Finite ((n-1):^t)) (Finite ((n'+1):^t'))

{-|
Compute the square root of an approximation.

This and many other operations on approximations is just a reimplementation of
interval arithmetic, with an extra argument limiting the effort put into the
computation. This is done via the precision argument.

The resulting approximation should approximate the image of every point in the
input approximation.
-}
sqrtA :: Precision -> Approx -> Approx
sqrtA _ x@(Approx 0 0 _) =  x
sqrtA k x = limitAndBound k $ x * sqrtRecA k x

{-|
This uses Newton's method for computing the reciprocal of the square root.
-}
sqrtRecA :: Precision -> Approx -> Approx
sqrtRecA _ Bottom = Bottom
sqrtRecA k a@(Approx m e s)
  | -m > e    = error "Attempting sqrtRec of Approx containing only negative numbers."
  | m < e     = Bottom
  | e == 0    = let (n:^t) = shiftD (-k) $ sqrtRecD (-k-2) (m:^s)
                in Approx n 1 t
  | m == e    = let (n:^t) = sqrtRecD (s `quot` 2 -errorBits) ((m+e):^s)
                    n' = (n+2) `quot` 2
                in Approx n' n' t
  | otherwise = let (Finite p) = significance a
                    s' = s `quot` 2 - p - errorBits
                    (n:^t) = sqrtRecD s' ((m-e):^s) -- upper bound of result
                    -- We have tried to use sqrtRecD' with the above value as
                    -- a first approximation to the result. However, the low
                    -- endpoint may be too far away as a starting value to
                    -- ensure convergence to the right root. It's possible
                    -- that if we swap the order we would be fine. But as it
                    -- is, this computes a new first approximation.
                    (n':^t') = sqrtRecD s' ((m+e):^s) -- lower bound of result
                in endToApprox (Finite ((n'+1):^t')) (Finite ((n-1):^t))

{-|
The starting values for newton iterations can be found using the auxiliary function findStartingValues below.

For example, to generate the starting values for sqrtRecD above using three leading bits for both odd and even exponents the following was used:

> findStartingValues ((1/) . sqrt) [1,1.125..2]
[Approx 4172150648 1 (-32),Approx 3945434766 1 (-32),Approx 3752147976 1 (-32),Approx 3584793264 1 (-32),Approx 3438037830 1 (-32),Approx 3307969824 1 (-32),Approx 3191645366 1 (-32),Approx 3086800564 1 (-32)]
> mapM_ (putStrLn . showInBaseA 2 . limitSize 6) it
0.111110~
0.111011~
0.111000~
0.110101~
0.110011~
0.110001~
0.110000~
0.101110~
> findStartingValues ((1/) . sqrt) [2,2.25..4]
[Approx 2950156016 1 (-32),Approx 2789843678 1 (-32),Approx 2653169278 1 (-32),Approx 2534831626 1 (-32),Approx 2431059864 1 (-32),Approx 2339087894 1 (-32),Approx 2256834080 1 (-32),Approx 2182697612 1 (-32)]
> mapM_ (putStrLn . showInBaseA 2 . limitSize 6) it
0.101100~
0.101010~
0.101000~
0.100110~
0.100100~
0.100011~
0.100010~
0.100001~
-}
findStartingValues :: (Double -> Double) -> [Double] -> [Approx]
findStartingValues f = map (fromRational . toRational . (/2)) . (\l -> zipWith (+) l (tail l)) . map f

-- |Square an approximation. Gives the exact image interval, as opposed to
-- multiplicating a number with itself which will give a slightly larger
-- interval due to the dependency problem.
sqrA :: Approx -> Approx
sqrA Bottom = Bottom
sqrA (Approx m e s)
  | am > e = Approx (m^(2 :: Int) + e^(2 :: Int)) (2*abs m*e) (2*s)
  | otherwise = let m' = (am + e)^(2 :: Int) in Approx m' m' (2*s-1)
  where am = abs m
-- Binary splitting

{-|
Binary splitting summation of linearly convergent series as described in
/'Fast multiprecision evaluation of series of rational numbers'/ by B Haible
and T Papanikolaou, ANTS-III Proceedings of the Third International Symposium
on Algorithmic Number Theory Pages 338-350, 1998.

The main idea is to balance the computations so that more operations are
performed with values of similar size. Using the underlying fast
multiplication algorithms this will give performance benefits.

The algorithm parallelises well. However, a final division is needed at the
end to compute /T\/BQ/ which amount to a substantial portion of the
computation time.
-}
abpq :: Num a => [Integer] -> [Integer] -> [a] -> [a] -> Int -> Int -> (a, a, Integer, a)
abpq as bs ps qs n1 n2
    | n == 1 = (ps !! n1, qs !! n1, bs !! n1, fromIntegral (as !! n1) * ps !! n1)
    | n < 6  = let as' = take n $ drop n1 as
                   bs' = take n $ drop n1 bs
                   ps' = take n $ drop n1 ps
                   qs' = take n $ drop n1 qs
                   pbs = product bs'
                   bs'' = map (pbs `div`) bs'
                   ps'' = scanl1 (*) ps'
                   qs'' = scanr1 (*) (tail qs' ++ [1])
               in (ps'' !! (n-1), product qs', pbs
                  , sum $ zipWith4 (\a b p q -> fromIntegral a * fromIntegral b * p * q)
                                   as' bs'' ps'' qs'')
    | n > 1  =
        let (pl, ql, bl, tl) = abpq as bs ps qs n1 m
            (pr, qr, br, tr) = abpq as bs ps qs m n2
        in (pl * pr, ql * qr, bl * br, fromIntegral br * qr * tl + fromIntegral bl * pl * tr)
    | otherwise = error "Non-expected case in binary splitting"
  where
    n = n2 - n1
    m = (n1 + n2 + 1) `div` 2

ones :: Num a => [a]
ones = repeat 1

{-|
Computes the list [lg 0!, lg 1!, lg 2!, ...].
-}
-- To be changed to Stirling formula if that is faster
log2Factorials :: [Int]
log2Factorials = map integerLog2 . scanl1 (*) $ 1:[1..]

-- Straighforward Taylor summation

{-|
Computes a sum of the form ∑ aₙ/qₙ where aₙ are approximations and qₙ are
integers. Terms are added as long as they are larger than the current
precision bound. The sum is adjusted for the tail of the series. For this to
be correct we need the the terms to converge geometrically to 0 by a factor of
at least 2.
-}
taylor :: Precision -> [Approx] -> [Integer] -> Approx
taylor res as qs =
  let res' = res + errorBits
      f a q = limitAndBound res' $ a * recipA res' (fromIntegral q)
      bs = zipWith f as qs
      (cs,(d:_)) = span nonZeroCentredA bs -- This span and the sum on the next line do probably not fuse!
  in fudge (sum cs) d

-- | A list of factorial values.
fac :: Num a => [a]
fac = map fromInteger $ 1 : scanl1 (*) [1..]

-- | A list of the factorial values of odd numbers.
oddFac :: Num a => [a]
oddFac = let f (_:x:xs) = x:f xs
             f _ = error "Impossible"
         in f fac

{-
evenFac :: Num a => [a]
evenFac = let f (x:_:xs) = x:f xs
              f _ = error "Impossible"
          in f fac
-}

-- | Checks if the centre of an approximation is not 0.
nonZeroCentredA :: Approx -> Bool
nonZeroCentredA Bottom = False
nonZeroCentredA (Approx 0 _ _) = False
nonZeroCentredA _ = True

-- This version is faster especially far smaller precision.

{-|
Computes the sum of the form ∑ aₙxⁿ where aₙ and x are approximations.

Terms are added as long as they are larger than the current precision bound.
The sum is adjusted for the tail of the series. For this to be correct we need
the the terms to converge geometrically to 0 by a factor of at least 2.
-}
taylorA :: Precision -> [Approx] -> Approx -> Approx
taylorA res as x =
  sum . takeWhile nonZeroCentredA . map (limitAndBound res) $ zipWith (*) as (pow x)

{- Exponential computed by standard Taylor expansion after range reduction.
-}

{-|
The exponential of an approximation. There are three implementation using
Taylor expansion here. This is just choosing between them.

More thorough benchmarking would be desirable.

Is faster for small approximations < ~2000 bits.
-}
expA :: Precision -> Approx -> Approx
expA = expTaylorA'

-- | Exponential by binary splitting summation of Taylor series.
expBinarySplittingA :: Precision -> Approx -> Approx
expBinarySplittingA _ Bottom = Bottom
expBinarySplittingA res a@(Approx m e s) =
  let s' = s + integerLog2 m
      -- r' chosen so that a' below is smaller than 1/2
      r' = floor . sqrt . fromIntegral . max 5 $ res
      r = s' + r'
      -- a' is a scaled by 2^k so that 2^(-r') <= a' < 2^(-r'+1)
      a' = Approx m e (s-r)
      -- (Finite c) = min (significance a) (Finite res)
      (Just n) = findIndex (>= res+r) $ zipWith (+) log2Factorials [0,r'..]
      (p, q, b, t) = abpq ones
                          ones
                          (1:repeat a')
                          (1:[1..])
                          0
                          n
      nextTerm = a * p * recipA (res+r) (fromIntegral n * q)
      ss = iterate (boundErrorTerm . sqrA) $ fudge (t * recipA (res+r) (fromIntegral b*q)) nextTerm
  in ss !! r

-- | Exponential by summation of Taylor series.
expTaylorA :: Precision -> Approx -> Approx
expTaylorA _ Bottom = Bottom
expTaylorA res (Approx m e s) =
  let s' = s + integerLog2 m
      -- r' chosen so that a' below is smaller than 1/2
      r' = floor . sqrt . fromIntegral . max 5 $ res
      r = max 0 $ s' + r'
      -- a' is a scaled by 2^k so that 2^(-r') <= a' < 2^(-r'+1)
      a' = (Approx m e (s-r))
      t = taylor
            (res + r)
            (iterate (a'*) 1)
            (scanl1 (*) $ 1:[1..])
  in (!! r) . iterate (boundErrorTerm . sqrA) $ t
   
-- | Exponential by summation of Taylor series.
expTaylorA' :: Precision -> Approx -> Approx
expTaylorA' _ Bottom = Bottom
expTaylorA' res (Approx m e s) =
  let s' = s + integerLog2 m
      -- r' chosen so that a' below is smaller than 1/2
      r' = floor . sqrt . fromIntegral . max 5 $ res
      r = max 0 $ s' + r'
      -- a' is a scaled by 2^k so that 2^(-r') <= a' < 2^(-r'+1)
      a' = (Approx m e (s-r))
      t = taylorA
            (res + r)
            (map (recipA (res+r)) fac)
            a'
  in (!! r) . iterate (boundErrorTerm . sqrA) $ t
   
{- Logarithms computed by ln x = 2*atanh ((x-1)/(x+1)) after range reduction.
-}

{-|

Computing the logarithm of an approximation. This chooses the fastest implementation.

More thorough benchmarking is desirable.

Binary splitting is faster than Taylor. AGM should be used over ~1000 bits.
-}
logA :: Precision -> Approx -> Approx
-- This implementation asks for the dyadic approximation of the endpoints, we
-- should instead use that, after the first range reduction, the derivative is
-- less than 3/2 on the interval, so it easy to just compute one expensive
-- computation. We could even make use of the fact that the derivative on the
-- interval x is bounded by 1/x to get a tighter bound on the error.
logA _ Bottom = Bottom
logA p x@(Approx m e _)
  | m > e = logInternal p x
--    let (n :^ t) = logD (negate p) $ (m-e) :^ s
--        (n' :^ t') = logD (negate p) $ (m+e) :^ s
--    in endToApprox (Finite ((n-1):^t)) (Finite ((n'+1):^t'))
  | otherwise = Bottom

logInternal :: Int -> Approx -> Approx
logInternal _ Bottom = error "LogInternal: impossible"
logInternal p (Approx m e s) =
  let t' = (negate p) - 10 - max 0 (integerLog2 m + s) -- (5 + size of argument) guard digits
      r = s + integerLog2 (3*m) - 1
      x = scale (m :^ s) (-r) -- 2/3 <= x' <= 4/3
      y = divD' t' (x - 1) (x + 1) -- so |y| <= 1/5
      (n :^ s') = flip scale 1 $ atanhD t' y
      (e' :^ s'') = divD' t' (e:^(s-r)) x -- Estimate error term.
      res = Approx n (scale (e' + 1) (s'' - s')) s'
  in boundErrorTerm $ res + fromIntegral r * log2A t'

-- | Logarithm by binary splitting summation of Taylor series.
logBinarySplittingA :: Precision -> Approx -> Approx
logBinarySplittingA _ Bottom = Bottom
logBinarySplittingA res a@(Approx m e s) =
    if m <= e then Bottom -- only defined for strictly positive arguments
    else
        let r = s + integerLog2 (3*m) - 1
            a' = Approx m e (s-r)  -- a' is a scaled by a power of 2 so that 2/3 <= |a'| <= 4/3
            u = a' - 1
            v = a' + 1
            u2 = sqrA u
            v2 = sqrA v
            Finite res' = min (significance a) (Finite res)
            n = ceiling . (/2) $ fromIntegral (-res')/(log 0.2/log 2) - 1
            (_, q, b, t) = abpq (repeat 2)
                                [1,3..]
                                (u:repeat u2)
                                (v:repeat v2)
                                0
                                n
            nextTerm = recipA (res') 5 ^^ (2*n+1)
        in boundErrorTerm $ fudge (t * recipA res (fromIntegral b*q) + fromIntegral r * log2A (-res)) nextTerm

-- | Logarithm by summation of Taylor series.
logTaylorA :: Precision -> Approx -> Approx
logTaylorA _ Bottom = Bottom
logTaylorA res (Approx m e s) =
    if m <= e then Bottom -- only defined for strictly positive arguments
    else
        let res' = res + errorBits
            r = s + integerLog2 (3*m) - 1
            a' = Approx m e (s-r)  -- a' is a scaled by a power of 2 so that 2/3 <= a' <= 4/3
            u = a' - 1
            v = a' + 1
            x = u * recipA (res') v  -- so |u/v| <= 1/5
            x2 = boundErrorTerm $ sqrA x
            t = taylor
                  res'
                  (iterate (x2*) x)
                  [1,3..]
        in boundErrorTerm $ 2 * t + fromIntegral r * log2A (-res')

-- Sine computed by Taylor expansion after 2 stage range reduction.

-- | Computes sine by summation of Taylor series after two levels of range reductions.
sinTaylorA :: Precision -> Approx -> Approx
sinTaylorA res = sinTaylorRed2A res . sinTaylorRed1A res

-- | First level of range reduction for sine. Brings it into the interval [-π/2,π/2].
sinTaylorRed1A :: Precision -> Approx -> Approx
sinTaylorRed1A res a = 
  let _pi = piA res
      _halfPi = scale _pi (-1)
      x = (subtract _halfPi) . abs . (_pi -) . abs . (subtract _halfPi) . modA a $ 2*_pi
  in x
  
-- | Second level of range reduction for sine.
sinTaylorRed2A :: Precision -> Approx -> Approx
sinTaylorRed2A _ Bottom = Bottom
sinTaylorRed2A res a@(Approx m _ s) = 
  let k = max 0 (integerLog2 m + s + (floor . sqrt $ fromIntegral res))
      a' = a * recipA res (3^k)
      a2 = negate $ sqrA a'
      t = taylorA res (map (recipA res) oddFac) a2
      step x = boundErrorTerm $ x * (3 - 4 * sqrA x)
  in limitAndBound res . (!! k) . iterate step . boundErrorTerm $ t * a'

-- | Computes the sine of an approximation. Chooses the best implementation.
sinA :: Precision -> Approx -> Approx
sinA = sinTaylorA

-- | Computes the cosine of an approximation. Chooses the best implementation.
cosA :: Precision -> Approx -> Approx
cosA res x = sinA res ((Approx 1 0 (-1)) * piA res - x)

-- | Computes the sine of an approximation by binary splitting summation of Taylor series.
--
-- Begins by reducing the interval to [0,π/4].
sinBinarySplittingA :: Precision -> Approx -> Approx
sinBinarySplittingA _ Bottom = Bottom
sinBinarySplittingA res a =
    let _pi = piBorweinA res
        (Approx m' e' s') = 4 * a * recipA res _pi
        (k,m1) = m' `divMod` bit (-s')
        a2 = _pi * fromDyadic (1:^(-2)) * (Approx m1 e' s')
    in case k `mod` 8 of
         0 -> sinInRangeA res a2
         1 -> cosInRangeA res (_pi * fromDyadic (1:^(-2)) - a2)
         2 -> cosInRangeA res a2
         3 -> sinInRangeA res (_pi * fromDyadic (1:^(-2)) - a2)
         4 -> - sinInRangeA res a2
         5 -> - cosInRangeA res (_pi * fromDyadic (1:^(-2)) - a2)
         6 -> - cosInRangeA res a2
         7 -> - sinInRangeA res (_pi * fromDyadic (1:^(-2)) - a2)
         _ -> error "Impossible"

-- | Computes the cosine of an approximation by binary splitting summation of Taylor series.
--
-- Begins by reducing the interval to [0,π/4].
cosBinarySplittingA :: Precision -> Approx -> Approx
cosBinarySplittingA _ Bottom = Bottom
cosBinarySplittingA res a =
    let _pi = piBorweinA res
        (Approx m' e' s') = 4 * a * recipA res _pi
        (k,m1) = m' `divMod` bit (-s')
        a2 = _pi * fromDyadic (1:^(-2)) * (Approx m1 e' s')
    in case k `mod` 8 of
         0 -> cosInRangeA res a2
         1 -> sinInRangeA res (_pi * fromDyadic (1:^(-2)) - a2)
         2 -> - sinInRangeA res a2
         3 -> - cosInRangeA res (_pi * fromDyadic (1:^(-2)) - a2)
         4 -> - cosInRangeA res a2
         5 -> - sinInRangeA res (_pi * fromDyadic (1:^(-2)) - a2)
         6 -> sinInRangeA res a2
         7 -> cosInRangeA res (_pi * fromDyadic (1:^(-2)) - a2)
         _ -> error "Impossible"

-- | Computes the arc tangent of an approximation. Chooses the best implementation.
atanA :: Precision -> Approx -> Approx
atanA = atanTaylorA

-- | Computes the arc tangent of an approximation by binary splitting summation of Taylor series.
atanBinarySplittingA :: Precision -> Approx -> Approx
atanBinarySplittingA _ Bottom = Bottom
atanBinarySplittingA res a =
  let rr x = x * recipA res (1 + sqrtA res (1 + sqrA x))
      a' = rr . rr . rr $ a -- range reduction so that |a'| < 1/4
      a2 = - sqrA a'
      res' = case (significance a) of
               (Finite _r) -> min res _r
               _ -> res
--      Finite res' = min (significance a) (Finite res)
      n = (res' + 1) `div` 2
      (_, q, b, t) = abpq ones
                          [1,3..]
                          (a':repeat a2)
                          (repeat 1)
                          0
                          n
      nextTerm = Approx 1 0 (-2*n)
  in boundErrorTerm . (8*) $ fudge (t * recipA res (fromIntegral b*q)) nextTerm

-- + Bottom
-- + Deal with sign -- Because of next line, not worthwhile
-- + if lowerbound(abs a) > 2 then pi/2 - atan (1/a) -- Don't want to do this, what if 0 \in a?
-- + else
--   - r = min res (significance a)
--   - k = floor (sqrt r) / 2 `min` 2 (guarantee |x| < 1/2)
--   - x = rr^k(a)
--   - taylor (r + k + 5) (-x^2) [1,3..]
--   - (x*)
--   - same error as x
--   - (2^k *)

atanTaylorA :: Precision -> Approx -> Approx
atanTaylorA _ Bottom = Bottom
atanTaylorA res a =
  let (Finite r) = min (pure res) (significance a)
      k = min (floor (sqrt (fromIntegral r)) `div` 2) 2
      res' = res + k + 5
      rr _x = _x * recipA res' (1 + sqrtA res' (1 + sqrA _x))
      x = boundErrorTerm $ iterate rr a !! k
      x2 = negate (sqrA x)
      t = boundErrorTerm $ x * taylorA res' (map (recipA res') [1,3..]) x2
  in scale t k

-- > let x = fromDouble (-0.2939788524332769)
-- > require 10 $ x
-- Approx (-5295852201093248) 1 (-54)
-- > require 10 . tan $ atan x
-- Approx (-10845905307838971904) 907 (-65)
-- > scale (-5295852201093248) 11
-- -10845905307838971904
--
-- problemet är att 1 måste skalas till 2^11, men blev bara 907
--
-- Men problemet verkar vara i tan, inte i atan.


-- | Computes the arc tangent of an approximation by summation of Taylor series.
-- atanTaylorA :: Precision -> Approx -> Approx
-- atanTaylorA _ Bottom = Bottom
-- atanTaylorA res a =
--   let rr x = x * recipA res (1 + sqrtA res (1 + sqrA x))
--       a' = rr . rr . rr $ a -- range reduction so that |a'| < 1/4
--       a2 = - sqrA a'
--       res' = case (significance a) of
--                (Finite _r) -> min res _r
--                _ -> res
-- --      Finite res' = min (significance a) (Finite res)
--       t = taylorA res' (map (recipA res') [1,3..]) a2
--   in boundErrorTerm . (8*) $ t

{-
swapSinCos :: Precision -> Approx -> Approx
swapSinCos res a = sqrtA res $ 1 - sqrA a
-}

-- Computes sine if second argument is in the range [0,pi/4]
sinInRangeA :: Precision -> Approx -> Approx
sinInRangeA _ Bottom = Bottom
sinInRangeA res a =
    let n = res `div` 2        -- need to improve this estimate (is valid from res>=80)
        (_, q, b, t) = abpq ones
                            ones
                            (a:repeat (- sqrA a))
                            (1:[2*i*(2*i+1) | i <- [1..]] :: [Approx])
                            0
                            n
        nextTerm = fromDyadic (1:^(-res))
    in boundErrorTerm $ fudge (t * recipA res (fromIntegral b*q)) nextTerm

-- Computes cosine if second argument is in the range [0,pi/4]
cosInRangeA :: Precision -> Approx -> Approx
cosInRangeA _ Bottom = Bottom
cosInRangeA res a =
    let n = res `div` 2        -- need to improve this estimate (is valid from res>=80)
        (_, q, b, t) = abpq ones
                            ones
                            (1:repeat (- sqrA a))
                            (1:[2*i*(2*i-1) | i <- [1..]] :: [Approx])
                            0
                            n
        nextTerm = fromDyadic (1:^(-res))
    in boundErrorTerm $ fudge (t * recipA res (fromIntegral b*q)) nextTerm

{-|
Computes a sequence of approximations of π using binary splitting summation of
Ramanujan's series. See Haible and Papanikolaou 1998.
-}
piRaw :: [Approx]
piRaw = unfoldr f (1, (1, 1, 1, 13591409))
    where as = [13591409,13591409+545140134..]
          bs = ones
          ps = (1:[-(6*n-5)*(2*n-1)*(6*n-1) | n <- [1,2..]])
          qs = (1:[n^3*640320^2*26680 | n <- [1,2..]])
          f (i, (pl, ql, bl, tl)) = 
            let i2 = i*2
                (pr, qr, br, tr) = abpq as bs ps qs i i2
                n = 21+47*(i-1)
                x = fromIntegral tl * recipA n (fromIntegral (bl*ql))
                x1 = fudge x $ fromDyadic (1:^(-n))
                x2 = boundErrorTerm $ sqrtA n 1823176476672000 * recipA n x1
            in Just ( x2
                    , (i2, (pl * pr, ql * qr, bl * br, fromIntegral br * qr * tl + fromIntegral bl * pl * tr))
                    )

-- | Computes an 'Approx' of π of the given precision.
piA :: Precision -> Approx
piA res = limitAndBound res . head $ dropWhile ((< pure res) . precision) piRaw

{-|
Second argument is noice to be added to first argument. Used to allow for the
error term when truncating a series.
-}
fudge :: Approx -> Approx -> Approx
fudge (Approx m 0 s) (Approx m' e' s') =
  Approx (m `shift` (s - s')) (abs m' + e' + 1) s'
fudge (Approx m e s) (Approx m' e' s') =
  let m'' = 1 + (abs m' + e') `shift` (s' - s + 1)
  in Approx m (e+m'') s
fudge _ _  = Bottom

--

-- | Compute π using Machin's formula. Lifted from computation on dyadic numbers.
piMachinA :: Precision -> Approx
piMachinA t = let (m:^s) = piMachinD (-t) in Approx m 1 s

-- | Compute π using AGM as described in Borwein and Borwein's book 'Pi and
-- the AGM'. Lifted from computation on dyadic numbers.
piBorweinA :: Precision -> Approx
piBorweinA t = let (m:^s) = piBorweinD (-t) in Approx m 1 s


-- | Compute π using AGM as described in Borwein and Borwein's book 'Pi and
-- the AGM'.
piAgmA :: Precision -> Approx -> Approx
piAgmA t x = let t' = t - 10
                 a = 1
                 b = boundErrorTerm $ (2*x*recipA (-t') (x^2-1))^2
                 ss = agmA t a b
                 c = boundErrorTerm . (1-) . (*recipA (-t') (1-b^2)) . agm2 . agm1 $ ss
                 d = sqrtA (-t') (1+b)
                 b2 = b^2
                 b3 = b2*b
                 b4 = b2^2
                 l = boundErrorTerm $ (((Approx 1 0 (-1))*b-(Approx 3 0 (-4))*b2+(Approx 9 0 (-5))*b3)*c*d-1/(1+b)+(2+b2)/d) / ((2+(Approx 1 0 (-1))*b2+(Approx 9 0 (-5))*b4)*c+b2)
                 u = boundErrorTerm $ ((Approx 1 0 (-1))*b*c*d-1/(1+b)+(2+b2+(Approx 3 0 (-3))*b3+(Approx 9 0 (-3))*b4)/d) / ((2+(Approx 1 0 (-1))*b2)*c+b2+(Approx 9 0 (-3))*b4)
                 r = boundErrorTerm $ unionA l u
                 e = boundErrorTerm $ unionA
                      ((2+(Approx 1 0 (-1))*b2)*r-(Approx 1 0 (-1))*b*d)
                      ((2+(Approx 1 0 (-1))*b2+(Approx 9 0 (-5))*b4)*r-((Approx 1 0 (-1))*b-(Approx 3 0 (-4))*b2+(Approx 9 0 (-5))*b3)*d)
                 _pi = boundErrorTerm $ unionA (2*(snd (last ss))*e) (2*(fst (last ss))*e)
             in _pi
                
-- | Compute approximations of ln 2. Lifted from computation on dyadic numbers.
log2A :: Precision -> Approx
log2A t = let (m:^s) = ln2D t in Approx m 1 s

-- AGM

-- | Compute logarithms using AGM as described in Borwein and Borwein's book 'Pi and
-- the AGM'. An approximation of pi is produced as a by-product.
lnSuperSizeUnknownPi :: Precision -> Approx -> (Approx,Approx)
lnSuperSizeUnknownPi t x =
    let t' = t - 10
        a = 1
        b = boundErrorTerm $ (2*x*recipA (-t') (x^2-1))^2
        ss = agmA t a b
        (an,bn) = last ss
        c = boundErrorTerm . (1-) . (*recipA (-t') (1-b^2)) . agm2 . agm1 $ ss
        d = sqrtA (-t') (1+b)
        b2 = b^2
        b3 = b2*b
        b4 = b2^2
        l = boundErrorTerm $
             (((Approx 1 0 (-1))*b-(Approx 3 0 (-4))*b2+(Approx 9 0 (-5))*b3)*c*d-1/(1+b)+(2+b2)/d)
             / ((2+(Approx 1 0 (-1))*b2+(Approx 9 0 (-5))*b4)*c+b2)
        u = boundErrorTerm $
             ((Approx 1 0 (-1))*b*c*d-1/(1+b)+(2+b2+(Approx 3 0 (-3))*b3+(Approx 9 0 (-3))*b4)/d)
             / ((2+(Approx 1 0 (-1))*b2)*c+b2+(Approx 9 0 (-3))*b4)
        r = boundErrorTerm $ unionA l u
        e = boundErrorTerm $ unionA
             ((2+(Approx 1 0 (-1))*b2)*r-(Approx 1 0 (-1))*b*d)
             ((2+(Approx 1 0 (-1))*b2+(Approx 9 0 (-5))*b4)*r
              -((Approx 1 0 (-1))*b-(Approx 3 0 (-4))*b2+(Approx 9 0 (-5))*b3)*d)
        _pi = boundErrorTerm $ unionA (2*bn*e) (2*an*e)
    in (r,_pi) --[a,b,c,d,b2,b3,b4,l,u,r,e,pi]

-- | Compute logarithms using AGM as described in Borwein and Borwein's book 'Pi and
-- the AGM'. An approximation of pi is needed as an extra argument.
lnSuperSizeKnownPi :: Precision -> Approx -> Approx -> Approx
lnSuperSizeKnownPi t _pi x =
    let t' = t - 10
        a = 1
        b = boundErrorTerm $ (2*x*recipA (-t') (x^2-1))^2
        b2 = b^2
        b3 = b2*b
        b4 = b2^2
        b1sqrt = sqrtA (-t') (1+b)
        step (_a,_b) = (boundErrorTerm $ Approx 1 0 (-1) * (_a+_b)
                       ,boundErrorTerm $ sqrtA (-t') (_a*_b))
        close (_a,_b) = approximatedBy 0 $ _a-_b
        ((an,bn):_) = dropWhile (not . close) $ iterate step (a,b)
        i = boundErrorTerm $ unionA (_pi*recipA (-t') (2*an)) (_pi*recipA (-t') (2*bn))
        l = (i + ((Approx 1 0 (-1))*b-(Approx 3 0 (-4))*b2+(Approx 9 0 (-5))*b3)*b1sqrt)
            / (2 + (Approx 1 0 (-1))*b2 + (Approx 9 0 (-5))*b4)
        u = (i + (Approx 1 0 (-1))*b*b1sqrt) / (2 + (Approx 1 0 (-1))*b2)
    in boundErrorTerm $ unionA l u

lnLarge :: Precision -> Approx -> Approx
lnLarge t x =
    let (Finite k) = min (significance x) (Finite (-t))
        _pi = piBorweinA t
        iL2 = integerLog2
        fI = fromIntegral
        n = max 0 . (1+) . (+(iL2 (fI k)-2)) . negate . iL2 . fI . iL2 . truncate $ toRational x
        (Approx m e s) = lnSuperSizeKnownPi t _pi $ x^(2^n)
    in Approx m e (s-n)

lnSmall :: Precision -> Approx -> Approx
lnSmall _ Bottom = Bottom
lnSmall t x@(Approx m _ s) =
    let (Finite t') = min (significance x) (Finite (-t))
        _pi = piBorweinA t'
        iL2 = integerLog2
        -- fI = fromIntegral
        k = (-t) `div` 4 - iL2 m - s
        logx2k = lnSuperSizeKnownPi (-t') _pi $ x * 2^k
        log2k = lnSuperSizeKnownPi (-t') _pi $ 2^k
    in logx2k - log2k

-- | Compute logarithms using AGM as described in Borwein and Borwein's book 'Pi and
-- the AGM'.
logAgmA :: Precision -> Approx -> Approx
logAgmA t x
    | significance x < pure 5     = Bottom
    | 0 `approximatedBy` x        = Bottom
    | signum x == (-1)            = error "Trying to take logarithm of purely negative Approx."
    | lowerBound x > pure 2       = lnLarge t x
    | upperBound x < pure 3       = lnSmall t x
    | otherwise                   = error "Logic fault in logAgmA."


agmA :: Precision -> Approx -> Approx -> [(Approx,Approx)]
agmA t a b = let t' = t - 5
                 step (_a,_b) = (boundErrorTerm $ Approx 1 0 (-1) * (a+b), boundErrorTerm $ sqrtA (-t') (_a*_b))
                 close (_a,_b) = approximatedBy 0 $ _a-_b
             in (\(as, bs) -> as ++ take 1 bs) . break close $ iterate step (a,b)

sqDiff :: Approx -> Approx -> Approx
sqDiff a b = boundErrorTerm $ a^2 - b^2

agm1 :: [(Approx, Approx)] -> [Approx]
agm1 = zipWith (*) [Approx 1 0 i | i <- [-1,0..]] . map (uncurry sqDiff)

agm2 :: [Approx] -> Approx
agm2 xs = sum (init xs) + unionA 0 (2 * last xs)

-- | Compute logarithms using AGM as described in Borwein and Borwein's book 'Pi and
-- the AGM'.
agmLn :: Precision -> Approx -> Approx
agmLn t x = let t' = t - 10
                a = 1
                b = boundErrorTerm $ (2*x*recipA (-t') (x^2-1))^2
                ss = agmA t a b
                -- (an,bn) = last ss
                c = boundErrorTerm . (1-) . (*recipA (-t') (1-b^2)) . agm2 . agm1 $ ss
                d = sqrtA (-t') (1+b)
                b2 = b^2
                b3 = b2*b
                b4 = b2^2
--                l = boundErrorTerm $ (((Approx 1 0 (-1))*b-(Approx 3 0 (-4))*b2+(Approx 9 0 (-5))*b3)*c*d-recipA t' (1+b)+(2+b2)*recipA t' d) * recipA t' ((2+(Approx 1 0 (-1))*b2+(Approx 9 0 (-5))*b4)*c+b2)
--                u = boundErrorTerm $ ((Approx 1 0 (-1))*b*c*d-recipA t' (1+b)+(2+b2+(Approx 3 0 (-3))*b3+(Approx 9 0 (-3))*b4)*recipA t' d) *recipA t' ((2+(Approx 1 0 (-1))*b2)*c+b2+(Approx 9 0 (-3))*b4)
                l = boundErrorTerm $ (((Approx 1 0 (-1))*b-(Approx 3 0 (-4))*b2+(Approx 9 0 (-5))*b3)*c*d-1/(1+b)+(2+b2)/d) / ((2+(Approx 1 0 (-1))*b2+(Approx 9 0 (-5))*b4)*c+b2)
                u = boundErrorTerm $ ((Approx 1 0 (-1))*b*c*d-1/(1+b)+(2+b2+(Approx 3 0 (-3))*b3+(Approx 9 0 (-3))*b4)/d) / ((2+(Approx 1 0 (-1))*b2)*c+b2+(Approx 9 0 (-3))*b4)
                r = boundErrorTerm $ unionA l u
                e = boundErrorTerm $ unionA
                      ((2+(Approx 1 0 (-1))*b2)*r-(Approx 1 0 (-1))*b*d)
                      ((2+(Approx 1 0 (-1))*b2+(Approx 9 0 (-5))*b4)*r-((Approx 1 0 (-1))*b-(Approx 3 0 (-4))*b2+(Approx 9 0 (-5))*b3)*d)
                _pi = boundErrorTerm $ unionA (2*(snd (last ss))*e) (2*(fst (last ss))*e)
            in r --[a,b,c,d,b2,b3,b4,l,u,r,e,_pi]
  

-- The CR implementation

type Resources = Int

startLimit :: Int
startLimit = 80

bumpLimit :: Int -> Int
bumpLimit n = n * 3 `div` 2

resources :: ZipList Resources
resources = ZipList $ iterate bumpLimit startLimit

-- Should not use show as it would be impossible to write a corresponding read instance.
-- instance Show CR where
--     show = show . require 40

instance Num CR where
    (CR x) + (CR y) = CR $ (\a b l -> ok (-100) $ limitAndBound l (a + b)) <$> x <*> y <*> resources
    (CR x) * (CR y) = CR $ (\a b l -> ok (-100) $ limitAndBound l (a * b)) <$> x <*> y <*> resources
    negate (CR x) = CR $ negate <$> x
    abs (CR x) = CR $ abs <$> x
    signum (CR x) = CR $ signum <$> x
    fromInteger n = CR $ pure (Approx n 0 0)

instance Fractional CR where
    recip (CR x) = CR $ recipA <$> resources <*> x
    fromRational x = CR $ toApprox <$> resources <*> pure x

instance Eq CR where
  (==) = error "CR does not have a total equality."

instance Ord CR where
  compare = error "CR does not have a total ordering."

instance Real CR where
    toRational = toRational . require 40

-- | Shows the internal representation of a 'CR'. The first /n/
-- approximations are shown on separate lines.
showCRN :: Int -> CR -> String
showCRN n (CR x) = concat . intersperse "\n" . map showA . take n . getZipList $ x

-- | Shows a 'CR' with the desired precision.
showCR :: Int -> CR -> String
showCR p = showA . require p

-- There is no show instance of 'CR' since the representation would be infinite. We can therefore not satisfy (read . show) = id.

-- | Reads a floating point representation of a real number and interprets
-- that as a CR. Does not currently allow for the same format output by
-- 'showCR'.
instance Read CR where
  readsPrec _ input =
    let (intPart, rest) = span isDigit input
    in if null rest || head rest /= '.'
       then [(CR $ pure (Approx (read intPart :: Integer) 0 0), rest)]
       else let (fracPart, rest') = span isDigit (tail rest)
            in [((CR $ pure (Approx (read (intPart ++ fracPart) :: Integer) 0 0)) / 10^(length fracPart), rest')]

-- | Check that an approximation has at least /d/ bits of precision. This is
-- used to bail out of computations where the size of approximation grow
-- quickly.
ok :: Int -> Approx -> Approx
ok d a = if precision a > fromIntegral d then a else Bottom

-- | Given a 'CR' this functions finds an approximation of that number to
-- within the precision required.
require :: Int -> CR -> Approx
require d (CR x) = head . dropWhile (== Bottom) . getZipList $ ok d <$> x

-- | Gives a 'Double' approximation of a 'CR' number.
toDouble :: CR -> Maybe Double
toDouble = toDoubleA . require (54+errorBits)

fromDouble :: Double -> CR
fromDouble x =
  let (m, s) = decodeFloat x
  -- When the mantissa of a floating point is interpreted as a whole number
  -- instead of as a fraction in the IEEE 754 encoding the exponent 972
  -- corresponds to 1024, which is what IEEE 754 use to encode infinity and
  -- NaN.
  in if (s == 972) then CR $ pure Bottom
     else CR $ pure (Approx m 1 s)

fromDoubleAsExactValue :: Double -> CR
fromDoubleAsExactValue x =
  let (m, s) = decodeFloat x
  -- When the mantissa of a floating point is interpreted as a whole number
  -- instead of as a fraction in the IEEE 754 encoding the exponent 972
  -- corresponds to 1024, which is what IEEE 754 use to encode infinity and
  -- NaN.
  in if (s == 972) then CR $ pure Bottom
     else CR $ pure (Approx m 0 s)

transposeZipList :: [ZipList a] -> ZipList [a]
transposeZipList = ZipList . transpose . map getZipList

-- | Evaluate a polynomial, given as a list of its coefficients, at the given point.
polynomial :: [CR] -> CR -> CR
polynomial as (CR x) = 
    CR $ (\as' x' l -> ok 10 . limitAndBound l $ poly as' x') <$> transposeZipList (map unCR as) <*> x <*> resources

-- | Computes the sum of a Taylor series, given as a list of its coefficients,
-- at the given point.
taylorCR :: [CR] -> CR -> CR
taylorCR as (CR x) =
    CR $ (\as' x' l -> sum . takeWhile nonZeroCentredA . map (limitAndBound l) $ zipWith (*) as' (pow x'))
    <$> transposeZipList (map unCR as) <*> x <*> resources

epsilon :: CR
epsilon = CR $ Approx 0 1 . negate <$> resources

-- | The square root function. Lifted from square root application on 'Approx'
-- approximations.
sqrtCR :: CR -> CR
sqrtCR (CR x) = CR $ (\a l -> ok 10 . limitAndBound l $ sqrtA (-l) a) <$> x <*> resources

alternateSign :: Num a => [a] -> [a]
alternateSign = zipWith (*) (cycle [1,-1])

atanCR :: CR -> CR
atanCR x =
  let rr y = y / (1 + sqrt (1 + x^2))
      x' = rr . rr . rr $ x -- range reduction so that |a'| < 1/4
      x2 = - x'^2
      (CR t) = epsilon + x * taylorCR (map ((1/) . fromIntegral) [1,3..]) x2
  in CR $ boundErrorTerm . (8*) <$> t
--  let x2 = x^2
--           in epsilon + x * taylor (map (1/) . alternateSign . map fromInteger $ [1,3..]) x2

-- | π computed using Machin's formula. Computed directly on 'CR'.
piCRMachin :: CR
piCRMachin = 4*(4*atanCR (1/5)-atanCR (1/239))

-- | π computed using Machin's formula. Computed on 'Approx' approximations.
piMachinCR :: CR
piMachinCR = CR $ piMachinA . negate <$> resources

-- | π computed using Borwein's formula. Computed on 'Approx' approximations.
piBorweinCR :: CR
piBorweinCR = CR $ piBorweinA . negate <$> resources

-- | π computed using binary splitting. Computed on 'Approx' approximations.
piBinSplitCR :: CR
piBinSplitCR = CR $ limitAndBound <$> resources <*> (require <$> resources <*> ZipList (repeat (CR $ ZipList piRaw)))

-- | The constant ln 2.
ln2 :: CR
ln2 = CR $ log2A . negate <$> resources

-- | The exponential computed using Taylor's series. Computed directly on
-- 'CR'. Will have poor behaviour on larger inputs as no range reduction is
-- performed.
expCR :: CR -> CR
expCR = (+ epsilon) . taylorCR (map (1/) $ fac)

halfPi :: CR
halfPi = scale pi (-1)

sinRangeReduction :: CR -> CR
sinRangeReduction (CR x) = (subtract halfPi) . abs . (pi -) . abs . (subtract halfPi) . CR $ modA <$> x <*> unCR (2 * pi)

sinRangeReduction2 :: CR -> CR
sinRangeReduction2 (CR x) =
  let k = (\a -> case a of 
                   (Approx m _ s) -> max 0 $ 8 * (integerLog2 m + s + 3) `div` 5
                   Bottom -> 0) <$> x
      (CR y) = sinCRTaylor ((CR x) / (CR $ fromIntegral . (3^) <$> k))
      step z = z*(3-4*z^2)
  in CR $ (\y' k' l -> limitAndBound l $ foldr ($) y' (replicate k' step)) <$> y <*> k <*> resources

-- | The sine function computed using Taylor's series. Computed directly on
-- 'CR'. Will have poor behaviour on larger inputs as no range reduction is
-- performed.
sinCRTaylor :: CR -> CR
sinCRTaylor x = let x2 = x^2
                in epsilon + x * taylorCR (map (1/) $ alternateSign oddFac) x2

-- | The sine function computed using Taylor's series. Computed directly on
-- 'CR'.
sinCR :: CR -> CR
sinCR = sinRangeReduction2 . sinRangeReduction

-- | The cosine function computed using Taylor's series. Computed directly on
-- 'CR'.
cosCR :: CR -> CR
cosCR = sinCR . (halfPi -)

instance Floating CR where
  sqrt (CR x) = CR $ sqrtA <$> resources <*> x
  pi = piBinSplitCR
  exp (CR x) = CR $ expA <$> resources <*> x
  log (CR x) = CR $ logA <$> resources <*> x
  sin (CR x) = CR $ sinA <$> resources <*> x
  cos (CR x) = CR $ cosA <$> resources <*> x
  asin x = 2 * (atan (x / (1 + (sqrt (1 - x^2)))))
  acos x = halfPi - asin x
  atan (CR x) = CR $ atanA <$> resources <*> x
  sinh x = ((exp x) - (exp $ negate x)) / 2
  cosh x = ((exp x) + (exp $ negate x)) / 2
  tanh x = let t = exp (2*x) in (t-1)/(t+1)
  asinh x = log (x + sqrt (x^2 + 1))
  acosh x = CR $ logA <$> resources <*> unCR (x + sqrt (x^2 - 1))
  atanh x = (CR $ logA <$> resources <*> unCR ((1+x) / (1-x))) / 2

-- | Partial comparison operation on 'CR'.
compareCR :: CR -> CR -> ZipList (Maybe Ordering)
compareCR (CR x) (CR y) = compareA <$> x <*> y

{-|
= Non-deterministic operations

Some operations on 'CR' can for theoretical reasons not be made total. For
example, equality, ordering and inverse are all partial functions on 'CR'. To
avoid partiality we may allow non-determinism for parts of the computation. It
would be the programmers responsibility to ensure singlevaluedness if that is
desired.
-}

{-| The ordering on computable reals is not decidable so our orderings of
'Approx' and 'CR' are partial. We can avoid partiality if we accept
non-determinism. The case operations 'caseA' and 'caseCR' allow us to look
through a list of conditions and return the value associated with the first
successful condition. These operations are not monotonic in the ordering of
approximations.
-}

-- | The arguments to 'caseA' should be a list of lists. The second level of
-- lists should have 3 elements each, i.e., argument should have the form
-- [[x1,y1,z1],...,[xn,yn,zn]]. The first i such that xi > yi causes the case
-- statement to return zi. Note that this is non-monotonic since better
-- approximations may make xj > yj for some j < i.
caseA :: [[Approx]] -> Approx
caseA [] = Bottom
caseA ((x:y:z:_) : as) =
  if compareA x y == Just GT
  then z
  else caseA as
caseA (_:_) = Bottom

-- | The arguments to 'caseCR' should be a list of lists. The second level of
-- lists should have 3 elements each, i.e., argument should have the form
-- [[x1,y1,z1],...,[xn,yn,zn]]. Any zi for which xi > yi may be returned.
-- Since 'caseA' is non-monotonic this is as well. Which means that different
-- zi can be approximated in various parts of the sequence of approximations
-- in CR. One way to interpret this behaviour is that the result of this
-- operation is /non-deterministic/.
caseCR :: [[CR]] -> CR
caseCR as =
  CR $ caseA <$> transposeZipList (fmap transposeZipList (fmap (fmap unCR) as))

-- | Data type for returning possible results of a select operation. 'Unknown'
-- is similar to 'Nothing' for 'Maybe'. 'None' confirms that none of the
-- alternatives can be selected.
data Select = Unknown | None | Select Int deriving (Eq,Ord,Read,Show)

-- | Tries to select a positive 'Approx' from a list. If all are negative,
-- then 'None' is returned. If one is positive, then the index of it is
-- returned. Otherwise, there is at least one 'Approx' containing zero so we
-- can only return 'Unknown'.
selectA :: [Approx] -> Select
selectA as = let rs = fmap (`compareA` 0) as
             in if all (== Just LT) rs
                then None
                else maybe Unknown Select $ findIndex (== Just GT) rs

-- | Tries to select a positive 'CR' from a list. If all are negative, then
-- 'None' is returned. If one is positive, then the index of it is returned.
-- Otherwise, there is at least one 'CR' containing zero so the computation
-- will diverge.
selectCR :: [CR] -> Select
selectCR as = head . dropWhile (== Unknown) . getZipList $ selectA <$> transposeZipList (fmap unCR as)

-- | The floor of an 'Approx'.
floorA :: Approx -> Approx
floorA Bottom = Bottom
floorA (Approx m e s) =
  let k = max (-s) 0
      l = Finite . fromInteger $ (m-e) `div` (scale 1 k)
      u = Finite . fromInteger $ (m+e) `div` (scale 1 k)
  in endToApprox l u

-- | The ceiling of an 'Approx'.
ceilingA :: Approx -> Approx
ceilingA Bottom = Bottom
ceilingA (Approx m e s) =
  let k = max (-s) 0
      l = Finite . fromInteger . (+1) $ (m-e-1) `div` (scale 1 k)
      u = Finite . fromInteger . (+1) $ (m+e-1) `div` (scale 1 k)
  in endToApprox l u

floorCR :: CR -> CR
floorCR (CR x) = CR $ floorA <$> x

ceilingCR :: CR -> CR
ceilingCR (CR x) = CR $ ceilingA <$> x

-- | Limit of a sequence of fast converging 'CR's. It is the callers
-- responsibility that 'f n' is within '2^^(-n)' for all n.
--
-- The implementation just follows the diagonal. Since the resource bound
-- grows very quickly within 'CR' it is likely that we index far into this
-- sequence. With the current list implementation this may be slow. We may
-- choose to use a different structure for the sequence or we may have a
-- version that requires faster convergence.
limCR :: (Int -> CR) -> CR
limCR f = CR . ZipList $ [let p = getZipList resources !! n
                          in (getZipList . unCR) (scale unitError (-p) + f p) !! n | n <- [0..]]

unitError :: CR
unitError = CR . pure $ Approx 0 1 0
