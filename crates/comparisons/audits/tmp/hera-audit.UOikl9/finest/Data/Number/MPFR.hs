{-# LANGUAGE TypeSynonymInstances #-}
{-# INCLUDE <mpfr.h> #-}
{-# INCLUDE <hsmpfr.h> #-}
module Data.Number.MPFR (
-- | This module should always be imported qualified.

-- *** Naming 
-- | - functions ending with _ return a pair (value, rounding indicator). 
--     Rounding indicator indicates whether the result is rounded and in which
--     directon as described in the MPFR manual.
--
-- - the same functions without the _ return just the value. 
--
-- - functions with added \"w\" correspond to MPFR _ui functions
--
-- - functions with added \"i\" correspond to MPFR _si functions


-- *** Equality testing
-- | Equality works as follows: 
-- 
--   - NaN \/= Nan, 
--
--   - Infinity = Infinity, 
--
--   - \-Infinity = -Infinity
--
--   - otherwise normal comparison 

-- *** Ordering      
-- | Ordering works as follows:
-- 
--   - compare NaN _ = GT
--
--   - compare _ NaN = GT
--
--   - infinity < _ = False
--
--   - \-infinity > _ = False
--
--   - NaN [\<,\>,\>=,<=] _ = False
--
--   This mimics the behaviour of built in haskell Float and Double.

-- *** Num instance
-- | Operations defined in Num will be computed so that no precision is lost.

  Dyadic,
  Precision,
  RoundMode(Near, Zero, Up, Down),
  add, sub, mul, div, inverse,
  add_, sub_, mul_, div_,
  addw, addi, mulw, muli, divw, divi, wdiv, idiv, subw, subi, wsub, isub,
  addw_, addi_, mulw_, muli_, divw_, divi_, wdiv_, idiv_, subw_, subi_, wsub_, isub_,
  mul2w, mul2i, div2w, div2i, mul2w_, mul2i_, div2w_, div2i_,
  int2i, int2w, int2i_, int2w_,
  fma, fms, fma_, fms_, nextBelow,
  sqr, sqrt, root, pow, poww, powi, wpoww, wpow, 
  sqr_, sqrt_, root_, pow_, poww_, powi_, wpoww_, wpow_,
  exp, exp2, exp10, log, log2, log10, sinh, cosh, tanh,
  exp_, exp2_, exp10_, log_, log2_, log10_, sinh_, cosh_, tanh_,
  neg, absD, dim, neg_, absD_, dim_, 
  isNaN, isInfinite, isNumber, isZero, greater, greatereq, less, lesseq,
  equal, maxD, minD, maxD_, minD_, sgn, 
  dyadicToDouble, dyadicToWord, dyadicToInt, dyadicToString, decompose, toStringExp, toString,
  pi, log2c, euler, catalan, pi_, log2c_, euler_, catalan_,
  set, set_,
  fromDouble, fromInt, fromWord, fromDouble_, fromInt_, fromWord_, fromIntegerA, compose, fromString,
  getPrec, getMantissa, getExp, 
  minPrec, one, zero, addPrec
) where



import Data.Number.FFIhelper

import Foreign.C(CInt, CLong, CULong, withCString, peekCString)
import Foreign.Marshal(alloca, peekArray)
import Foreign(peek, Ptr, mallocForeignPtrBytes, with)
import System.IO.Unsafe(unsafePerformIO)

import Data.Bits(shiftL)

import Data.Word(Word)
import Prelude hiding (div, sqrt, read, isNaN, isInfinite, exp, log, sinh, cosh, tanh, pi)

type Dyadic = MPFR_T

type Precision = Word


-- these are helper functions, only for internal use
{-# INLINE withDyadicsBA #-}
withDyadicsBA                     :: RoundMode -> Precision -> Dyadic -> Dyadic
                                     -> (Ptr MPFR_T -> Ptr MPFR_T -> Ptr MPFR_T -> CRoundMode -> IO CInt)
                                     -> (Dyadic, Int)
withDyadicsBA r p mp1 mp2 f = unsafePerformIO go
    where go = do ls <- mpfr_custom_get_size (fromIntegral p)
                  fp <- mallocForeignPtrBytes (fromIntegral ls)
                  let dummy = MP (fromIntegral p) 0 0 fp
                  with dummy $ \p1 -> do
                    with mp1 $ \p2 -> do
                      with mp2 $ \p3 -> do
                          r2 <- f p1 p2 p3 ((fromIntegral . fromEnum) r)
                          r1 <- peekP p1 fp
                          return (r1, fromIntegral r2)

{-# INLINE withDyadicBAui #-}
withDyadicBAui             :: RoundMode -> Precision -> Dyadic -> CULong
                              ->  (Ptr MPFR_T -> Ptr MPFR_T -> CULong -> CRoundMode -> IO CInt)
                              -> (Dyadic, Int) 
withDyadicBAui r p mp1 d f = unsafePerformIO go
    where go = do ls <- mpfr_custom_get_size (fromIntegral p)
                  fp <- mallocForeignPtrBytes (fromIntegral ls)
                  let dummy = MP (fromIntegral p) 0 0 fp
                  with dummy $ \p1 -> do
                    with mp1 $ \p2 -> do
                      r2 <- f p1 p2 d ((fromIntegral . fromEnum) r)
                      r1 <- peekP p1 fp
                      return (r1, fromIntegral r2)
                                
{-# INLINE withDyadicBAsi #-}
withDyadicBAsi            :: RoundMode -> Precision -> Dyadic -> CLong
                             -> (Ptr MPFR_T -> Ptr MPFR_T -> CLong -> CRoundMode -> IO CInt)
                             -> (Dyadic, Int)
withDyadicBAsi r p mp1 d f = unsafePerformIO go 
    where go = do ls <- mpfr_custom_get_size (fromIntegral p)
                  fp <- mallocForeignPtrBytes (fromIntegral ls)
                  let dummy = MP (fromIntegral p) 0 0 fp
                  with dummy $ \p1 -> do
                    with mp1 $ \p2 -> do
                      r2 <- f p1 p2 d ((fromIntegral . fromEnum) r)
                      r1 <- peekP p1 fp
                      return (r1, fromIntegral r2)
                                  
{-# INLINE withDyadicBAiu #-}
withDyadicBAiu            :: RoundMode -> Precision -> CULong -> Dyadic
                             -> (Ptr MPFR_T -> CULong -> Ptr MPFR_T -> CRoundMode -> IO CInt)
                             -> (Dyadic, Int) 
withDyadicBAiu r p d mp1 f = unsafePerformIO go 
    where go = do ls <- mpfr_custom_get_size (fromIntegral p)
                  fp <- mallocForeignPtrBytes (fromIntegral ls)
                  let dummy = MP (fromIntegral p) 0 0 fp
                  with dummy $ \p1 -> do
                    with mp1 $ \p2 -> do
                      r2 <- f p1 d p2 ((fromIntegral . fromEnum) r)
                      r1 <- peekP p1 fp
                      return (r1, fromIntegral r2)

{-# INLINE withDyadicBAis #-}
withDyadicBAis             :: RoundMode -> Precision -> CLong -> Dyadic
                              -> (Ptr MPFR_T -> CLong -> Ptr MPFR_T -> CRoundMode -> IO CInt)
                              -> (Dyadic, Int) 
withDyadicBAis r p d mp1 f = unsafePerformIO go
    where go = do ls <- mpfr_custom_get_size (fromIntegral p)
                  fp <- mallocForeignPtrBytes (fromIntegral ls)
                  let dummy = MP (fromIntegral p) 0 0 fp
                  with dummy $ \p1 -> do
                    with mp1 $ \p2 -> do
                      r2 <- f p1 d p2 ((fromIntegral . fromEnum) r)
                      r1 <- peekP p1 fp
                      return (r1, fromIntegral r2)

{-# INLINE withDyadicB #-}
withDyadicB :: Dyadic -> (Ptr MPFR_T -> IO CInt) -> CInt 
withDyadicB mp1 f = unsafePerformIO go
    where go = with mp1 $ \p1 -> f p1

withDyadicP :: Dyadic -> (Ptr MPFR_T -> IO CPrecision) -> CPrecision 
withDyadicP mp1 f = unsafePerformIO go
    where go = with mp1 $ \p1 -> f p1

{-# INLINE withDyadic #-}
withDyadic           :: RoundMode -> Precision -> Dyadic 
                        -> (Ptr MPFR_T -> Ptr MPFR_T -> CRoundMode -> IO CInt) 
                        -> (Dyadic, Int)
withDyadic r p mp1 f = unsafePerformIO go 
    where go = do ls <- mpfr_custom_get_size (fromIntegral p)
                  fp <- mallocForeignPtrBytes (fromIntegral ls)
                  let dummy = MP (fromIntegral p) 0 0 fp
                  with dummy $ \p1 -> do
                    with mp1 $ \p2 -> do
                      r2 <- f p1 p2 ((fromIntegral . fromEnum) r)
                      r1 <- peekP p1 fp
                      return (r1, fromIntegral r2)
                  
{-# INLINE withDyadicBB #-}
withDyadicBB           :: Dyadic -> Dyadic 
                          -> (Ptr MPFR_T -> Ptr MPFR_T -> IO CInt) 
                          -> CInt  
withDyadicBB mp1 mp2 f = unsafePerformIO go
    where go = do with mp1 $ \p1 -> do 
                    with mp2 $ \p2 -> do 
                                      f p1 p2
                              
{-# INLINE withDyadicC #-}
withDyadicC       :: RoundMode -> Precision ->
                     (Ptr MPFR_T -> CRoundMode -> IO CInt) -> (Dyadic, Int)
withDyadicC r p f = unsafePerformIO go
    where go = do ls <- mpfr_custom_get_size (fromIntegral p)
                  fp <- mallocForeignPtrBytes (fromIntegral ls)
                  let dummy = MP (fromIntegral p) 0 0 fp
                  with dummy $ \p1 -> do
                    r2 <- f p1 ((fromIntegral . fromEnum) r)
                    r1 <- peekP p1 fp
                    return (r1, fromIntegral r2)
   
checkPrec :: Precision -> Precision
checkPrec = max minPrec

stringToDyadic       :: RoundMode -> Precision -> Word -> String -> Dyadic
stringToDyadic r p b d = unsafePerformIO go
    where go = do ls <- mpfr_custom_get_size (fromIntegral p)
                  fp <- mallocForeignPtrBytes (fromIntegral ls)
                  let dummy = MP (fromIntegral p) 0 0 fp
                  with dummy $ \p1 -> do 
                    withCString d $ \p2 -> do 
                      _ <- mpfr_set_str p1 p2 (fromIntegral b) ((fromIntegral . fromEnum) r) 
                      peekP p1 fp


getMantissa'     :: Dyadic -> [Limb]
getMantissa' mp1 = unsafePerformIO go
    where go = do with mp1 $ \p1 -> do 
                    pt <- mpfr_custom_get_mantissa p1 
                    arr <- peekArray (ceiling ((fromIntegral p ::Double) / fromIntegral bitsPerMPLimb)) pt ;
                    return arr 
          p = getPrec mp1

{- TODO: this is inefficient 
binprec   :: Integer -> Precision
binprec i = length (takeWhile (/= 0) (iterate (flip shiftR 1) i)
-}

binprec   :: Integer -> Precision
binprec d = floor (logBase 2 (fromInteger (if d >= 0 then d else -d)) :: Double) + 1


--------------------------------------------------------------------

-- pure wrappers for basic arithmetic operations

add           :: RoundMode -> Precision -> Dyadic -> Dyadic -> Dyadic
add r p d1 d2 = fst $ add_ r p d1 d2 

sub           :: RoundMode -> Precision -> Dyadic -> Dyadic -> Dyadic
sub r p d1 d2 = fst $ sub_ r p d1 d2

mul           :: RoundMode -> Precision -> Dyadic -> Dyadic -> Dyadic
mul r p d1 d2 = fst $ mul_ r p d1 d2

div           :: RoundMode -> Precision -> Dyadic -> Dyadic -> Dyadic
div r p d1 d2 = fst $ div_ r p d1 d2

add_           :: RoundMode -> Precision -> Dyadic -> Dyadic -> (Dyadic,Int)
add_ r p d1 d2 =  withDyadicsBA r p d1 d2 mpfr_add

sub_           :: RoundMode -> Precision -> Dyadic -> Dyadic -> (Dyadic,Int)
sub_ r p d1 d2 =  withDyadicsBA r p d1 d2 mpfr_sub

mul_           :: RoundMode -> Precision -> Dyadic -> Dyadic -> (Dyadic,Int)
mul_ r p d1 d2 =  withDyadicsBA r p d1 d2 mpfr_mul

div_           :: RoundMode -> Precision -> Dyadic -> Dyadic -> (Dyadic, Int)
div_ r p d1 d2 =  withDyadicsBA r p d1 d2 mpfr_div


inverse :: Dyadic -> Dyadic
inverse d = div Near (getPrec d) one d 

----------------------------------------------------------------

-- basic arithmetic operations with mixed operands

addw          :: RoundMode -> Precision -> Dyadic -> Word -> Dyadic
addw r p d1 d = fst $ addw_ r p d1 d 

addi          :: RoundMode -> Precision -> Dyadic -> Int -> Dyadic
addi r p d1 d = fst $ addi_ r p d1 d 

mulw          :: RoundMode -> Precision -> Dyadic -> Word -> Dyadic
mulw r p d1 d = fst $ mulw_ r p d1 d 

muli          :: RoundMode -> Precision -> Dyadic -> Int -> Dyadic
muli r p d1 d = fst $ muli_ r p d1 d 

divw          :: RoundMode -> Precision -> Dyadic -> Word -> Dyadic
divw r p d1 d = fst $ divw_ r p d1 d 

divi          :: RoundMode -> Precision -> Dyadic -> Int -> Dyadic
divi r p d1 d = fst $ divi_ r p d1 d 

wdiv          :: RoundMode -> Precision -> Word -> Dyadic -> Dyadic
wdiv r p d d1 = fst $ wdiv_ r p d d1 

idiv          :: RoundMode -> Precision -> Int -> Dyadic -> Dyadic
idiv r p d d1 = fst $ idiv_ r p d d1 

subw          :: RoundMode -> Precision -> Dyadic -> Word -> Dyadic
subw r p d1 d = fst $ subw_ r p d1 d 

subi          :: RoundMode -> Precision -> Dyadic -> Int -> Dyadic
subi r p d1 d = fst $ subi_ r p d1 d 

wsub          :: RoundMode -> Precision -> Word -> Dyadic -> Dyadic
wsub r p d d1 = fst $ wsub_ r p d d1 

isub          :: RoundMode -> Precision -> Int -> Dyadic -> Dyadic
isub r p d d1 = fst $ isub_ r p d d1 

addw_          :: RoundMode -> Precision -> Dyadic -> Word -> (Dyadic, Int)
addw_ r p d1 d = withDyadicBAui r p d1 (fromIntegral d) mpfr_add_ui

addi_          :: RoundMode -> Precision -> Dyadic -> Int -> (Dyadic, Int)
addi_ r p d1 d = withDyadicBAsi r p d1 (fromIntegral d) mpfr_add_si

mulw_          :: RoundMode -> Precision -> Dyadic -> Word -> (Dyadic, Int)
mulw_ r p d1 d = withDyadicBAui r p d1 (fromIntegral d) mpfr_mul_ui

muli_          :: RoundMode -> Precision -> Dyadic -> Int -> (Dyadic, Int)
muli_ r p d1 d = withDyadicBAsi r p d1 (fromIntegral d) mpfr_mul_si

divw_          :: RoundMode -> Precision -> Dyadic -> Word -> (Dyadic, Int)
divw_ r p d1 d = withDyadicBAui r p d1 (fromIntegral d) mpfr_div_ui

divi_          :: RoundMode -> Precision -> Dyadic -> Int -> (Dyadic, Int)
divi_ r p d1 d = withDyadicBAsi r p d1 (fromIntegral d) mpfr_div_si

wdiv_          :: RoundMode -> Precision -> Word -> Dyadic -> (Dyadic, Int)
wdiv_ r p d d1 = withDyadicBAiu r p (fromIntegral d) d1 mpfr_ui_div

idiv_          :: RoundMode -> Precision -> Int -> Dyadic -> (Dyadic, Int)
idiv_ r p d d1 = withDyadicBAis r p (fromIntegral d) d1 mpfr_si_div

subw_          :: RoundMode -> Precision -> Dyadic -> Word -> (Dyadic, Int)
subw_ r p d1 d = withDyadicBAui r p d1 (fromIntegral d) mpfr_sub_ui

subi_          :: RoundMode -> Precision -> Dyadic -> Int -> (Dyadic, Int)
subi_ r p d1 d = withDyadicBAsi r p d1 (fromIntegral d) mpfr_sub_si

wsub_          :: RoundMode -> Precision -> Word -> Dyadic -> (Dyadic, Int)
wsub_ r p d d1 = withDyadicBAiu r p (fromIntegral d) d1 mpfr_ui_sub

isub_          :: RoundMode -> Precision -> Int -> Dyadic -> (Dyadic, Int)
isub_ r p d d1 = withDyadicBAis r p (fromIntegral d) d1 mpfr_si_sub

----------------------------------------------------------

-- multiplication and division with 2 ^ x

mul2w           :: RoundMode -> Precision -> Dyadic -> Word -> Dyadic
mul2w r p d1 d2 = fst $ mul2w_ r p d1 d2

mul2i          :: RoundMode -> Precision -> Dyadic -> Int -> Dyadic
mul2i r p d1 d2 = fst $ mul2i_ r p d1 d2

div2w          :: RoundMode -> Precision -> Dyadic -> Word -> Dyadic
div2w r p d1 d2 = fst $ div2w_ r p d1 d2

div2i          :: RoundMode -> Precision -> Dyadic -> Int -> Dyadic
div2i r p d1 d2 = fst $ div2i_ r p d1 d2

mul2w_           :: RoundMode -> Precision -> Dyadic -> Word -> (Dyadic, Int)
mul2w_ r p d1 d2 = withDyadicBAui r p d1 (fromIntegral d2) mpfr_mul_2ui

mul2i_          :: RoundMode -> Precision -> Dyadic -> Int -> (Dyadic, Int)
mul2i_ r p d1 d2 = withDyadicBAsi r p d1 (fromIntegral d2) mpfr_mul_2si

div2w_          :: RoundMode -> Precision -> Dyadic -> Word -> (Dyadic, Int)
div2w_ r p d1 d2 = withDyadicBAui r p d1 (fromIntegral d2) mpfr_div_2ui

div2i_          :: RoundMode -> Precision -> Dyadic -> Int -> (Dyadic, Int)
div2i_ r p d1 d2 = withDyadicBAsi r p d1 (fromIntegral d2) mpfr_div_2si

----------------------------------------------------------

-- x * 2 ^ y
int2i         :: RoundMode -> Precision -> Int -> Int -> Dyadic
int2i r p i e = fst $ int2i_ r p i e

int2w         :: RoundMode -> Precision -> Word -> Int -> Dyadic
int2w r p i e = fst $ int2w_ r p i e

int2i_         :: RoundMode -> Precision -> Int -> Int -> (Dyadic, Int)
int2i_ r p i e = unsafePerformIO go
    where go = do ls <- mpfr_custom_get_size (fromIntegral p)
                  fp <- mallocForeignPtrBytes (fromIntegral ls)
                  let dummy = MP (fromIntegral p) 0 0 fp
                  with dummy $ \p1 -> do
                    r2 <- mpfr_set_si_2exp p1 (fromIntegral i) (fromIntegral e) ((fromIntegral . fromEnum) r)
                    r1 <- peekP p1 fp
                    return (r1, fromIntegral r2)

int2w_         :: RoundMode -> Precision -> Word -> Int -> (Dyadic, Int)
int2w_ r p i e = unsafePerformIO go
    where go = do ls <- mpfr_custom_get_size (fromIntegral p)
                  fp <- mallocForeignPtrBytes (fromIntegral ls)
                  let dummy = MP (fromIntegral p) 0 0 fp
                  with dummy $ \p1 -> do
                    r2 <- mpfr_set_ui_2exp p1 (fromIntegral i) (fromIntegral e) ((fromIntegral . fromEnum) r)
                    r1 <- peekP p1 fp
                    return (r1, fromIntegral r2)
             
----------------------------------------------------------


-- Return d1 * d2 + d3
fma              :: RoundMode -> Precision -> Dyadic -> Dyadic -> Dyadic -> Dyadic
fma r p d1 d2 d3 = fst $ fma_ r p d1 d2 d3

-- Return d1 * d2 - d3 
fms              :: RoundMode -> Precision -> Dyadic -> Dyadic -> Dyadic -> Dyadic
fms r p d1 d2 d3 = fst $ fms_ r p d1 d2 d3


fma_                 :: RoundMode -> Precision -> Dyadic -> Dyadic -> Dyadic -> (Dyadic, Int)
fma_ r p mp1 mp2 mp3 = unsafePerformIO go
    where go = do ls <- mpfr_custom_get_size (fromIntegral p)
                  fp <- mallocForeignPtrBytes (fromIntegral ls)
                  let dummy = MP (fromIntegral p) 0 0 fp
                  with dummy $ \p1 -> do
                    with mp1 $ \p2 -> do 
                      with mp2 $ \p3 -> do 
                        with mp3 $ \p4 -> do 
                          r2 <- mpfr_fma p1 p2 p3 p4 ((fromIntegral . fromEnum) r) 
                          r1 <- peekP p1 fp 
                          return (r1, fromIntegral r2)


fms_                 :: RoundMode -> Precision -> Dyadic -> Dyadic -> Dyadic -> (Dyadic, Int)
fms_ r p mp1 mp2 mp3 = unsafePerformIO go
    where go = do ls <- mpfr_custom_get_size (fromIntegral p)
                  fp <- mallocForeignPtrBytes (fromIntegral ls)
                  let dummy = MP (fromIntegral p) 0 0 fp
                  with dummy $ \p1 -> do
                    with mp1 $ \p2 -> do 
                      with mp2 $ \p3 -> do 
                        with mp3 $ \p4 -> do 
                          r2 <- mpfr_fms p1 p2 p3 p4 ((fromIntegral . fromEnum) r) 
                          r1 <- peekP p1 fp
                          return (r1, fromIntegral r2)

nextBelow     :: Dyadic -> Dyadic
nextBelow mp1 = unsafePerformIO go
    where go = do let p = fromIntegral (getPrec mp1)
                  ls <- mpfr_custom_get_size p
                  fp <- mallocForeignPtrBytes (fromIntegral ls)
                  let dummy = MP p 0 0 fp
                  with dummy $ \p1 -> do
                      with mp1 $ \p2 -> do 
                        _ <- mpfr_set p1 p2 ((fromIntegral . fromEnum) Near) 
                        mpfr_nextbelow p1 
                        peekP p1 fp

----------------------------------------------------------
-- powers

sqr       :: RoundMode -> Precision -> Dyadic -> Dyadic 
sqr r p d = fst $ sqr_ r p d

sqrt       :: RoundMode -> Precision -> Dyadic -> Dyadic
sqrt r p d = fst $ sqrt_ r p d

root         :: RoundMode -> Precision -> Dyadic -> Word -> Dyadic
root r p d n = fst $ root_ r p d n

pow           :: RoundMode -> Precision -> Dyadic -> Dyadic -> Dyadic
pow r p d1 d2 = fst $ pow_ r p d1 d2 

poww           :: RoundMode -> Precision -> Dyadic -> Word -> Dyadic 
poww r p d1 d2 = fst $ poww_ r p d1 d2

powi           :: RoundMode -> Precision -> Dyadic -> Int -> Dyadic 
powi r p d1 d2 = fst $ powi_ r p d1 d2

wpoww           :: RoundMode -> Precision -> Word -> Word -> Dyadic 
wpoww r p d1 d2 = fst $ wpoww_ r p d1 d2

wpow           :: RoundMode -> Precision -> Word -> Dyadic -> Dyadic 
wpow r p d1 d2 = fst $ wpow_ r p d1 d2

sqr_       :: RoundMode -> Precision -> Dyadic -> (Dyadic, Int)
sqr_ r p d = withDyadic r p d mpfr_sqr

sqrt_       :: RoundMode -> Precision -> Dyadic -> (Dyadic, Int)
sqrt_ r p d = withDyadic r p d mpfr_sqrt
 
root_        :: RoundMode -> Precision -> Dyadic -> Word -> (Dyadic, Int)
root_ r p d n = withDyadicBAui r p d (fromIntegral n) mpfr_root

pow_          :: RoundMode -> Precision -> Dyadic -> Dyadic -> (Dyadic, Int)
pow_ r p d1 d2 = withDyadicsBA r p d1 d2 mpfr_pow 

poww_          :: RoundMode -> Precision -> Dyadic -> Word -> (Dyadic , Int)
poww_ r p d1 d2 = withDyadicBAui r p d1 (fromIntegral d2) mpfr_pow_ui

powi_           :: RoundMode -> Precision -> Dyadic -> Int -> (Dyadic , Int)
powi_ r p d1 d2 = withDyadicBAsi r p d1 (fromIntegral d2) mpfr_pow_si

wpoww_          :: RoundMode -> Precision -> Word -> Word -> (Dyadic , Int)
wpoww_ r p d1 d2 = unsafePerformIO go
    where go = do ls <- mpfr_custom_get_size (fromIntegral p)
                  fp <- mallocForeignPtrBytes (fromIntegral ls)
                  let dummy = MP (fromIntegral p) 0 0 fp
                  with dummy $ \p1 -> do 
                    r2 <- mpfr_ui_pow_ui p1 (fromIntegral d1) (fromIntegral d2) ((fromIntegral . fromEnum) r)
                    r1 <- peekP p1 fp
                    return (r1, fromIntegral r2)
        
wpow_           :: RoundMode -> Precision -> Word -> Dyadic -> (Dyadic , Int)
wpow_ r p d1 d2 = withDyadicBAiu r p (fromIntegral d1) d2 mpfr_ui_pow

-----------------------------------------------------------

-- transcendental functions

exp       :: RoundMode -> Precision -> Dyadic -> Dyadic
exp r p d = fst $ exp_ r p d

exp2       :: RoundMode -> Precision -> Dyadic -> Dyadic
exp2 r p d = fst $ exp2_ r p d

exp10       :: RoundMode -> Precision -> Dyadic -> Dyadic
exp10 r p d = fst $ exp10_ r p d

log       :: RoundMode -> Precision -> Dyadic -> Dyadic
log r p d = fst $ log_ r p d

log2       :: RoundMode -> Precision -> Dyadic -> Dyadic
log2 r p d = fst $ log2_ r p d

log10       :: RoundMode -> Precision -> Dyadic -> Dyadic
log10 r p d = fst $ log10_ r p d

sinh       :: RoundMode -> Precision -> Dyadic -> Dyadic
sinh r p d = fst $ sinh_ r p d

cosh       :: RoundMode -> Precision -> Dyadic -> Dyadic
cosh r p d = fst $ cosh_ r p d

tanh       :: RoundMode -> Precision -> Dyadic -> Dyadic
tanh r p d = fst $ tanh_ r p d 

exp_       :: RoundMode -> Precision -> Dyadic -> (Dyadic, Int)
exp_ r p d = withDyadic r p d mpfr_exp

exp2_       :: RoundMode -> Precision -> Dyadic -> (Dyadic, Int)
exp2_ r p d = withDyadic r p d mpfr_exp2

exp10_       :: RoundMode -> Precision -> Dyadic -> (Dyadic, Int)
exp10_ r p d = withDyadic r p d mpfr_exp10

log_       :: RoundMode -> Precision -> Dyadic -> (Dyadic, Int)
log_ r p d = withDyadic r p d mpfr_log

log2_       :: RoundMode -> Precision -> Dyadic -> (Dyadic, Int)
log2_ r p d = withDyadic r p d mpfr_log2

log10_       :: RoundMode -> Precision -> Dyadic -> (Dyadic, Int)
log10_ r p d = withDyadic r p d mpfr_log10

sinh_       :: RoundMode -> Precision -> Dyadic -> (Dyadic, Int)
sinh_ r p d = withDyadic r p d mpfr_sinh

cosh_       :: RoundMode -> Precision -> Dyadic -> (Dyadic, Int)
cosh_ r p d = withDyadic r p d mpfr_cosh

tanh_       :: RoundMode -> Precision -> Dyadic -> (Dyadic, Int)
tanh_ r p d = withDyadic r p d mpfr_tanh

------------------------------------------------------------

neg       :: RoundMode -> Precision -> Dyadic -> Dyadic
neg r p d = fst $ neg_ r p d

absD      :: RoundMode -> Precision -> Dyadic -> Dyadic 
absD r p d = fst $ absD_ r p d

dim           :: RoundMode -> Precision -> Dyadic -> Dyadic -> Dyadic
dim r p d1 d2 = fst $ dim_ r p d1 d2 

neg_       :: RoundMode -> Precision -> Dyadic -> (Dyadic, Int)
neg_ r p d = withDyadic r p d mpfr_neg

absD_      :: RoundMode -> Precision -> Dyadic -> (Dyadic , Int)
absD_ r p d = withDyadic r p d mpfr_abs

dim_           :: RoundMode -> Precision -> Dyadic -> Dyadic -> (Dyadic, Int)
dim_ r p d1 d2 = withDyadicsBA r p d1 d2 mpfr_dim

--------------------------------------------------------

--  comparison functions

isNaN   :: Dyadic -> Bool
isNaN d = withDyadicB d mpfr_nan_p /= 0

isInfinite   :: Dyadic -> Bool
isInfinite d = withDyadicB d mpfr_inf_p /= 0 

isNumber   :: Dyadic -> Bool
isNumber d = withDyadicB d mpfr_number_p /= 0 

isZero   :: Dyadic -> Bool
isZero d = withDyadicB d mpfr_zero_p /= 0

greater       :: Dyadic -> Dyadic -> Bool
greater d1 d2 = withDyadicBB d1 d2 mpfr_greater_p /= 0

greatereq       :: Dyadic -> Dyadic -> Bool
greatereq d1 d2 = withDyadicBB d1 d2 mpfr_greaterequal_p /= 0

less       :: Dyadic -> Dyadic -> Bool
less d1 d2 = withDyadicBB d1 d2 mpfr_less_p /= 0

lesseq       :: Dyadic -> Dyadic -> Bool
lesseq d1 d2 = withDyadicBB d1 d2 mpfr_lessequal_p /= 0

{- sets erange flag
lessgreater       :: Dyadic -> Dyadic -> Bool
lessgreater d1 d2 = withDyadicBB d1 d2 mpfr_lessgreater_p /= 0
-}

equal       :: Dyadic -> Dyadic -> Bool
equal d1 d2 = withDyadicBB d1 d2 mpfr_equal_p /= 0


maxD           :: RoundMode -> Precision -> Dyadic -> Dyadic -> Dyadic
maxD r p d1 d2 = fst $ maxD_ r p d1 d2

minD           :: RoundMode -> Precision -> Dyadic -> Dyadic -> Dyadic
minD r p d1 d2 = fst $ minD_ r p d1 d2

maxD_           :: RoundMode -> Precision -> Dyadic -> Dyadic -> (Dyadic, Int)
maxD_ r p d1 d2 = withDyadicsBA r p d1 d2 mpfr_max

minD_           :: RoundMode -> Precision -> Dyadic -> Dyadic -> (Dyadic, Int)
minD_ r p d1 d2 = withDyadicsBA r p d1 d2 mpfr_min

sgn   :: Dyadic -> Int 
sgn d = case compare zero d of
          LT -> 1
          EQ -> 0
          _  -> -1

--conversion from dyadics to basic haskell types

dyadicToDouble         :: RoundMode -> Dyadic -> Double
dyadicToDouble r mp1 = (realToFrac . unsafePerformIO) go
                         where go = with mp1 $ \p -> mpfr_get_d p ((fromIntegral . fromEnum) r)

dyadicToWord         :: RoundMode -> Dyadic -> Word
dyadicToWord r mp1 = (fromIntegral . unsafePerformIO) go
                       where go = with mp1 $ \p -> mpfr_get_ui p ((fromIntegral . fromEnum) r)

dyadicToInt     :: RoundMode -> Dyadic -> Int
dyadicToInt r mp1 = (fromIntegral . unsafePerformIO) go
                       where go = with mp1 $ \p -> mpfr_get_si p ((fromIntegral . fromEnum) r)

dyadicToString         :: RoundMode -> Word -- ^ number of significant digits 
                                    -> Word -- ^ base 
                                    -> Dyadic -> (String, Int)
dyadicToString r n b mp1 = unsafePerformIO go 
    where go = with mp1 $ \p1 -> do 
                 alloca $ \p2 -> do 
                   withCString (replicate (fromIntegral (n + 2)) '0') $ \p3 -> do 
                     _ <- mpfr_get_str p3 p2 (fromIntegral b) (fromIntegral n) p1 ((fromIntegral . fromEnum) r)
                     r1 <- peekCString p3 
                     r2 <- peek p2  
                     return (r1, fromIntegral r2) 

decompose   :: Dyadic -> (Integer, Int)
decompose d = (getMantissa d, getExp d - ceiling (fromIntegral (getPrec d) / fromIntegral bitsPerMPLimb :: Double) * bitsPerMPLimb)


toStringExp       :: Word -> Dyadic -> String
toStringExp dec d = s ++ case e > 0 of
                           True  -> case floor (logBase 10 2 * fromIntegral (getExp d) :: Double) > dec  of
                                      False -> take e ss ++ let bt = backtrim (drop e ss) in if null bt then "" else "." ++ bt
                                      True  -> head ss : "." ++ let bt = (backtrim . tail) ss in if null bt then "0"
                                                                                                   else bt ++ "e" ++ show (pred e)
                           False -> head ss : "." ++ (let bt = (backtrim . tail) ss in
                                                     if null bt then "0" 
                                                       else bt )
                                                  ++ "e" ++ show (pred e)
                    where (str, e) = dyadicToString Near n 10 d
                          n        = max dec 5
                          (s, ss) = case head str of
                                      '-' -> ("-", tail str)
                                      _   -> ("" , str)
                          backtrim = reverse . dropWhile (== '0') . reverse 

toString       :: Word -> Dyadic -> String
toString dec d = s ++ case compare 0 e of
                         LT -> take e ss ++ (let bt = all (== '0') (drop e ss) in if bt then "" else '.' : (drop e ss))
                               ++ (if fromIntegral n - e < 0 then "e" ++ show (e - fromIntegral n) else "")
                         GT -> let ee = fromIntegral dec + e in 
                               if ee <= 0 then "0" else 
                                   head ss : "." ++ (backtrim . tail . take ee) ss ++ "e" ++ show (pred e)
                         EQ -> "0." ++ let bt = all (== '0') ss in if bt then "0" else ss
                  where (str, e) = dyadicToString Near n 10 d
                        n        = max dec 5
                        (s, ss) = case head str of
                                    '-' -> ("-", tail str)
                                    _   -> ("" , str)
                        backtrim = reverse . dropWhile (== '0') . reverse 

--asignment functions                              

set           :: RoundMode -> Precision -> Dyadic -> Dyadic
set r p d = fst $ set_ r p d


set_         :: RoundMode -> Precision -> Dyadic -> (Dyadic, Int)
set_ r p mp1 = unsafePerformIO go
    where go = do ls <- mpfr_custom_get_size (fromIntegral p)
                  fp <- mallocForeignPtrBytes (fromIntegral ls)
                  let dummy = MP (fromIntegral p) 0 0 fp
                  with dummy $ \p1 -> do
                    with mp1 $ \p2 -> do 
                      r2 <- mpfr_set p1 p2 ((fromIntegral . fromEnum) r) 
                      r1 <- peekP p1 fp
                      return (r1, fromIntegral r2)
 
------------------------------------
-- mpfr constants
pi :: RoundMode -> Precision -> Dyadic
pi r p = fst $ pi_ r p

log2c     :: RoundMode -> Precision -> Dyadic
log2c r p = fst $ pi_ r p

euler     :: RoundMode -> Precision -> Dyadic
euler r p = fst $ pi_ r p

catalan     :: RoundMode -> Precision -> Dyadic
catalan r p = fst $ pi_ r p

pi_     :: RoundMode -> Precision -> (Dyadic, Int)
pi_ r p = withDyadicC r p mpfr_const_pi

log2c_     :: RoundMode -> Precision -> (Dyadic, Int)
log2c_ r p = withDyadicC r p mpfr_const_log2

euler_     :: RoundMode -> Precision -> (Dyadic, Int)
euler_ r p = withDyadicC r p mpfr_const_euler

catalan_     :: RoundMode -> Precision -> (Dyadic, Int)
catalan_ r p = withDyadicC r p mpfr_const_catalan

----------------------------------------------------------
-- conversion from basic haskell types to dyadics

fromDouble       :: RoundMode -> Precision -> Double -> Dyadic
fromDouble r p d = fst $ fromDouble_ r p d

fromInt       :: RoundMode -> Precision -> Int -> Dyadic
fromInt r p d = fst $ fromInt_ r p d

fromWord       :: RoundMode -> Precision -> Word -> Dyadic
fromWord r p d = fst $ fromWord_ r p d

fromDouble_       :: RoundMode -> Precision -> Double -> (Dyadic, Int)
fromDouble_ r p d = unsafePerformIO go
    where go = do ls <- mpfr_custom_get_size (fromIntegral p)
                  fp <- mallocForeignPtrBytes (fromIntegral ls)
                  let dummy = MP (fromIntegral p) 0 0 fp
                  with dummy $ \p1 -> do 
                    r2 <- mpfr_set_d p1 (realToFrac d) ((fromIntegral . fromEnum) r)
                    r1 <- peekP p1 fp
                    return (r1, fromIntegral r2)

fromInt_       :: RoundMode -> Precision -> Int -> (Dyadic, Int)
fromInt_ r p d = unsafePerformIO go
    where go = do ls <- mpfr_custom_get_size (fromIntegral p)
                  fp <- mallocForeignPtrBytes (fromIntegral ls)
                  let dummy = MP (fromIntegral p) 0 0 fp
                  with dummy $ \p1 -> do 
                    r2 <- mpfr_set_si p1 (fromIntegral d) ((fromIntegral . fromEnum) r)
                    r1 <- peekP p1 fp
                    return (r1, fromIntegral r2)


fromWord_       :: RoundMode -> Precision -> Word -> (Dyadic, Int)
fromWord_ r p d = unsafePerformIO go
    where go = do ls <- mpfr_custom_get_size (fromIntegral p)
                  fp <- mallocForeignPtrBytes (fromIntegral ls)
                  let dummy = MP (fromIntegral p) 0 0 fp
                  with dummy $ \p1 -> do 
                    r2 <- mpfr_set_ui p1 (fromIntegral d) ((fromIntegral . fromEnum) r)
                    r1 <- peekP p1 fp
                    return (r1, fromIntegral r2)                  


fromIntegerA       :: RoundMode -> Precision -> Integer -> Dyadic
fromIntegerA r p d = stringToDyadic r p 10 (show d)

compose             :: RoundMode -> Precision -> (Integer, Int) -> Dyadic 
compose r p (i, ii) = div2i r p (fromIntegerA r p i) ii

fromString       :: String -> Precision -> Word -> Dyadic
fromString s p b = stringToDyadic Near p b s

---------------------------------------------------------

-- functions getting properties of dyadics

getPrec   :: Dyadic -> Precision
getPrec d = fromIntegral (withDyadicP d mpfr_get_prec)

-- | getMantissa and getExp return values such that
--
-- > d = getMantissa d * 2^(getExp d - ceiling ((getPrec d) / bitsPerMPLimb)* bitsPerMPLimb )
getMantissa   :: Dyadic -> Integer
getMantissa d = if d < zero then -h else h
               where (h, _) = foldl (\(a,b) c -> (a + (toInteger c) `shiftL` b, b + bitsPerMPLimb)) (0,0) (getMantissa' d) 

getExp   :: Dyadic -> Int
getExp d = (fromIntegral . unsafePerformIO) go
                 where go = do with d $ \p1 -> 
                                mpfr_custom_get_exp p1

--------------------------------------------------------

-- some constants
minPrec :: Precision
minPrec = 32

one ::  Dyadic              
one = fromWord Near minPrec 1

zero :: Dyadic              
zero = fromWord Near minPrec 0

-- instances

instance Eq Dyadic where
    (==) = equal

instance Ord Dyadic where
    (<)  = less
    (<=) = lesseq
    (>)  = greater
    (>=) = greatereq
                     
instance Show Dyadic where
    show = toStringExp 16

-- these are exact operations, without rounding
instance Num Dyadic where
    d + d' = add Zero (addPrec d d') d d'
    d - d' = sub Zero (addPrec d d') d d'
    d * d' = mul Zero (getPrec d + getPrec d') d d'
    negate d = neg Zero (getPrec d) d 
    signum d = case compare d zero of 
                 LT -> negate one
                 EQ -> zero
                 _  -> one
    abs d = absD Zero (getPrec d) d
    fromInteger i = fromIntegerA Zero (checkPrec $ binprec i) i

addPrec       :: Dyadic -> Dyadic -> Precision
addPrec d1 d2 = fromIntegral (max (p1 + e1 - e3) (p2 + e2 - e3)) + 1
                where e1 = if d1 == 0 then 0 else getExp d1
                      e2 = if d2 == 0 then 0 else getExp d2
                      p1 = fromIntegral $ getPrec d1
                      p2 = fromIntegral $ getPrec d2
                      e3 = min e1 e2

{-
addPrec d1 d2 = max e1 e2 + 1 - min (e1 - p1) (p2 - e2)
                where e1 = if d1 == 0 then 0 else fromIntegral $ getExp d1
                      e2 = if d2 == 0 then 0 else fromIntegral $ getExp d2
                      p1 = getPrec d1
                      p2 = getPrec d2
-}

