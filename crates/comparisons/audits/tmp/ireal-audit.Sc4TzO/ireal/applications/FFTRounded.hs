{-# LANGUAGE PolyKinds #-}
module FFTRounded where

import ListNumSyntax
import Data.Complex
import Data.Number.IReal.Rounded

-- All functions in this file take a precision argument for (less in-)efficiency.
-- To get d correct decimals in result, slightly higher precision 
-- argument should be used.

-- basic fast Fourier transform, for power of 2 lengths only
fft :: (RealFloat a) => [Complex a] -> [Complex a]
fft [] = error "fft: empty list"
fft [a] = [a]
fft [a,b] = [a+b,a-b]
fft as = interleave (fft cs) (fft ds)
  where
    n = length as
    (ls,rs) = splitAt (n `div` 2)  as

    a = -2*pi/fromIntegral n
    tws = [ cis (a * fromIntegral k) | k <-  [0..n `div` 2 - 1] ]

    cs =  ls + rs
    ds =  tws * (ls - rs)

    interleave [] bs = bs
    interleave (a : as) bs = a : interleave bs as

-- inverse FFT
ifft :: (RealFloat a) => [Complex a] -> [Complex a]
ifft as = (fft . rev) as / repeat n
  where n = fromIntegral (length as)
        rev (a : as) = a : reverse as

-- type 1 discrete cosine transform (for lengths 2^n + 1)
dct :: (RealFloat a) => [a] -> [a]
dct as =  (map ((/2) . realPart) . take (length as) . fft . map (:+ 0)) as'
   where as' = as ++ tail (reverse (tail as))

-- inverse DCT
idct :: (RealFloat a) => [a] -> [a]
idct cs = dct cs / repeat n'
   where n' = fromIntegral ((length cs - 1) `div` 2)

-- unscaled type 1 DCT 
dctu [x0,x1] = [x0+x1,x0-x1]
dctu xs = dct (eDouble xs)
  where eDouble (x:xs) = 2*x : t xs
        t [x] = [2*x]
        t (x:xs) = x : t xs

-- its inverse
idctu d [c0,c1] = [(c0+c1)/2,(c0-c1)/2]
idctu d xs = eHalve (idct xs)
   where eHalve (x:xs) = x/2 : t xs
         t [x] = [x/2]
         t (x:xs) = x : t xs

-- discrete sine transform (for lengths 2^n - 1)
dst as =  (map ((/2) . negate . imagPart) . tail . take (length as+1) . fft . map (:+ 0)) as'
   where as' = 0 : as ++ 0 : map negate (reverse as)

-- its inverse
idst cs = dst cs / repeat n'
  where n' =  fromIntegral ((length cs + 1) `div` 2)

--instance VarPrec a => VarPrec (Complex a) where
--  precB p (a :+ b) = precB p a :+ precB p b
 
-- All the instances below are dubious and present only since Haskell requires
-- (for dubious reasons) that inorder to define an instance of Num  for Complex a,
-- a must be an instance of RealFloat. And we do want to define the FFT...

instance Precision p => Real (Rounded p) where
  toRational (R x) = toRational x

instance Precision p => RealFrac (Rounded p) where
  properFraction (R x) =(i,r y) 
    where (i,y) = properFraction x

instance Precision p => RealFloat (Rounded p) where
   floatRadix = undefined
   floatDigits = undefined
   floatRange = undefined
   decodeFloat = undefined
   encodeFloat = undefined
   isNaN = undefined
   isInfinite = undefined
   isDenormalized = undefined
   isNegativeZero = undefined
   isIEEE = undefined
