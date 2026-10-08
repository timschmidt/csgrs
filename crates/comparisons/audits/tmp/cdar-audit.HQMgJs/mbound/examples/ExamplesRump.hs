{-# LANGUAGE NoMonomorphismRestriction #-}
{-|
Two examples given to me by S Rump that challenge the precision of floating
point computations.

The first example gives the wrong sign with Double arithmetic, but even the
first approximation of a CReal is correct.

The second example gives the correct value with Double, but we can see that it
is difficult as it takes some extra precision to produce the right result.

The usage of showCRealN is to show the underlying computations; it is not
intended way to use CReal.
-}
module ExamplesRump (examplesRump) where

import Data.CDAR

a :: Num t => t
a = 77617
b :: Num t => t
b = 33096
p :: Fractional a => a
p = 206987/2048
q :: Fractional a => a
q = 119504/2048
r :: Fractional a => a
r = p^^3*(p^^16+6561*q^^16-17496*p^^2*q^^14+20412*p^^4*q^^12-13608*p^^6*q^^10+5670*p^^8*q^^8-1512*p^^10*q^^6+252*p^^12*q^^4-24*p^^14*q^^2) - q

examplesRump :: IO ()
examplesRump = do
  print $ 21*b*b-2*a*a+55*b*b*b*b-10*a*a*b*b+a/(2*b)
  putStrLn . showCRN 3 $ 21*b*b-2*a*a+55*b*b*b*b-10*a*a*b*b+a/(2*b)
  print r
  putStrLn . showCRN 3 $ r
  
