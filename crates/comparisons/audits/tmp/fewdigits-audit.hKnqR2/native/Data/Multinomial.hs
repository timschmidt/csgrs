{-# OPTIONS_GHC -fglasgow-exts #-}
module Data.Multinomial 
                   (Polynomial, xP, yP, zP, constP, o,
                    evalP, degree,
                    BoundPolynomial, boundPolynomial,
                    derive, dx, dy, dz,
                    Zero, One, Two, zero, one, two,
                    VectorQ, (<+>), (<*>)) where

import Data.Interval
import Combinatorics
import Control.Exception
import Test.QuickCheck hiding (choose,two)

infixr 7 <*>
infixl 6 <+>

newtype Polynomial a = Poly [a] deriving (Show, Arbitrary)

polyNormalize [] = []
polyNormalize (a:x) | a==(fromInteger 0) && (null rec) = []
                    | otherwise = a:rec
 where
  rec = polyNormalize x

instance (Num a) => Eq (Polynomial a) where
 (Poly p) == (Poly q) = (polyNormalize p) == (polyNormalize q)

degree (Poly p) = length (polyNormalize p)

constP x = Poly [x]

polyPlus [] y = y
polyPlus x [] = x
polyPlus (a:x) (b:y) = (a+b):(polyPlus x y)

polyNegate x = map negate x

polyMult _ [] = []
polyMult [] _ = []
polyMult (a:x) y = polyPlus (map (a*) y) (0:(polyMult x y))

(Poly []) `o` q = Poly []
(Poly (a:p)) `o` q = (constP a) + q*((Poly p) `o` q)

evalP (Poly []) _ = (fromInteger 0)
evalP (Poly (a:p)) x = a+x*(evalP (Poly p) x)


instance (Num a) => Num (Polynomial a) where
 (Poly a) + (Poly b) = Poly (polyPlus a b)
 (Poly a) * (Poly b) = Poly (polyMult a b)
 negate (Poly a) = Poly (polyNegate a)
 abs = error "abs not implemented for polynomials"
 signum = error "signum not implemented for polynomials"
 fromInteger 0 = Poly []
 fromInteger x = constP $ fromInteger x

xP :: (Num a) => Polynomial a
xP = Poly [(fromInteger 0), (fromInteger 1)]

yP :: (Num a) => Polynomial (Polynomial a)
yP = constP xP

zP :: (Num a) => Polynomial (Polynomial (Polynomial a))
zP = constP yP

prop_test1 = (xP^2 + xP*yP - yP^2) == Poly [Poly [0,0,-1], Poly [0,1], Poly [1]]

bernstein :: (Fractional a) => Interval a -> Int -> Int -> Polynomial a
bernstein (Interval (a,b)) k i = 
   (constP $ (fromInteger (k `choose` i))/(b-a)^k)
  *(xP-constP a)^i*(constP b-xP)^(k-i)

toBernsteinIJ k i j = (fromInteger $ product [j'-i'+1..j'])
                      /(fromInteger $ product [k'-i'+1..k'])
 where
  i' = fromIntegral i
  j' = fromIntegral j
  k' = fromIntegral k

toBernsteinMatrix k = [[toBernsteinIJ k i j| j <- [0..k]] | i <- [0..k]]

toBernstein :: (VectorQ a) => Polynomial a -> [a]
toBernstein (Poly p) = 
  foldr (zipWith (<+>)) (replicate (length p) zeroVector) $
  zipWith toBernsteinzXs p [0..]
 where
  -- Find the bernstein basis for z*x^s
  toBernsteinzXs z s = [(toBernsteinIJ k s j)<*>z | j <- [0..k]]
  k = fromIntegral $ (length p)-1

toGeneralBernstein :: (VectorQ a, Num a) => Interval Rational -> Polynomial a -> [a]
toGeneralBernstein (Interval (a,b)) p = toBernstein $
  p `o` ((b-a)<*>xP <+> a<*>1)

class BoundPolynomial a b where
 boundPolynomial :: a -> b -> Interval Rational

instance BoundPolynomial (Interval Rational) (Polynomial Rational) where
 boundPolynomial _ (Poly []) = point 0
 boundPolynomial i p = Interval (minimum cs, maximum cs)
  where
   cs = toGeneralBernstein i p

instance (BoundPolynomial a b, VectorQ b, Num b) => 
         BoundPolynomial ((Interval Rational),a) (Polynomial b) where
 boundPolynomial _ (Poly []) = point 0
 boundPolynomial (i,j) p = unionList $ 
   map (boundPolynomial j) $ toGeneralBernstein i p

prop_bound :: (Polynomial Rational) -> (Interval Rational) -> Rational -> Property
prop_bound p i x = (x `isIn` i) ==> (y `isIn` j)
 where
  y = evalP p x
  j = boundPolynomial i p

prop_bound2 :: (Polynomial (Polynomial Rational)) ->
               (Interval Rational) -> (Interval Rational) ->
               Rational -> Rational -> Property
prop_bound2 p i j x y = (x `isIn` i) && (y `isIn` j) ==> (z `isIn` k)
 where
  z = evalP (evalP p (constP x)) y
  k = boundPolynomial (i,j) p

{- The deriviate code is written by apfelmus@dialin-145-254-186-070.pools.arcor-ip.net
   and on 2006-06-02T12:58:48Z he donated the code to the public domain 
   <http://tunes.org/~nef/logs/haskell/06.06.02> -}
        -- derivatives
data Zero = Zero
data Succ a = Succ a

type One = Succ Zero
type Two = Succ One

zero = Zero
one = Succ zero
two = Succ one

-- derive :: Two -> Poly ..
--                derives with resp. to the 2nd outermost variable

class Derive xi ring where
        derive :: xi -> ring -> ring

instance Num a => Derive Zero (Polynomial a) where
        derive _ (Poly xs) = Poly [(fromInteger k)*a | (k,a)<- zip [1..] (tail xs)]

instance Derive a b => Derive (Succ a) (Polynomial b) where
        derive (Succ a) (Poly xs) = Poly $ map (derive a) xs

dx :: (Num b) => Polynomial b -> Polynomial b 
dx = derive zero
dy :: (Num b) => Polynomial (Polynomial b) -> Polynomial (Polynomial b)
dy = derive one
dz :: (Num b) => Polynomial (Polynomial (Polynomial b)) -> Polynomial (Polynomial (Polynomial b))
dz = derive two

-- test
type Poly2 = Polynomial (Polynomial Int)

f :: Poly2
f = xP*xP*yP
prop_test_dx = dx f == 2*xP*yP
prop_test_dy = dy f == xP*xP
prop_test_dxy = dx (dy f) == dy (dx f)

class VectorQ a where
 (<+>) :: a -> a -> a
 (<*>) :: Rational -> a -> a
 zeroVector :: a

instance VectorQ Rational where
 (<+>) = (+)
 q <*> x = q * x
 zeroVector = fromInteger 0

instance VectorQ Double where
 (<+>) = (+)
 q <*> x = (fromRational q) * x
 zeroVector = fromInteger 0

instance (VectorQ a, Num a) => VectorQ (Polynomial a) where
 (<+>) = (+)
 q <*> (Poly p) = Poly $ map (q<*>) p
 zeroVector = fromInteger 0