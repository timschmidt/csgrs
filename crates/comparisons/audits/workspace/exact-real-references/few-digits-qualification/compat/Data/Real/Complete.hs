{-# OPTIONS_GHC -fglasgow-exts #-}
module Data.Real.Complete 
                (Gauge, Complete, unit, join, mapC, bind, mapC2,
                 (:=>), mkUniformCts, modulus, forgetUniformCts,
                 constCts, o) where

infixr 9 :=>

type Gauge = Rational 

-- Intended that d (f x) (f y) <= x + y
type Complete a = Gauge -> a

unit :: a -> Complete a
unit x eps = x

join :: (Complete (Complete a)) -> Complete a
join f eps = (f (eps/2)) (eps/2)

-- A uniformly continuous function on some subset of a to b
-- Hopefully the name of the function gives an indication of
-- the domain.
data a :=> b = UniformCts 
               {modulus :: (Gauge -> Gauge)
               ,forgetUniformCts :: (a -> b)}

mkUniformCts = UniformCts

mapC :: (a :=> b) -> Complete a -> Complete b
mapC (UniformCts mu f) x eps = f (x (mu eps))

bind :: (a :=> Complete b) -> Complete a -> Complete b
bind f x = join $ mapC f x

mapC2 :: (a :=> b :=> c) -> Complete a -> Complete b -> Complete c
mapC2 f x y eps = (mapC approxf y) (eps/2)
 where
  approxf = (mapC f x) (eps/2)

o :: (b :=> c) -> (a :=> b) -> (a :=> c)
f `o` g = mkUniformCts mu h
 where
  mu = (modulus g) . (modulus f)
  h = (forgetUniformCts f) . (forgetUniformCts g)

constCts :: a -> b :=> a
constCts a = mkUniformCts (const undefined) (const a)