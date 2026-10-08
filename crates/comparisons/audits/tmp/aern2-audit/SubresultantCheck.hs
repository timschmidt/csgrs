{-# LANGUAGE NoRebindableSyntax, CPP #-}
module Main where
import Prelude
import Control.Monad (forM_)
import qualified Data.Map as Map
import qualified AERN2.Poly.Power.Type as P
import qualified AERN2.Poly.Basics as C
#ifdef USE_MAP
import qualified AERN2.Poly.Power.SignedSubresultantMap as S
#else
import qualified AERN2.Poly.Power.SignedSubresultantVector as S
#endif
import System.Environment (getArgs)
import System.IO (hSetBuffering, stdout, BufferMode(LineBuffering))

trim :: [Rational] -> [Rational]
trim = reverse . dropWhile (==0) . reverse
degree :: [Rational] -> Int
degree = subtract 1 . length . trim
minus :: [Rational] -> [Rational] -> [Rational]
minus a b = trim (zipWith (-) (a++replicate (n-length a) 0) (b++replicate (n-length b) 0))
  where n = max (length a) (length b)
divide :: [Rational] -> [Rational] -> ([Rational],[Rational])
divide a b = go [] (trim a)
  where go q r | degree r < degree b = (q,r)
               | otherwise = let k=degree r-degree b; c=last r/last b
                                 t=replicate k 0++[c]
                             in go (minus q (map negate t)) (minus r (replicate k 0++map (c*) b))
monic :: [Rational] -> [Rational]
monic a = case trim a of [] -> []; cs -> map (/last cs) cs
gcdPoly :: [Rational] -> [Rational] -> [Rational]
gcdPoly a b | null (trim b) = monic a
            | otherwise = gcdPoly b (snd (divide a b))
deriv :: [Rational] -> [Rational]
deriv a = trim (zipWith (*) (map fromIntegral [1 :: Int ..]) (drop 1 a))
times :: [Integer] -> [Integer] -> [Integer]
times a b = [sum [x*y | (i,x) <- zip [0..] a,(j,y) <- zip [0..] b,i+j==k]
            | k <- [0..length a+length b-2]]
toDonor :: [Integer] -> P.PowPoly Integer
toDonor = P.fromList . zip [0..]
fromDonor :: P.PowPoly Integer -> [Rational]
fromDonor (P.PowPoly (C.Poly cs)) =
  [fromInteger (Map.findWithDefault 0 i cs) | i <- [0..fst (Map.findMax cs)]]
cases :: [[Integer]]
cases = [[-1,0,1],[-2,0,1],[1,1,1],[1,-3,2]] ++
  [foldl times [1] (replicate n [-1,1]++replicate m [2,1]) | n <- [1..3],m <- [1..3]] ++
  [times (times [1,0,1] [1,0,1]) (times [-1,1] [-1,1]),
   times [-1,0,0,1] [-1,0,0,1]]

main :: IO ()
main = do
  hSetBuffering stdout LineBuffering
  args <- getArgs
  let indices = if null args then [0..length cases-1] else map read args
  forM_ indices $ \i -> do
    let input=cases!!i; p=map fromInteger input; dp=deriv p
        targetGcd=gcdPoly p dp
        targetFree=monic (fst (divide p targetGcd))
        donor=toDonor input
        (g,free)=S.gcdAndgcdFreePart donor (P.derivative donor)
    putStrLn ("case "++show i++" gcd/free/separable "++show
      (monic (fromDonor g)==targetGcd, monic (fromDonor free)==targetFree,
       monic (fromDonor (S.separablePart donor))==targetFree))
