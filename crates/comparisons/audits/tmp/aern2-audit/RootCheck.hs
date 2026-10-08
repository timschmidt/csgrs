{-# LANGUAGE NoRebindableSyntax #-}
module Main where
import Prelude
import Control.Monad (forM_)
import Data.List (nub, sort)
import Data.Ratio ((%), numerator, denominator)
import qualified Data.Map as Map
import qualified Data.Vector as Vec
import qualified AERN2.Poly.Power.Type as P
import qualified AERN2.Poly.Power.RootsIntVector as V
import qualified AERN2.Poly.Power.RootsIntMap as A
import qualified AERN2.Interval as I
import qualified AERN2.MP as B
import System.IO (hSetBuffering, stdout, BufferMode(LineBuffering))

choose :: Int -> Int -> Rational
choose n k = fromInteger (product [toInteger (n-k+1)..toInteger n] `div` product [1..toInteger k])

-- Independent closed-form power -> Bernstein conversion, not a donor affine
-- transform or de Casteljau recurrence.
oracle :: [Integer] -> Rational -> Rational -> [Rational]
oracle cs l r = [sum [powerAt j * choose i j / choose d j | j <- [0..i]] | i <- [0..d]]
  where d = length cs-1
        powerAt j = sum [fromInteger (cs!!k) * choose k j * l^(k-j) * (r-l)^j | k <- [j..d]]

coeffs :: Int -> Int -> [Integer]
coeffs d s = [if k == d then toInteger (1+s `mod` 7) else toInteger ((s*17+k*11) `mod` 19-9) | k <- [0..d]]
poly :: [Integer] -> P.PowPoly Integer
poly cs = P.fromList (zip [0..] cs)
vcoeffs :: V.Terms -> [Rational]
vcoeffs (_,scale,cs) = map (%scale) (Vec.toList cs)
mcoeffs :: A.Terms -> [Rational]
mcoeffs (_,scale,cs) = map (%scale) (Map.elems cs)
variations :: [Rational] -> Integer
variations xs = toInteger (length [() | (a,b) <- zip signs (drop 1 signs), a /= b])
  where signs = map signum (filter (/=0) xs)
report :: String -> [Bool] -> IO ()
report label bs = putStrLn (label ++ ": " ++ show (length (filter id bs)) ++ "/" ++ show (length bs))

times :: [Integer] -> [Integer] -> [Integer]
times a b = [sum [x*y | (i,x) <- zip [0..] a, (j,y) <- zip [0..] b, i+j == k]
             | k <- [0..length a+length b-2]]
fromRoots :: [Rational] -> [Integer]
fromRoots = foldl times [1] . map (\r -> [-numerator r,denominator r])

main :: IO ()
main = do
  hSetBuffering stdout LineBuffering
  let intervals = [(-2,3),(0,1),(-1%2,7%5),(1,2),(2,3),(5%2,3),(-4,-3)]
      cases = [(coeffs d s,l,r) | d <- [0..10],s <- [0..11],(l,r) <- intervals]
      zero = B.errorBound (0 :: Integer)
      vv cs l r = V.initialBernsteinCoefs (poly cs) zero l r
      aa cs l r = A.initialBernsteinCoefs (poly cs) zero l r
  report "vector initial Bernstein" [vcoeffs (vv cs l r) == oracle cs l r | (cs,l,r) <- cases]
  report "map initial Bernstein" [mcoeffs (aa cs l r) == oracle cs l r | (cs,l,r) <- cases]
  report "declared positive vector multiplier" [c > 0 | (cs,l,r) <- cases, let (_,c,_) = vv cs l r]
  report "declared positive map multiplier" [c > 0 | (cs,l,r) <- cases, let (_,c,_) = aa cs l r]
  report "vector exact sign variation" [V.signVars (vv cs l r) == Just (variations (oracle cs l r)) | (cs,l,r) <- cases]
  report "map exact sign variation" [A.signVars (aa cs l r) == Just (variations (oracle cs l r)) | (cs,l,r) <- cases]
  report "vector subdivision+extrapolation"
    [vcoeffs bl == oracle cs l m && vcoeffs br == oracle cs m r
     | (cs,l,r) <- cases,t <- [1%3,3%2],let m = l+t*(r-l),
       let (bl,br) = V.bernsteinCoefs l r m (vv cs l r)]
  report "map subdivision+extrapolation"
    [mcoeffs bl == oracle cs l m && mcoeffs br == oracle cs m r
     | (cs,l,r) <- cases,t <- [1%3,3%2],let m = l+t*(r-l),
       let (bl,br) = A.bernsteinCoefs l r m (aa cs l r)]
  -- Constant uncertainty delta shifts every Bernstein coefficient by delta.
  -- Any claimed certain variation must hold for these exact included values.
  let e = B.errorBound (1%2 :: Rational)
  report "nonzero-radius vector certain variations on included shifts"
    [maybe True (\n -> n == variations (map (+delta) (oracle cs l r)))
                (V.signVars (V.initialBernsteinCoefs (poly cs) e l r))
     | (cs,l,r) <- cases,delta <- [-1%2,0,1%2]]
  report "nonzero-radius map certain variations on included shifts"
    [maybe True (\n -> n == variations (map (+delta) (oracle cs l r)))
                (A.signVars (A.initialBernsteinCoefs (poly cs) e l r))
     | (cs,l,r) <- cases,delta <- [-1%2,0,1%2]]
  let rootCases = [("interior",rs,(-2,2)) | rs <- [[-1,0,1],[-3%2,-1%2,1%2,3%2],
                                                 [1%3,2%3],[-1%3,-1%3,2%3],[0,0,0]]] ++
                  [("endpoint",rs,(0,1)) | rs <- [[0,1%4],[0,3%4],[0,1%4,3%4],
                                                 [0,0,1%4],[1%4,1],[0,1%4,1]]] ++
                  [("no real roots",[],(-1,1))]
      okay (I.Interval l r) = r-l <= 1%64
  forM_ (zip [0 :: Int ..] rootCases) $ \(idx,(label,rs,(l,r))) -> do
    let cs = if label == "no real roots" then [1,0,1] else fromRoots rs
        target = nub [x | x <- rs,l < x && x < r]
        expectedIn out = and [any (\(I.Interval a b) -> a <= x && x <= b) out | x <- target]
        out = V.findRoots (poly cs) okay l r
        outWith = map fst (V.findRootsWithEvaluation (poly cs) (\(I.Interval a b) -> b-a) (<=1%64) l r)
        inDomain = all (\(I.Interval a b) -> l <= a && a <= b && b <= r)
        ordered xs = let starts = [a | I.Interval a _ <- xs] in starts == sort starts
    putStrLn ("cover " ++ show idx ++ " " ++ label ++ " " ++ show
      (length out, expectedIn out, all okay out, inDomain out, ordered out,expectedIn outWith))
