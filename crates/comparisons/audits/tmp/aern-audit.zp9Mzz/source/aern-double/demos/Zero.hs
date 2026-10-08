
module Main where 

import Numeric.AERN.DoubleBasis.RealIntervalApprox
import Numeric.AERN.DoubleBasis.MRealIntervalApprox

import Control.Monad.ST (ST, runST)

type RI = RealIntervalApprox

main =
  do
  putStrLn $ "zero   0.1 (\\x -> x^2-0.5) (1 </\\> 2) = " ++ 
    show (zero 0.1 (\x -> x^2-0.5) (1 </\> 2))  
  putStrLn $ "zero   0.1 (\\x -> x^2-0.5) (0 </\\> 1) = " ++ 
    show (zero 0.1 (\x -> x^2-0.5) (0 </\> 1))
  putStrLn $ "zero  0.01 (\\x -> x^2-0.5) (0 </\\> 1) = " ++ 
    show (zero 0.01 (\x -> x^2-0.5) (0 </\> 1))
  putStrLn $ "zero 0.001 (\\x -> x^2-0.5) (0 </\\> 1) = " ++ 
    show (zero 0.001 (\x -> x^2-0.5) (0 </\> 1))

-- find leftmost zero of f in d to within e accuracy
zero :: RI -> (RI -> RI) -> RI -> Maybe RI
zero e f d =
  zero' e f [d]
zero' :: RI -> (RI -> RI) -> [RI] -> Maybe RI
zero' _ _ [] = Nothing 
zero' e f (d:ds)
  | imageContainsZero == Just False = 
      zero' e f ds
  | imageContainsZero == Just True && reachedSufficientAccuracy = 
      Just d
  | cannotSplit =
      error $ "zero: cannot split interval " ++ show d
  | otherwise = 
      zero' e f (dl:dr:ds)
  where
  cannotSplit = (dl |==? d) == Just True || (dr |==? d) == Just True
  imageContainsZero = fd |<=? 0 
  reachedSufficientAccuracy = 
    case width fd <? e of
      Just True -> True
      _ -> False
  (dl,dr) = bisect Nothing d
  fd = f d

  
  