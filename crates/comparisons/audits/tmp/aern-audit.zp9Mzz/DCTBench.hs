module DCTBench where
import qualified Main as P
import qualified FnReps.Polynomial.UnaryChebSparse.DCTMultiplication as D
import Criterion.Main
import Control.Monad (unless)
main = do
  let degrees = [4,16,32,64,128]
      operands d = (P.coeffs 17 d,P.coeffs 29 d)
  failures <- concat <$> mapM (\d -> let (a,b)=operands d in P.checkCase ("bench " ++ show d,a,b)) degrees
  unless (sum failures == 0) $ error "independent pre-benchmark oracle failed"
  defaultMain [bgroup (show d)
    [bench "direct" $ nf (uncurry D.multiplyDirect_terms) input,
     bench "dct-quiet" $ nf (uncurry D.multiplyDCT_terms) input]
    | d <- degrees, let (a,b) = operands d, let input = (P.asInput a,P.asInput b)]
