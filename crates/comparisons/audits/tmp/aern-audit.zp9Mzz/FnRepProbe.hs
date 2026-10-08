module FnRepProbe where
import AuditFnRepComparison
import FunctionAbstraction
import Numeric.AERN.MPFRBasis.Interval

data Linear = Linear
instance RF Linear where
  build _ _ = Linear
  evalR _ input = input
  evalMI _ x = x - fromRationalWithPrec (getPrec x) (1/2)
  integrate _ = error "unused probe method"
  supportsIntegration _ = False

main = do
  let result = zeroUsingTrisection Linear 0 1 20
  print ("linear zero at 1/2",result,"width",width result)
  print ("contains half",(result |<=? fromRationalWithPrec 100 (1/2)))
