-- Compatibility names only: preserve the donor's outward interval operations.
module AuditDCTCompat (DI, piOut, (/|), (|*)) where
import Numeric.AERN.RealArithmetic.Interval.Double (DI, sampleDI)
import Numeric.AERN.RealArithmetic.Basis.Double ()
import qualified Numeric.AERN.RealArithmetic.RefinementOrderRounding as R

piOut :: DI
piOut = R.piOut sampleDI
infixl 7 /|, |*
(/|) :: DI -> Int -> DI
(/|) = R.mixedDivOut
(|*) :: Int -> DI -> DI
(|*) = flip R.mixedMultOut
