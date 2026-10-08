{-# LANGUAGE TypeFamilies #-}
{-# LANGUAGE FlexibleContexts #-}
{-# LANGUAGE MultiParamTypeClasses #-}
{-|
    Module      :  Numeric.AERN.RmToRn
    Description :  real function operations (except pointwise ones)  
    Copyright   :  (c) Michal Konecny
    License     :  BSD3

    Maintainer  :  mikkonecny@gmail.com
    Stability   :  experimental
    Portability :  portable
    
    Common real function operations (except pointwise ones).
-}

module Numeric.AERN.RmToRn
(
    module Numeric.AERN.RmToRn.Domain,
    module Numeric.AERN.RmToRn.New,
    module Numeric.AERN.RmToRn.Evaluation,
    module Numeric.AERN.RmToRn.Integration,
    module Numeric.AERN.RmToRn.Differentiation,
    RoundedRealFn(..)
)
where

import Numeric.AERN.RmToRn.Domain
import Numeric.AERN.RmToRn.New
import Numeric.AERN.RmToRn.Evaluation
import Numeric.AERN.RmToRn.Integration
import Numeric.AERN.RmToRn.Differentiation

--import Numeric.AERN.RealArithmetic.ExactOps
--import Numeric.AERN.RealArithmetic.Measures
--
--import qualified Numeric.AERN.RealArithmetic.NumericOrderRounding as ArithUpDn
import qualified Numeric.AERN.RealArithmetic.RefinementOrderRounding as ArithInOut
--
--import qualified Numeric.AERN.NumericOrder as NumOrd
import qualified Numeric.AERN.RefinementOrder as RefOrd
--
--import Numeric.AERN.Basics.Effort

{-|
   An aggregate class collecting together all functionality
   normally expected from approximations to real number functions
   such as polynomial enclosures.
   
   It also provides a single aggregate effort indicator type
   from which effort indicators for all the rounded operations can
   be extracted.
-}
class 
    (ArithInOut.RoundedReal f,
     ArithInOut.RoundedReal (Domain f),
     RefOrd.IntervalLike (Domain f),
     HasDomainBox f, GeneratableVariables (Var f),
     HasConstFns f, HasProjections f,
     CanEvaluate f
    ) 
    => 
    RoundedRealFn f
    where
    type RoundedRealFnEffortIndicator f
    roundedRealFnDefaultEffort :: f -> RoundedRealFnEffortIndicator f
    fnEffortReal :: f -> (RoundedRealFnEffortIndicator f) -> (ArithInOut.RoundedRealEffortIndicator f)
    fnEffortDomReal :: f -> (RoundedRealFnEffortIndicator f) -> (ArithInOut.RoundedRealEffortIndicator (Domain f))
    fnEffortDomGetEndpoints :: f -> (RoundedRealFnEffortIndicator f) -> (RefOrd.GetEndpointsEffortIndicator (Domain f))
    fnEffortDomFromEndpoints :: f -> (RoundedRealFnEffortIndicator f) -> (RefOrd.FromEndpointsEffortIndicator (Domain f))
    fnEffortEval :: f -> (RoundedRealFnEffortIndicator f) -> EvaluationEffortIndicator f
    
    