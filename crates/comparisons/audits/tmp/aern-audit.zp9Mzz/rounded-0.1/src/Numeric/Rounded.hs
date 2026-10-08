-----------------------------------------------------------------------------
-- |
-- Module      :  Numeric.Rounded
-- Copyright   :  (C) 2012-2014 Edward Kmett, Daniel Peebles
--                (C) 2013-2018 Claude Heiland-Allen
-- License     :  LGPL
-- Maintainer  :  Claude Heiland-Allen <claude@mathr.co.uk>
-- Stability   :  experimental
-- Portability :  non-portable
--
----------------------------------------------------------------------------
module Numeric.Rounded
    (
    -- * Floating point numbers with a specified rounding mode and precision
      Rounded()
    , fromInt
    , fromDouble
    , fromLongDouble
    , toDouble
    , toLongDouble
    , toInteger'
    , precRound
    -- * Precision
    , Precision(precision)
    , Bytes
    , reifyPrecision
    -- * Rounding
    , Rounding(rounding)
    , RoundingMode(..)
    , reifyRounding
    -- * Useful Constants
    , kLog2
    , kEuler
    , kCatalan
    -- * Combinators that are oblivious to precision
    , (.+.)
    , (.-.)
    , (.*.)
    , abs'
    , negate'
    , decodeFloat'
    , succUlp
    , predUlp
    -- * Mixed-precision operations
    , (!+!)
    , (!-!)
    , (!*!)
    , (!/!)
    , abs_
    , negate_
    , compare_
    , (!==!)
    , (!/=!)
    , (!>=!)
    , (!<=!)
    , (!>!)
    , (!<!)
    , min_
    , max_
    , sqrt_
    , exp_
    , expm1_
    , log_
    , log1p_
    , sin_
    , cos_
    , tan_
    , asin_
    , acos_
    , atan_
    , atan2_
    , sinh_
    , cosh_
    , tanh_
    , asinh_
    , acosh_
    , atanh_
    , truncate_
    , round_
    , ceiling_
    , floor_
    -- * Foreign Function Interface
    , withInRounded
    , withInOutRounded
    , withInOutRounded_
    , withOutRounded
    , withOutRounded_
    , unsafeWithOutRounded
    , unsafeWithOutRounded_
    , peekRounded
    ) where

import Numeric.Rounded.Internal
import Numeric.Rounded.Precision
import Numeric.Rounded.Rounding
