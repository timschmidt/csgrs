-----------------------------------------------------------------------------
-- |
-- Module      :  Numeric.MPFR.Raw
-- Copyright   :  (C) 2012-2014 Edward Kmett, Daniel Peebles
--                (C) 2013-2017 Claude Heiland-Allen
-- License     :  LGPL
-- Maintainer  :  Claude Heiland-Allen <claude@mathr.co.uk>
-- Stability   :  experimental
-- Portability :  non-portable
--
-- This module contains FFI imports.  In a future release they may switch
-- at runtime between safe and unsafe calls depending on cost estimates, but
-- for now thie module simply re-exports 'Numeric.MPFR.Raw.Safe'.
--
-- Note: beware issues with the GHC threaded runtime if you need the MPFR
-- status flags (which use OS thread local storage aka TLS).  Even if you use
-- 'Control.Concurrent.forkOS' to create a bound thread whose FFI will all be
-- done by the same OS thread, sparks created with 'Control.Parallel.par'
-- may run on a different OS thread and lead to unpredictable behaviour.

module Numeric.MPFR.Raw
  ( module Numeric.MPFR.Raw.Safe
  ) where

import Numeric.MPFR.Raw.Safe
