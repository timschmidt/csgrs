module Test where

import Convert
import DyStream
import SBinDec

xmul a b = dysMul (sbsToDys (decSb a))  (sbsToDys (decSb b))

