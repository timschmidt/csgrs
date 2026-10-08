module Main (main) where

import ICalc
import Interact
import IDatabase

-- Dave's stuff to compile it...

calculator = writeStr "Integration Calculator version 1.0:" 
	     (evalexpr emptyDB 10)


evalexpr :: Database -> Int -> String -> String
evalexpr db digits | digits > 0 = readLine "\n\n==> " (\s ->
			    	  let (ndb, ans, ndigits) = icalc db s digits
		                  in  writeStr ans (evalexpr ndb ndigits))
		   | otherwise  = writeStr " ...\n" end


main :: IO ()
main = interact calculator
