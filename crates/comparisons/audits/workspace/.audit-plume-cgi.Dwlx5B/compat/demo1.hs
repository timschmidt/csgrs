module Main (main) where


import Interact
import SBinDec
import Demos


demonstrate = writeStr "Logistic Map demo" (do_demo)


do_demo :: String -> String
do_demo = readLine "\n\nn: -> " (\s -> 
	  readLine "\n\nd: -> " (\t -> 
	  readLine "\n\nx: -> " (\u ->
	      let {n = read s;
		   d = read t;
	           x = decSbf u}
		  in  writeStr (tabulate x d n 0) end)))


	


main :: IO ()
main = interact demonstrate
