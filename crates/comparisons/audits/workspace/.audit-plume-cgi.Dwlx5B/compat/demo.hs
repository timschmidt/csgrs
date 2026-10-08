module Main (main) where

import CGI
import WebSite
import Test

main
 = website demo
 
demo
 = WebSite "Welcome" welcomeScript []
      
welcomeScript query
 = do{ intro <- helloScript query
     ; return $ welcomePage intro
     }

welcomePage intro
 = [ h1 "Welcome to Haskell/CGI home page"
   , p intro ]

