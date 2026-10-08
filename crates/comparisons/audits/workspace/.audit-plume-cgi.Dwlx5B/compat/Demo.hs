module Main (main) where

import CGI
import WebSite
import EnvPassed
import Hello
import Counter

main
 = website demo
 
demo
 = WebSite "Welcome" welcomeScript
     [ WebSite "Environment" envScript []
     , WebSite "Counter" counterScript []
     ]
      
welcomeScript query
 = do{ intro <- helloScript query
     ; return $ welcomePage intro
     }

welcomePage intro
 = [ h1 "Welcome to Haskell/CGI home page"
   , p intro 
   , p [ prose $ "From this page you can navigate to "++show n++" demo pages:" 
       , ol demos
       ]
   , p [ prose $ "The main specification of this page looks like:"
       , element "PRE" 
           [ prose $ "\ndemo = WebSite \"Welcome\" welcomeScript\n"
                   ++"   [ WebSite \"Environment\" envScript []\n"
                   ++"   , WebSite \"Counter\" counterScript []\n"
                   ++"   ]"  
           ]
        ]
   ] where
        n = length demos
        demos
         = [ [post [ submit ("","Environment"), hidden ("page","(1,0)")]]
           , [post [ submit ("","Counter")    , hidden ("page","(1,1)")]]
           ]
