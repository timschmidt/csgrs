module Counter 
 ( counterScript
 ) 
where

import CGI

counterScript :: Query -> IO [HTML]
counterScript = \query -> let this = find query "page" in
 do{ case check query of
        Down n -> return $ counterPage this (n-1)
        Up   n -> return $ counterPage this (n+1)
        Same n -> return $ counterPage this (n)
   } 

data Cmd 
 = Up Int
 | Down Int
 | Same Int
 deriving (Read,Show)

check :: Query -> Cmd
check query
 = case tryRead (find query "Cmd") of
     Nothing  -> Same 0
     Just cmd -> cmd

counterPage this n
 = [ table [ row [ cell 2 [prose $ show n] ]
           , row [ cell 1 upButton, cell 1 downButton]
           ]
   ]
   where 
      upButton   = [ post [ submit (""    , "Up")
                          , hidden ("Cmd" , show $ Up n)
                          , hidden ("page", this)
                          ]  
                   ]
      downButton = [ post [ submit (""    , "Down")
                          , hidden ("Cmd" , show $ Down n)
                          , hidden ("page", this)
                          ] 
                   ]