module Main 
 ( main
 ) 
where

import CGI

main = wrapper $ \request -> let query = request2query request in 
 do { case check query of
        Just Download  -> zipFiles query
        Just Back      -> return $ location agentHome
        otherwise      -> return $ html registrationPage
    } 

data Command
 = Download
 | Back
 deriving (Show, Read)

check :: Query -> Maybe Command
check query
 = tryRead (find query "Command") 

registrationPage 
 = page "HaskellScript Registration Page"
    [ h1 "Welcome to the HaskellScript Download Page"
    , p [ prose "If you want to stay informed about the "
        , prose "latest version of HaskellScript, "
        , prose "you can fill in your name and e-mail address."
        ]
    , get [ p [ prose "Name:"  , textfield ("Name","")      ]
           , p [ prose "e-mail:", textfield ("Email","")     ]
           , p [ submit ("Command", show Download)
               , reset ("", "Reset")
               , submit ("Command", show Back)
               ]
           ] 
    , p [ prose "When you use Netscape Communicator "
        , prose "you might get stuck after clicking Download."
        , prose "If this happens, "
        , prose "download the "
        , prose ("<a href="++(show automationHome)++">")
        , prose "Agent files</a> directly. "
        , prose ":-( :-( :-("
        ]
    ]

zipFiles query
 = do{ appendFile logFile newUser 
     ; return $ location automationHome
     }
   where
      newUser 
       = "Name: " ++ (find query "Name") ++ "\n" ++
         "Email: "++ (find query "Email") ++"\n"

logFile
 = "/ogi/guest/erik/public_html/Personal/LogFiles/users.log" 

agentHome
 = "http://www.cse.ogi.edu/~erik/Personal/agentscr.html"

automationHome
 = "http://www.cse.ogi.edu/~erik/HugsAutomation.zip"