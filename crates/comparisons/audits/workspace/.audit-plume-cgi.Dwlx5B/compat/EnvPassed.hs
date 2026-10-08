module EnvPassed 
 ( envScript
 ) 
where

import CGI

envScript = \query ->
  do{ env <- cgiEnv
   ; return $ envPage (env, query)
   }
   
envPage (env, query)
 = [ h1 "Environment"
    , showAssoc env
    , h1 "Query"
    , showAssoc query
    ]
    
showAssoc :: [(String,String)] -> HTML
showAssoc nvs
 = dl [ (name, [prose value]) 
      | (name,value) <- nvs 
      ]