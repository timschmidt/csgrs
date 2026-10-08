module Response
 ( Response(Content, Location, Status)
 , putResponse
 , content
 , location
 , html
 , textplain
 , urlencoded
 , status
 , Reason
 )
where

import Mime
import URL

type Code = Int
type Reason = String

data Response
 = Content Mime
 | Location URL
 | Status Code Reason

content :: Mime -> Response
content mime = Content mime

location :: URL -> Response
location url = Location url

status :: Code -> Reason -> Response
status code reason = Status code reason

textplain :: String -> Response
textplain = content.TextPlain

html :: HTML -> Response
html = content.TextHtml

urlencoded :: Query -> Response
urlencoded = content.UrlEncoded

putResponse response
 = case response of
     { Content mime
        -> do{ putStr ("Content-type: " ++ mimeType mime ++ "\n\n")
             ; putMime mime
             }
     ; Location url
        -> do{ putStr ("Location: " ++ url ++ "\n\n") }
     ; Status code reason
        -> do{ putStr ("Status: " ++ (show code) ++ " ")
             ; putStr (reason ++ "\n\n")
             }
     }