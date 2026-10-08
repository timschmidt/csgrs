module UrlEncoded
 ( Query
 , emptyQuery
 , readQuery
 , find
 )
where

import Parsing
import Char
import Pretty

type Query = [(String,String)]

emptyQuery = []

find :: Query -> String -> String
find query name
 = head [ value | (name',value) <- query ++ [(name,"")], name == name']

readQuery s
 = case (papply query s) of
      []            -> emptyQuery
      ((query,_):_) -> query

{-
An URL encoded value consist of a sequence of
zero or more name "=" value pairs separated by "&"

Query ::= [Name "=" Value {"&" Name "=" Value}]

Names and values are URL-encoded,
according to the following table

   character | encoding
   ----------|---------
    ' '      | '+'
    '<'      | "%XX"
     c       | "%"hexval(ord c)

-}

query
 = do{ name <- urlEncoded
     ; string "="
     ; value <- urlEncoded
     ; return (name,value)
     } `sepby` (string "&")

urlEncoded
 = many ( alphanum ++ extra ++ safe ++ space ++ hexencoded )

hexencoded
 = do{ char '%'
     ; d <- hexadecimal
     ; return $ chr (hex2int d)
     }

space = do{ char '+' ; return ' '}

extra = sat (`elem` "!*'(),")

safe = sat (`elem` "$-_.")

hexadecimal :: Parser HexString
hexadecimal
 = do{ d1 <- hexdigit; d2 <- hexdigit; return [d1,d2] }

type HexString = String

hex2int :: HexString -> Int
hex2int ds
 = foldl (\n d -> n*16+d) 0 (map (toInt.toUpper) ds)
   where
      toInt d | isDigit d = ord d - ord '0'
      toInt d | isHex d   = (ord d - ord 'A') + 10