module HTMLWizard
 ( HTML
 , URL
 , prose
 , attributedElement
 , element
 , set, Name, Value
 , page
 , format, Tag
 , h, h1, h2, h3, h4, h5, h6, h7
 , p
 , hr
 , font, Color, Face, Size
 , href
 , name
 , image
 , ul
 , ol
 , dl
 , pre
 , table, cell, row, matrix
 , form, get, post, Action
 , submit, reset, radio, checkbox, textfield, hidden, button
 , menu
 , textarea, Wrap
 , object, CLSID
 )
where

import HTML
import URL

{-
HTML utility combinators (Friday, March 07, 1997)

Erik Meijer (erik@cs.ruu.nl)
-}

-- Interface HTMLWizard --------------------------------------
page     :: String -> [HTML] -> HTML

format   :: Tag -> String -> HTML

h        :: Int -> String -> HTML
p        :: [HTML] -> HTML
font     :: Color -> [Face] -> Size -> [HTML] -> HTML

href     :: URL -> [HTML] -> HTML
name     :: String -> [HTML] -> HTML
image    :: String -> URL -> HTML

ul       :: [[HTML]] -> HTML
ol       :: [[HTML]] -> HTML
dl       :: [(String,[HTML])] -> HTML

matrix   :: [ [ [HTML] ] ] -> HTML
table    :: [HTML] -> HTML
row      :: [HTML] -> HTML
cell     :: Int -> [HTML] -> HTML

get      :: [HTML] -> HTML
post     :: [HTML] -> HTML
form     :: Action -> Method -> [HTML] -> HTML
widget   :: Widget -> (Name,Value) -> HTML
menu     :: Name -> [Value] -> HTML
textarea :: Name -> Int -> Int -> Wrap -> Value -> HTML
object   :: CLSID -> String -> [(Name,Value)] -> HTML
--------------------------------------------------------------

type Color = String
type Face  = String
type Size  = String

type Action = URL
type Method = String

type Widget = String
type Wrap   = String
type CLSID  = String

{-
page title attributes html
 = <HTML>
     <HEAD>
       <TITLE>title</TITLE>
     </HEAD>
     <BODY attributes>
       html
     </BODY>
   </HTML>
-}

page title html
 = element "HTML"
     [ element "HEAD" [element "TITLE" [prose title]]
     , element "BODY" html
     ]

{-
format tag text
 = <tag>text</tag>
-}

format tag text = element tag [prose text]

br = format "BR" ""

{-
h i heading
 = <Hi>heading</Hi>
-}

h n heading = element ("H"++show n) [prose heading]
h1 = h 1; h2 = h 2; h3 = h 3; h4 = h 4; h5 = h 5; h6 = h 6; h7 = h 7

{-
p html
 = <P>html</P>
-}

p = element "P"

{-
hr = <HR></HR>
-}

hr = element "HR" []

{-
font color [face1,...,facen] size html
 = <FONT
      COLOR = color
      FACE  = face1,...,facen
      SIZE  = size
   >
     html
   </FONT>
-}

font color face size html
 = attributedElement "FONT"
     [ ("COLOR", color)
     , ("FACE" ,(tail.init.show) face)
     , ("SIZE" , size)
     ] html

{-
href url html
 = <A HREF = url>
     html
   </A>
-}

href url html   = attributedElement "A" [("HREF",url)] html

{-
name label html
 = <A HREF = name>
     html
   </A>
-}

name label html = attributedElement "A" [("NAME",label)] html

{-
image alt url
 = <IMG
      SRC = src
      ALT = alt
   >
-}

image alt src
 = attributedElement "IMAGE" [("SRC",src), ("ALT",alt)] []

{-
ul [item1,...,itemn]
 = <UL>
     <LI>item1</LI>
     ...
     <LI>itemn</LI>
   </UL>
-}

ul is = element "OL" [ element "LI" i | i <- is ]

{-
ol [item1,...,itemn]
 = <OL>
     <LI>item1</LI>
     ...
     <LI>itemn</LI>
   </OL>
-}

ol is = element "OL" [ element "LI" i | i <- is ]

{-
dl [(label1,item1),...(labeln,itemn)]
 = <DL>
     <DT><B>label1</B></DT>
     <DD>item1</DD>
     ...
     <DT><B>labeln</B></DT>
     <DD>itemn</DD>
   </DL>
-}

dl tds
 = element "DL" (concat [ [ element "DT" [format "B" dt]
                          , element "DD" dd
                          ]
                        | (dt,dd) <- tds
                        ]
                )

pre s
 = element "PRE" [prose s]

{-
cel html
 = <TD>html</TD>
-}

cell n html
 = attributedElement "TD" [("COLSPAN",show n)] html
 
{-
table [ [td_11,...,td_1a]
      ,...
      , [td_m1,...,td_z]
      ]
 = <TABLE BORDER>
     <TR>
       <TD>td_11</TD>...<TD>td_1a</TD>
     </TR>
     ...
     <TR>
       <TD>td_m1</TD>...<TD>td_mz</TD>
     </TR>
   </TABLE>
-}

table rows
 = attributedElement "TABLE" [("BORDER","")] rows
 
matrix rows
 = table [ row [ cell 1 td | td <- r ] | r <- rows ]
       
row r
 = element "TR" r
     
{-
widget w (name,value)
 = <INPUT
      TYPE  = w
      NAME  = name
      VALUE = value
   >
   </INPUT>
-}

widget w (name,value)
 = attributedElement "INPUT" [("TYPE",w), ("NAME",name), ("VALUE",value)] []

checkbox
 = widget "CHECKBOX"

hidden 
 = widget "HIDDEN"

password
 = widget "PASSWORD"

radio
 = widget "RADIO"

reset
 = widget "RESET"
 
submit
 = widget "SUBMIT"
 
textfield
 = widget "TEXT"

button (name,src)
 = attributedElement "INPUT" 
    [ ("TYPE","IMAGE")
    , ("NAME",name)
    , ("SRC",src)
    ] []
    
group w name values
 = let buttons = [ [ [w (name,value)], [prose value] ]
                 | value <- values
                 ]
   in matrix {-name-} {-[]-} buttons
{-
menu name [choice1,...,choicen]
 = <SELECT NAME=name>
     <OPTION>choice1</OPTION>
     ...
     ,OPTION>choicen</OPTION>
   </SELECT>
-}

menu name choices
 = attributedElement "SELECT" [("NAME",name)]
     [ element "OPTION" [prose choice]
     | choice <- choices
     ]

{-
textarea name rows cols wrap text
 = <TEXTAREA
      NAME = name
      ROWS = rows
      COLS = cols
      WRAP = wrap
   >
     text
   </TEXTAREA>
-}

textarea name rows cols wrap text
 = attributedElement "TEXTAREA"
     [ ("NAME",name), ("ROWS", show rows), ("COLS", show cols), ("WRAP", wrap)]
     [prose text]

{-
form action method html
 = <FORM
      METHOD = method
      ACTION = action
   >
     html
   </FORM>
-}

form action method html
 = attributedElement "FORM"
     [ ("ACTION",action), ("METHOD",method) ]
     html

post html
 = attributedElement "FORM"
      [ ("METHOD","POST") ]
      html

get html
 = attributedElement "FORM"
     [ ("METHOD", "GET") ]
     html
 
{-
object classid id [(name1,value1),...,(namen,valuen)]
 = <OBJECT
      CLASSID = "clsid:"++classid
      ID      = id
   >
     <PARAM name1=value1></PARAM>
     ...
     <PARAM namen = valuen></PARAM>
   </OBJECT>
-}

object classid id params
 = attributedElement "OBJECT"
     [ ("CLASSID", "clsid:"++classid), ("ID", id) ]
     [ attributedElement "PARAM" [("NAME",n),("VALUE",v)] [] | (n,v) <- params ]
