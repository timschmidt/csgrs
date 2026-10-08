module HTML where

-- Interface HTML --------------------------------------------
prose             :: String -> HTML
attributedElement :: Tag -> [(Name,Value)] -> [HTML] -> HTML
element           :: Tag -> [HTML] -> HTML
set               :: [(Name,Value)] -> (HTML -> HTML)
--------------------------------------------------------------

type Tag   = String
type Name  = String
type Value = String
type Attributes = [(Name,Value)]

data HTML
 = Element Tag Attributes [HTML]
 | Text String
 deriving (Show,Read)

{-
element tage html
 = <tag>html</tag>
-}

element tag html
 = attributedElement tag [] html

{-
attributedElement tag [(name1,value1),...,(namen,valuen)] html
 = <tag
      name1 = value1
      ...
      namen = valuen
   >
     html
   </tag>
-}

attributedElement tag attributes html
 = Element tag attributes html

{-
prose text
 = text
-}

prose text = Text text

{-
-}

set attributes' (Text text )
 = Text text
set attributes' (Element tag attributes body)
 = Element tag ([ (n,v) | (n,v) <- attributes, not (elem n ns) ] ++ attributes') body
   where
    ns = map fst attributes'


