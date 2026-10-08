rounded
=======

[![Build Status](https://secure.travis-ci.org/ekmett/rounded.png?branch=master)](http://travis-ci.org/ekmett/rounded)

This package provides properly rounded floating point numbers of arbitrary precision.
It does so by wrapping the GNU MPFR library.

Phantom types carry the information about the precision and rounding mode, letting you treat properly rounded floating
point numbers as instances of `Num` or `Floating`, like any other numeric type in Haskell.

Unlike other attempts to port MPFR to Haskell, this library does not require you to cripple `Integer` performance
or link your code in an unnatural way.

Usage
-----

```haskell
{-# LANGUAGE DataKinds #-}
{-# LANGUAGE ScopedTypeVariables #-}
import Numeric.Rounded
import Data.Proxy
```

To use a 53 bit significand (the same size as used by a Double), and round down intermediate results:

```haskell
>>> pi :: Rounded TowardZero Double
3.141592653589793
```

We can also round away from zero, or use other rounding modes.

```haskell
>>> pi :: Rounded AwayFromZero Double
3.1415926535897936
```

We can specify the significand size directly using type literals in GHC:

```haskell
>>> kCatalan :: Rounded TowardZero 128
0.915965594177219015054603514932384110773
```

You can also specify a dynamic significand size at runtime:

```haskell
>>> reifyPrecision 512 (\(_ :: Proxy p) -> show (logBase 10 2 :: Rounded TowardNearest p))
"0.3010299956639811952137388947244930267681898814621085413104274611271081892744245094869272521181861720406844771914309953790947678811335235059996923337046956"
```

or a dynamic rounding mode:

```haskell
ghci> reifyRounding TowardZero (\(_ :: Proxy r) -> show (logBase 10 2 :: Rounded r 512))
"0.30102999566398119521373889472449302676818988146210854131042746112710818927442450948692725211818617204068447719143099537909476788113352350599969233370469556"
```

Contact Information
-------------------

Please, feel free to contact me with questions, concerns, or bug fixes.

I can be reached as ekmett via github or as edwardk on the #haskell IRC channel on irc.freenode.net.

-Edward Kmett
