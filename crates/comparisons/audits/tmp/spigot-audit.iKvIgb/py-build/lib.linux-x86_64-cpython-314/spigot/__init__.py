# User-visible side of the spigot Python package. Uses Python operator
# overloading to build a wrapper around the C++ spigot class that
# makes it look more or less like a Python numeric type.

import sys as _sys
import spigot._internal as _internal
import string as _string
import types as _types
import numbers as _numbers
import collections as _collections
try:
    from collections.abc import Iterator as _collectionIterator
except ImportError: # this moved between Python versions
    from collections import Iterator as _collectionIterator
import struct as _struct

if _sys.version_info.major >= 3:
    _is_stringish = lambda s: isinstance(s, str) or isinstance(s, bytes)
else:
    _is_stringish = lambda s: isinstance(s, basestring) or isinstance(s, bytes)

def _to_spigot(x, permit_string=False):
    ret = _internal.Spigot()
    if isinstance(x, Spigot):
        ret.clone(x._sp)
    elif isinstance(x, _numbers.Integral):
        ret.parse("{:#x}".format(int(x)), {}, None)
    elif isinstance(x, _numbers.Rational):
        ret.parse("{:#x}/{:#x}".format(x.numerator, x.denominator), {}, None)
    elif isinstance(x, _numbers.Real):
        ret.parse(float(x).hex(), {}, None)
    elif permit_string and _is_stringish(x):
        ret.parse_literal(x)
    else:
        raise TypeError("cannot convert %s to spigot" % repr(type(x)))
    return ret

class _ScopeWrapper(object):
    def __init__(self, scopes):
        if scopes is None:
            self.scopes = []
        elif type(scopes) is list:
            self.scopes = scopes
        else:
            self.scopes = [scopes]
    def __getitem__(self, name):
        for scope in self.scopes:
            try:
                value = scope(name) if callable(scope) else scope[name]
                if value is not None:
                    return _to_spigot(value)
            except KeyError:
                pass

# Expose symbolic constants like the rounding mode enumeration.
globals().update(_internal.constants())

_default_float_precision = 20

class Spigot(object):
    r'''The main data type defined by the ``spigot`` module, representing a
    real number in an exact manner.

    You can construct a ``Spigot`` object from a value `x` of any
    Python integral, rational or real-valued type, for example
    ``int``, ``fractions.Fraction`` or ``float``. You can also
    construct one from a string containing various numeric literal
    formats.

    .. testsetup::

       import spigot
       import fractions

    .. doctest::

       >>> spigot.Spigot(12345)
       <Spigot:12345>
       >>> spigot.Spigot(fractions.Fraction(3, 2))
       <Spigot:3/2>
       >>> spigot.Spigot(1.5)
       <Spigot:3/2>
       >>> spigot.Spigot("3.27")
       <Spigot:327/100>
       >>> spigot.Spigot("1.3e-2")
       <Spigot:13/1000>
       >>> spigot.Spigot("22/7")
       <Spigot:22/7>

    In each case, the Spigot object will represent the *exact* value
    of the input object, even if that in turn was an approximation of
    some previous value. For example:

    .. doctest::

       >>> spigot.Spigot(fractions.Fraction(1, 3))
       <Spigot:1/3>
       >>> spigot.Spigot(1.0 / 3.0)
       <Spigot:6004799503160661/18014398509481984>

    In this example, a ``fractions.Fraction`` representing the number
    :math:`\frac13` is converted into a ``Spigot`` object representing
    the same exact fraction. But a ``float`` representing the best
    available approximation to :math:`\frac13` comes out as a
    ``Spigot`` storing the more complicated fraction that is the
    closest available floating-point approximation to the same number.

    This floating-point inaccuracy is not limited to fractional
    numbers:

    .. doctest::

      >>> spigot.Spigot(10**50)
      <Spigot:100000000000000000000000000000000000000000000000000>
      >>> spigot.Spigot("1.0e+50")
      <Spigot:100000000000000000000000000000000000000000000000000>
      >>> spigot.Spigot(1.0e+50)
      <Spigot:100000000000000007629769841091887003294964970946560>

    Here, we see that converting a Python integer into a ``Spigot``
    returns exactly the number you expected; letting ``Spigot`` parse
    a string containing a typical floating-point literal format also
    gives the exact number; but starting from what *Python* made of
    the same floating-point literal returns only an approximation.

    The default string representation of a ``Spigot`` can show an
    exact integer or fraction, as in the examples above; if the
    ``Spigot`` object does not represent a rational (or, at least, if
    the exact-real-arithmetic system does not *know* it represents a
    rational) then the string representation will show a decimal
    representation:

    .. doctest::

      >>> spigot.pi
      <Spigot:3.1415926535897932385>
      >>> spigot.sqrt(2)
      <Spigot:1.4142135623730950488>
      >>> spigot.sqrt(2*10**100)
      <Spigot:1.4142135623730950488e+50>

    When you see a representation with a decimal point in it, that
    means the representation is approximate rather than exact. But
    it's only the *representation* that is approximate; each of the
    objects shown above is capable of returning the represented number
    to any precision, given enough computing time, memory and patience.

    '''

    def __init__(self, x):
        if isinstance(x, _internal.Spigot):
            self._sp = _internal.Spigot()
            self._sp.clone(x)
        else:
            self._sp = _to_spigot(x, permit_string=True)

    def __add__(self, s2):
        return self.eval("x+y", {"x":self, "y":s2})

    def __radd__(self, s2):
        return self.eval("y+x", {"x":self, "y":s2})

    def __sub__(self, s2):
        return self.eval("x-y", {"x":self, "y":s2})

    def __rsub__(self, s2):
        return self.eval("y-x", {"x":self, "y":s2})

    def __mul__(self, s2):
        return self.eval("x*y", {"x":self, "y":s2})

    def __rmul__(self, s2):
        return self.eval("y*x", {"x":self, "y":s2})

    def __div__(self, s2):
        return self.eval("x/y", {"x":self, "y":s2})

    def __rdiv__(self, s2):
        return self.eval("y/x", {"x":self, "y":s2})

    def __truediv__(self, s2):
        return self.eval("x/y", {"x":self, "y":s2})

    def __rtruediv__(self, s2):
        return self.eval("y/x", {"x":self, "y":s2})

    def __pow__(self, s2):
        return self.eval("x^y", {"x":self, "y":s2})

    def __rpow__(self, s2):
        return self.eval("y^x", {"x":self, "y":s2})

    def __neg__(self):
        return self.eval("-x", {"x":self})

    def __pos__(self):
        return self

    def __abs__(self):
        return self.eval("abs(x)", {"x":self})

    def __long__(self):
        sp = _internal.Spigot()
        sp.clone(self._sp)
        intpart = sp.cfracterm()
        if intpart < 0:
            next = sp.cfracterm()
            if next != None:
                intpart = intpart + 1
        return intpart

    def __int__(self):
        return int(self.__long__())

    def to_int(self, rmode=ROUND_TO_NEAREST_EVEN):
        r'''Converts the represented real number :math:`x` to an integer, and
        returns it as an ordinary Python integer type.

        If :math:`x` stores an *exact* integer value already, then of
        course ``to_int`` will return that integer.

        .. doctest::

            >>> spigot.Spigot(12345)
            <Spigot:12345>
            >>> spigot.Spigot(12345).to_int()
            12345

        Otherwise, :math:`x` will be rounded to either the next
        smaller integer :math:`\lfloor x\rfloor`, or the next larger
        integer :math:`\lceil x\rceil`. The optional parameter
        ``rmode`` controls how ``spigot`` will choose which of those
        to return. See :ref:`roundingmodes` for a description of all
        the available options.

        '''
        return self._sp.to_int(rmode)
    def floor_int(self):
        r'''Returns :math:`\lfloor x\rfloor`, as an ordinary Python integer.

        This function is a shorthand for ``x.to_int(spigot.ROUND_DOWN)``.
        
        '''

        return self.to_int(rmode=ROUND_DOWN)
    def ceil_int(self):
        r'''Returns :math:`\lceil x\rceil`, as an ordinary Python integer.

        This function is a shorthand for ``x.to_int(spigot.ROUND_UP)``.
        
        '''
        return self.to_int(rmode=ROUND_UP)

    def __float__(self):
        # Use sp.ieee to convert into a hex string in IEEE 754 double
        # encoding. Convert that to a 64-bit integer, and then use the
        # standard struct module to type-pun that integer into a double.
        sp = _internal.Spigot()
        sp.clone(self._sp)
        sp.ieee(64, 0, ROUND_TO_NEAREST_EVEN)
        hextext = "".join(iter(sp.readfmt, ""))
        return _struct.unpack(">d", _struct.pack(">Q", int(hextext, 16)))[0]

    def __cmp__(self, s2): 
        if not isinstance(s2, Spigot):
            s2 =  Spigot(s2)
        sp = _internal.Spigot()
        sp.parse("x-y", {"x":self._sp, "y":s2._sp}, None)
        return sp.sign()

    def __lt__(self, rhs):
        return self.__cmp__(rhs) < 0
    def __gt__(self, rhs):
        return self.__cmp__(rhs) > 0
    def __le__(self, rhs):
        return self.__cmp__(rhs) <= 0
    def __ge__(self, rhs):
        return self.__cmp__(rhs) >= 0
    def __eq__(self, rhs):
        return self.__cmp__(rhs) == 0
    def __ne__(self, rhs):
        return self.__cmp__(rhs) != 0

    def __nonzero__(self):
        sp = _internal.Spigot()
        sp.clone(self._sp)
        return sp.sign() != 0

    def __bool__(self):
        return self.__nonzero__()

    def sign_int(self):
        r'''Returns a Python integer giving the sign of :math:`x`.

        The returned value is :math:`+1` if :math:`x>0`; :math:`-1` if
        :math:`x<0`; or :math:`0`, if the system can determine that
        :math:`x=0` without an exactness hazard.

        (If :math:`x=0` but the system *cannot* deduce that from first
        principles, this function wlil hang.)

        '''
        return self._sp.sign()

    def known_rational_value(self, output_type=None):
        r'''Returns the numerator and denominator of the ``Spigot`` object's
        value :math:`x`, if that value is known by the system to be a
        rational.

        If the value is not a rational -- or even if it is but the
        system can't determine that from purely mechanical calculation
        -- then the returned value will be ``None``:

        .. doctest::

            >>> print(spigot.pi.known_rational_value())
            None
            >>> print(spigot.sin(spigot.asin(0.5)).known_rational_value())
            None

        However, if a ``Spigot`` value was constructed directly from
        an integer or a fraction, or was made by doing arithmetic on
        other such values, then the system will *know* that it is
        rational, and will be able to tell you its value here. This is
        also true in some cases of the transcendental functions, if
        the returned value from one of those can be known to be a
        rational:

        .. doctest::

            >>> spigot.Spigot(23).known_rational_value()
            (23, 1)
            >>> spigot.fraction(22, 46).known_rational_value()
            (11, 23)
            >>> (spigot.fraction(22, 46) + 1).known_rational_value()
            (34, 23)
            >>> (spigot.log(0.125,32)).known_rational_value()
            (-3, 5)

        When the answer is not ``None``, it is returned by default as
        a tuple consisting of the numerator and denominator (in lowest
        terms) of the fraction. However, if you provide the optional
        ``output_type`` parameter, then the numerator and denominator
        will be converted into that type, and their quotient will be
        returned. So you can retrieve the known rational value of a
        ``Spigot`` in the form of a ``fractions.Fraction`` if you like:

        .. doctest::

            >>> val = spigot.log(0.125,32)
            >>> val.known_rational_value(fractions.Fraction)
            Fraction(-3, 5)

        '''

        ratval = self._sp.rationalval()
        if ratval is None:
            return None
        if output_type is not None:
            n, d = ratval
            return output_type(n) / output_type(d)
        return ratval

    def to_cfrac(self):
        r'''Returns an iterator which yields the simple continued fraction
        coefficients of the represented real number, as Python
        integers.

        The (simple) continued fraction representation of a number is
        a sequence of positive integers, usually written in the form
        :math:`[a;b,c,d,e,f,\ldots]`, representing the value

        .. math::

            a+\frac1{b+\frac1{c+\frac1{d+\frac1{e+\frac1{f+\frac1\cdots}}}}}

        If the number is rational, then the sequence eventually
        terminates, and the above expression will be finite, so it can
        be simplified in an obvious (if tedious) way back into the
        usual :math:`a/b` form for representing a rational.

        There is a small redundancy in the representation, in
        principle, because if the final coefficient is not 1 then it
        can always be reduced by 1 and a 1 appended. For example,
        these two expressions are the same number, simply because
        :math:`3 = 2+\frac11`:

        .. math::

            \begin{align}
            [1;2,3] &= 1+\frac1{2+\frac1{3}} \\
            [1;2,2,1] &= 1+\frac1{2+\frac1{2+\frac11}} \\
            \end{align}

        ``spigot``'s resolution of this ambiguity is to always choose
        the representation with a final coefficient that is *not* 1,
        with the sole exception that if the represented real number is
        itself exactly the integer 1, then the returned sequence of
        coefficients (consisting of just the integer part) has no
        choice but to end in a 1.

        If the number is not rational, then the value represented by
        an infinite continued fraction expression of this kind is
        defined to be the limit of the sequence of rational values you
        get by truncating it to finite length at later and later
        points. For example, we define

        .. math::

            \begin{align}
            [1;1,1,1,1,\ldots] &= \lim \left( 1,
                                              1+\frac11,
                                              1+\frac1{1+\frac11},
                                              1+\frac1{1+\frac1{1+\frac11}},
                                              \ldots \right) \\
                               &= \lim \left( 1, 2, {\textstyle\frac32},
                                              {\textstyle\frac53},
                                              \ldots \right) \\
            \end{align}

        which it so happens works out to :math:`\frac12(1+\sqrt5)`,
        otherwise known as the golden ratio :math:`\phi`.

        The rational approximations obtained by this truncation are
        called the 'convergents', and are (under a certain metric) the
        *best* rational approximations to the represented number.

        '''

        sp = _internal.Spigot()
        sp.clone(self._sp)
        while True:
            term = sp.cfracterm()
            if term is None:
                return
            yield term

    def to_convergents(self, output_type=None):
        r'''Returns an iterator which yields the continued fraction
        convergents of the represented real number.

        As discussed in the documentation for
        :func:`spigot.Spigot.to_cfrac`, the convergents are rational
        numbers obtained by evaluating finite prefixes of a number's
        continued fraction, and are especially good rational
        approximations to the true value.

        By default, this function returns each convergent in the form
        of a tuple of two Python integers, giving the numerator and
        denominator of the rational approximation.

        .. doctest::

            >>> cv_it = spigot.pi.to_convergents()
            >>> next(cv_it)
            (3, 1)
            >>> next(cv_it)
            (22, 7)
            >>> next(cv_it)
            (333, 106)
            >>> next(cv_it)
            (355, 113)

        Similarly to :func:`spigot.Spigot.known_rational_value`, you
        can provide a type name as the ``output_type`` optional
        parameter, in which case the numerator and denominator will be
        converted into that type and divided to give a single returned
        value. So you can retrieve the convergents in the form of the
        ``fractions.Fraction`` type, for example:

        .. doctest::

            >>> cv_it = spigot.pi.to_convergents(fractions.Fraction)
            >>> next(cv_it)
            Fraction(3, 1)
            >>> next(cv_it)
            Fraction(22, 7)

        Or you can pass ``float``, to get a floating-point
        approximation to each convergent's value. You can even pass
        the ``spigot.Spigot`` class itself, to get the rational
        convergents back in the form of objects of the ``Spigot`` real
        type.

        .. doctest::

            >>> cv_it = spigot.pi.to_convergents(spigot.Spigot)
            >>> next(cv_it)
            <Spigot:3>
            >>> next(cv_it)
            <Spigot:22/7>

        '''

        n0, n1, d0, d1 = 0, 1, 1, 0
        for term in self.to_cfrac():
            n0, n1 = n1, term*n1+n0
            d0, d1 = d1, term*d1+d0
            if output_type is not None:
                yield output_type(n1) / output_type(d1)
            else:
                yield (n1, d1)

    def to_digits(self, base=10, positive_fraction=False):
        r'''Returns an iterator which delivers the decimal digits of the
        represented real number :math:`x`, as Python integers.

        The first value returned from the iterator is the number's
        entire integer part, so there is no limit on its value. All
        subsequent digits are fractional, and will be in the range
        :math:`[0,\mathrm{base})`.

        If :math:`x\geqslant0`, the first value (integer part) will
        simply be a Python integer with value :math:`\lfloor
        x\rfloor`. If :math:`x<0`, the absolute value of the integer
        part will be wrapped in a :class:`spigot.BASE_NEG` to indicate
        that the number is negative. (You can retrieve the integer
        stored inside that object by accessing its ``value`` field.)

        The use of ``BASE_NEG`` allows numbers like :math:`+0.123` and
        :math:`-0.123` to be distinguished. If the first return value
        was a plain integer in both cases, then it would have to be
        :math:`+0` and :math:`-0` respectively, but they're the same
        integer. As it is, the two numbers can be easily told apart:

        .. doctest::

            >>> list(spigot.Spigot("0.123").to_digits())
            [0, 1, 2, 3]
            >>> list(spigot.Spigot("-0.123").to_digits())
            [spigot.BASE_NEG(0), 1, 2, 3]

        Alternatively, if you set the optional parameter
        ``positive_fraction`` to ``True``, then the first number
        returned from the iterator will be an integer regardless of
        sign, and the subsequent fractional digits will have positive
        sense:

        .. doctest::

            >>> list(spigot.Spigot("-0.123").to_digits(
            ...      positive_fraction=True))
            [-1, 8, 7, 7]

        This is expressing :math:`-0.123` in the form :math:`(-1) +
        0.877`, sometimes written as :math:`\overline{1}.877`.

        You can set the optional parameter ``base`` to a number other
        than 10 to request the digits of :math:`x` in a different
        base. The base can be any integer greater than 1, even very
        large ones too big to fit in a machine integer:

        .. doctest::

            >>> ds = spigot.pi.to_digits(base=10**30)
            >>> "{:d}".format(next(ds))
            '3'
            >>> "{:d}".format(next(ds))
            '141592653589793238462643383279'

        The object returned from ``to_digits`` also provides a method
        called ``get_digit``, which can be called directly with an
        argument giving the base in which to extract the next digit.
        This allows you to find a representation of a number in a
        positional system in which the base varies per digit -- such
        as the 'factorial base' system, in which the first fractional
        digit is in base 2, the next in base 3, and so on increasing
        by 1 each time. For example, in this system, :math:`e` has the
        very simple expansion :math:`2.\dot{1}`:

        .. doctest::

            >>> ds = spigot.e.to_digits()
            >>> next(ds)
            2
            >>> ds.get_digit(2)
            1
            >>> ds.get_digit(3)
            1
            >>> ds.get_digit(4)
            1

        If you retrieve the first number from the iterator by calling
        the ``get_digit`` method, then the ``base`` parameter is
        ignored.

        '''
        sp = _internal.Spigot()
        sp.clone(self._sp)
        return _ToDigitsIterator(sp, base, positive_fraction)

    def base_format(self, base=10, uppercase=False, digitlimit=None,
                    rmode=ROUND_TOWARD_ZERO, minintdigits=1):
        r'''Returns an iterator which yields the decimal expansion of the
        represented real number, character by character.

        Successive calls to ``next()`` on the returned iterator will
        yield the digits of the number's integer part, then a ``'.'``
        for the decimal point, and then the digits of the number's
        fractional part. By default, an unlimited number of digits
        will be returned, until memory runs out, or until you stop
        asking for digits, or until an exactness hazard is
        encountered. Or the expansion might terminate if the number is
        one with an exact terminating representation in decimal.

        .. testsetup::

            import itertools

        .. doctest::

            >>> bf_pi = spigot.pi.base_format()
            >>> list(itertools.islice(bf_pi, 0, 10))
            ['3', '.', '1', '4', '1', '5', '9', '2', '6', '5']
            >>> bf_exact = spigot.fraction(12345,100).base_format()
            >>> list(itertools.islice(bf_exact, 0, 10))
            ['1', '2', '3', '.', '4', '5']

        You can change the number base of the returned character
        string by specifying the optional ``base`` parameter as any
        integer from 2 (binary) to 36 inclusive. For bases larger than
        10, the extra digits will be represented by lower-case letters
        of the Latin alphabet (for example, the commonly used base 16
        will have digits ``0123456789abcdef`` as usual). If you prefer
        upper-case letters, you can also set ``uppercase=True``.

        If you want the digit stream to terminate after a certain
        number of digits *regardless* of whether the number's true
        expansion goes on for longer, you can set the ``digitlimit``
        parameter. This counts digits *after* the point, not including
        the point itself or any digits before it.

        .. doctest::

            >>> bf_pi = spigot.pi.base_format(digitlimit=3)
            >>> list(bf_pi)
            ['3', '.', '1', '4', '1']

        By default, a digit stream generated with a limit in this way
        is a truncation of the full infinite digit stream. In other
        words, the value represented by the output digits has been
        rounded towards zero from the true value of the input number.
        You can specify different rounding behaviour by setting the
        ``rmode`` parameter; see :ref:`roundingmodes` for a list of
        the available options.

        Finally, the ``minintdigits`` parameter controls whether
        leading zeroes are printed for the integer part. Normally, the
        integer part will be output with as few digits as possible;
        increasing the value of ``minintdigits`` will cause leading
        zeroes to be output if the integer part would otherwise have
        fewer digits than that.

        .. doctest::

            >>> bf_pi = spigot.pi.base_format(minintdigits=3)
            >>> list(itertools.islice(bf_pi, 0, 10))
            ['0', '0', '3', '.', '1', '4', '1', '5', '9', '2']

        '''

        # Reorganise arguments into the simpler form the C side will
        # expect.
        if digitlimit is None:
            digitlimit = 0
            rmode = -1
        else:
            assert rmode is not None

        sp = _internal.Spigot()
        sp.clone(self._sp)
        sp.base(base, digitlimit, rmode, minintdigits, uppercase)

        while True:
            s = sp.readfmt()
            if s == "":
                break
            for c in s:
                yield c

    def base_format_str(self, **kws):
        r'''Returns a limited-length decimal expansion of the represented real
        number, as a single string.

        This is a shorthand for calling the method
        :py:func:`spigot.Spigot.base_format()`, and then concatenating
        all the returned characters into a single string.

        This function accepts all the same keyword arguments as
        ``base_format``, except that ``digitlimit`` *must* be set to
        something other than ``None`` (because an infinitely long
        string is not a possible return value for this function in the
        way that an infinitely long iterator makes sense for
        ``base_format``).

        .. doctest::

            >>> spigot.pi.base_format_str(digitlimit=20, base=7)
            '3.06636514320361341102'

        '''

        assert kws.get("digitlimit") is not None
        return "".join(self.base_format(**kws))

    def _to_ieee(self, bits=64, extra=0, rmode=ROUND_TO_NEAREST_EVEN):
        sp = _internal.Spigot()
        sp.clone(self._sp)
        sp.ieee(bits, extra, rmode)
        return "".join(iter(sp.readfmt, ""))

    def to_ieee_d(self, extra=0, rmode=ROUND_TO_NEAREST_EVEN):
        r'''Returns a hexadecimal string giving the binary encoding of
        the represented number, in IEEE 754 double precision.

        By default, the output string is 16 hex digits long, and
        describes a 64-bit number of which the topmost bit is the sign
        (0 for a positive number and 1 for a negative one), the next
        11 bits are the exponent, and the remaining 52 bits are the
        mantissa.

        .. doctest::

            >>> spigot.Spigot(1).to_ieee_d()
            '3ff0000000000000'
            >>> spigot.Spigot(-1).to_ieee_d()
            'bff0000000000000'
            >>> spigot.fraction(1,10).to_ieee_d()
            '3fb999999999999a'

        Non-zero real numbers so small that they underflow to zero
        will be returned as the IEEE 754 representation of :math:`+0`
        or :math:`-0` according to their sign. An exact zero is always
        returned as :math:`+0`.

        .. doctest::

            >>> spigot.Spigot("1e-1000").to_ieee_d()
            '0000000000000000'
            >>> spigot.Spigot("-1e-1000").to_ieee_d()
            '8000000000000000'
            >>> spigot.Spigot(0).to_ieee_d()
            '0000000000000000'

        Numbers too large to be represented are returned as the IEEE
        754 representation of :math:`+\infty` or :math:`-\infty`.

        .. doctest::

            >>> spigot.Spigot("1e+1000").to_ieee_d()
            '7ff0000000000000'
            >>> spigot.Spigot("-1e+1000").to_ieee_d()
            'fff0000000000000'

        This function never returns a IEEE 754 NaN representation.

        If you set ``extra`` to a value greater than zero, the initial
        64 bits will be followed by a ``'.'`` character, and then
        further hex digits which show what the mantissa field would
        contain if it had room for further data. ``extra`` is measured
        in bits, so the number of extra characters of the string will
        be about :math:`\frac14` of the value you set it to.

        .. doctest::

            >>> spigot.fraction(1,10).to_ieee_d()
            '3fb999999999999a'
            >>> spigot.fraction(1,10).to_ieee_d(32)
            '3fb9999999999999.9999999a'

        By default, the number is converted into IEEE 754
        representation using the default IEEE 754 rounding mode (round
        to nearest, breaking ties by rounding to even), represented by
        the constant ``spigot.ROUND_TO_NEAREST_EVEN``. You can change
        this by setting the ``rmode`` parameter to any of the other
        rounding modes supported by ``spigot``, as documented in
        :ref:`roundingmodes`.

        .. doctest::

            >>> spigot.fraction(1,10).to_ieee_d()
            '3fb999999999999a'
            >>> spigot.fraction(1,10).to_ieee_d(rmode=spigot.ROUND_DOWN)
            '3fb9999999999999'

        '''
        return self._to_ieee(bits=64, extra=extra, rmode=rmode)
    def to_ieee_s(self, extra=0, rmode=ROUND_TO_NEAREST_EVEN):
        r''' Similar to :func:`spigot.Spigot.to_ieee_d`, but returns IEEE 754
        single precision instead of double.

        By default, the output string is 8 hex digits long, and
        describes a 32-bit number of which the topmost bit is the sign
        (0 for a positive number and 1 for a negative one), the next 8
        bits are the exponent, and the remaining 23 bits are the
        mantissa.

        .. doctest::

            >>> spigot.Spigot(1).to_ieee_s()
            '3f800000'
            >>> spigot.Spigot(-1).to_ieee_s()
            'bf800000'
            >>> spigot.fraction(1,10).to_ieee_s()
            '3dcccccd'
            >>> spigot.fraction(1,10).to_ieee_s(32)
            '3dcccccc.cccccccd'
            >>> spigot.fraction(1,10).to_ieee_s(rmode=spigot.ROUND_DOWN)
            '3dcccccc'

        '''
        return self._to_ieee(bits=32, extra=extra, rmode=rmode)
    def to_ieee_f(self, extra=0, rmode=ROUND_TO_NEAREST_EVEN):
        '''An alias for :func:`spigot.Spigot.to_ieee_s`.'''
        return self._to_ieee(bits=32, extra=extra, rmode=rmode)
    def to_ieee_h(self, extra=0, rmode=ROUND_TO_NEAREST_EVEN):
        r''' Similar to :func:`spigot.Spigot.to_ieee_d`, but returns IEEE 754
        half-precision instead of double.

        By default, the output string is 4 hex digits long, and
        describes a 16-bit number of which the topmost bit is the sign
        (0 for a positive number and 1 for a negative one), the next 5
        bits are the exponent, and the remaining 10 bits are the
        mantissa.

        .. doctest::

            >>> spigot.Spigot(1).to_ieee_h()
            '3c00'
            >>> spigot.Spigot(-1).to_ieee_h()
            'bc00'
            >>> spigot.fraction(1,10).to_ieee_h()
            '2e66'
            >>> spigot.fraction(1,10).to_ieee_h(32)
            '2e66.66666666'
            >>> spigot.fraction(1,10).to_ieee_h(rmode=spigot.ROUND_UP)
            '2e67'

        '''
        return self._to_ieee(bits=16, extra=extra, rmode=rmode)
    def to_ieee_q(self, extra=0, rmode=ROUND_TO_NEAREST_EVEN):
        r''' Similar to :func:`spigot.Spigot.to_ieee_d`, but returns IEEE 754
        single precision instead of double.

        By default, the output string is 32 hex digits long, and
        describes a 128-bit number of which the topmost bit is the sign
        (0 for a positive number and 1 for a negative one), the next 15
        bits are the exponent, and the remaining 112 bits are the
        mantissa.

        .. doctest::

            >>> spigot.Spigot(1).to_ieee_q()
            '3fff0000000000000000000000000000'
            >>> spigot.Spigot(-1).to_ieee_q()
            'bfff0000000000000000000000000000'
            >>> spigot.fraction(1,10).to_ieee_q()
            '3ffb999999999999999999999999999a'
            >>> spigot.fraction(1,10).to_ieee_q(32)
            '3ffb9999999999999999999999999999.9999999a'
            >>> spigot.fraction(1,10).to_ieee_q(rmode=spigot.ROUND_DOWN)
            '3ffb9999999999999999999999999999'

        '''
        return self._to_ieee(bits=128, extra=extra, rmode=rmode)

    def __format__(self, fstr):
        flags = 0

        aligns = '<^>='
        align = None
        if len(fstr) >= 2 and fstr[1] in aligns:
            fill, align, fstr = fstr[0], fstr[1], fstr[2:]
        elif len(fstr) >= 1 and fstr[0] in aligns:
            fill, align, fstr = ' ', fstr[0], fstr[1:]
        else:
            fill, align = ' ', None

        if len(fstr) >= 1 and fstr[0] in '+- ':
            if fstr[0] == '+':
                flags |= _PRINTF_SIGN_PLUS
            elif fstr[0] == ' ':
                flags |= _PRINTF_SIGN_SPACE
            # and ignore '-', because that's the default anyway
            fstr = fstr[1:]

        if len(fstr) >= 1 and fstr[0] == '#':
            flags |= _PRINTF_FORCE_POINT
            fstr = fstr[1:]

        if len(fstr) >= 1 and fstr[0] == '0':
            fill = '0'
            fstr = fstr[1:]
            if align is None:
                align = '='

        if align is None:
            align = '>'

        width = 0
        if len(fstr) >= 1 and fstr[0] in _string.digits:
            old = fstr
            fstr = fstr.lstrip(_string.digits)
            # You'd like to say old[:-len(fstr)] here, but that only
            # works if len(fstr) > 0, because Python can't distinguish
            # '0' and '-0' as string indices.
            width = int(old[:len(old)-len(fstr)])

        comma = False
        if len(fstr) >= 1 and fstr[0] == ",":
            flags |= _PRINTF_COMMAS
            fstr = fstr[1:]

        prec = _default_float_precision
        if len(fstr) >= 2 and fstr[0] == "." and fstr[1] in _string.digits:
            old = fstr[1:]
            fstr = old.lstrip(_string.digits)
            prec = int(old[:len(old)-len(fstr)])

        # My own extension to the usual formatting system, to allow
        # you to choose any of spigot's supported rounding modes.
        rmode = ROUND_TO_NEAREST_EVEN
        if len(fstr) >= 1 and fstr[0] == "r":
            try:
                if len(fstr) >= 3 and fstr[:2] == "rn":
                    rmode = _rmodemap[fstr[:3]]
                    fstr = fstr[3:]
                else:
                    rmode = _rmodemap[fstr[:2]]
                    fstr = fstr[2:]
            except KeyError as e:
                raise ValueError(
                    "Unrecognised rounding specification '{}'".format(
                        e.args[0]))

        spec = 'g'
        if len(fstr) >= 1 and fstr[0] in "eEfFgG":
            spec, fstr = fstr[0], fstr[1:]

        if len(fstr) != 0:
            raise ValueError("Unable to parse format string for Spigot object")

        if align == '<':
            flags |= _PRINTF_ALIGN_LEFT
        elif align == '=':
            flags |= _PRINTF_ALIGN_MIDDLE
        elif align == '^':
            flags |= _PRINTF_ALIGN_CENTRE

        sp = _internal.Spigot()
        sp.clone(self._sp)
        sp.printf(rmode, width, prec, flags, ord(fill), ord(spec), False)

        toret = ""
        for s in iter(sp.readfmt, ""):
            toret += s
        return toret

    def __str__(self):
        return "{}".format(self)

    def __repr__(self):
        rat = self.known_rational_value()
        if rat is not None:
            n, d = rat
            if d == 1:
                number = "{:d}".format(n)
            else:
                number = "{:d}/{:d}".format(n, d)
        else:
            number = "{}".format(self)
        return "<{}:{}>".format(type(self).__name__, number)

    @classmethod
    def from_cfrac(cls, terms):
        'Class-method equivalent of :func:`spigot.from_cfrac`.'
        if isinstance(terms, _collectionIterator):
            terms = _IteratorStash(terms)
        sp = _internal.Spigot()
        sp.cfracsource(terms)
        return cls(sp)

    @classmethod
    def from_digits(cls, digits, base=10):
        'Class-method equivalent of :func:`spigot.from_digits`.'
        if isinstance(digits, _collectionIterator):
            digits = _IteratorStash(digits)
        sp = _internal.Spigot()
        sp.intervalsource(lambda n: _base_digit_to_interval(
            n==0, base, digits(n)))
        return cls(sp)

    @classmethod
    def from_file(cls, fileobject, base=10, exact=False):
        'Class-method equivalent of :func:`spigot.from_file`.'
        sp = _internal.Spigot()
        sp.parse("base{:d}{}file:dummy".format(base, "x" if exact else ""),
                 lambda *args: None, fileobject)
        return cls(sp)

    @classmethod
    def from_cfrac_file(cls, fileobject, exact=False):
        'Class-method equivalent of :func:`spigot.from_cfrac_file`.'
        sp = _internal.Spigot()
        sp.parse("cfrac{}file:dummy".format("x" if exact else ""),
                 lambda *args: None, fileobject)
        return cls(sp)

    @classmethod
    def eval(cls, expr, scopes=None):
        'Class-method equivalent of :func:`spigot.eval`.'
        sp = _internal.Spigot()
        sp.parse(expr, _ScopeWrapper(scopes), None)
        return cls(sp)

    @classmethod
    def fraction(cls, n, d=1):
        'Class-method equivalent of :func:`spigot.fraction`.'
        return cls.eval("n/d", {"n":n,"d":d})

# Expose all the function names from expr.cpp as Python functions in
# this module.
for _name, _arity in iter(_internal.functions().items()):
    if _arity == 0:
        _val = Spigot.eval(_name)
    elif _arity == 1:
        def _fndef(_name):
            def f(x):
                return Spigot.eval(_name + "(x)", {"x":x})
            f.__name__ = f.__qualname__ = _name
            return f
        _val = _fndef(_name)
        del _fndef
    elif _arity == 2:
        def _fndef(_name):
            def f(x, y):
                return Spigot.eval(_name + "(x,y)", {"x":x, "y":y})
            f.__name__ = f.__qualname__ = _name
            return f
        _val = _fndef(_name)
        del _fndef
    else:
        assert _arity < 0
        def _fndef(_name):
            def f(*args):
                scope = {}
                arglist = []
                for i, arg in enumerate(args):
                    argname = "Arg%d" % i
                    scope[argname] = arg
                    arglist.append(argname)
                expression = _name + "(" + ",".join(arglist) + ")"
                return Spigot.eval(expression, scope)
            f.__name__ = f.__qualname__ = _name
            return f
        _val = _fndef(_name)
        del _fndef
    globals()[_name] = _val
    del _val
del _name
del _arity

# And give them docstrings after the fact.
sin.__doc__ = r'''Computes the trigonometric sine function :math:`\sin x`,
with :math:`x` interpreted in radians.'''
cos.__doc__ = r'''Computes the trigonometric cosine function :math:`\cos x`,
with :math:`x` interpreted in radians.'''
tan.__doc__ = r'''Computes the trigonometric tangent function :math:`\tan x`,
with :math:`x` interpreted in radians.

In principle, it would be an error to call this function with
:math:`x` being an odd integer multiple of :math:`\frac\pi2`. However,
in practice, ``spigot`` will never be able to tell for sure that that
has happened, so rather than reporting an error, it will simply
compute for ever waiting for the input number to turn out to be on one
side or the other of the threshold value.'''
sind.__doc__ = r'''Computes the trigonometric sine function :math:`\sin x`,
with :math:`x` interpreted in degrees.'''
cosd.__doc__ = r'''Computes the trigonometric cosine function :math:`\cos x`,
with :math:`x` interpreted in degrees.'''
tand.__doc__ = r'''Computes the trigonometric tangent function :math:`\tan x`,
with :math:`x` interpreted in degrees.

It is an error to call this function with :math:`x` being an odd
integer multiple of 90.'''
sinc.__doc__ = r'''Computes the 'sinc' or 'cardinal sine' function, :math:`\frac{\sin x}{x}`. When :math:`x=0` (so that that expression evaluates to :math:`0/0`), the return value is 1.'''
sincn.__doc__ = r'''Computes the 'normalised' sinc function, equivalent to :math:`\operatorname{sinc} (\pi x)`.'''
asin.__doc__ = r'''Computes the inverse sine function :math:`\sin^{-1} x`,
returning a value in radians in the range :math:`[-\frac\pi2,+\frac\pi2]`.

It is an error to call this function with :math:`x\not\in[-1,+1]`.'''
asind.__doc__ = r'''Computes the inverse sine function :math:`\sin^{-1} x`,
returning a value in degrees in the range :math:`[-90,+90]`.

It is an error to call this function with :math:`x\not\in[-1,+1]`.'''
acos.__doc__ = r'''Computes the inverse cosine function :math:`\cos^{-1} x`,
returning a value in radians in the range :math:`[0,\pi]`.

It is an error to call this function with :math:`x\not\in[-1,+1]`.'''
acosd.__doc__ = r'''Computes the inverse cosine function :math:`\cos^{-1} x`,
returning a value in degrees in the range :math:`[0,180]`.

It is an error to call this function with :math:`x\not\in[-1,+1]`.'''
atan.__doc__ = r'''Computes the inverse tangent function :math:`\tan^{-1} x`,
returning a value in radians in the range :math:`[-\frac\pi2,+\frac\pi2]`.'''
atand.__doc__ = r'''Computes the inverse tangent function :math:`\tan^{-1} x`,
returning a value in degrees in the range :math:`[-90,+90]`.'''
sinh.__doc__ = r'''Computes the hyperbolic sine function :math:`\sinh x`.'''
cosh.__doc__ = r'''Computes the hyperbolic cosine function :math:`\cosh x`.'''
tanh.__doc__ = r'''Computes the hyperbolic tangent function :math:`\tanh x`.'''
asinh.__doc__ = r'''Computes the inverse hyperbolic sine function
:math:`\sinh^{-1} x`.'''
acosh.__doc__ = r'''Computes the inverse hyperbolic cosine function
:math:`\cosh^{-1} x`.

It is an error to call this function with :math:`x<1`.'''
atanh.__doc__ = r'''Computes the inverse hyperbolic tangent function
:math:`\tanh^{-1} x`.

It is an error to call this function with :math:`x\not\in(-1,+1)`.'''
exp.__doc__ = r'''Computes the exponential function :math:`\exp x`, also
known as :math:`e^x`.'''
exp10.__doc__ = r'''Computes :math:`10^x`.'''
exp2.__doc__ = r'''Computes :math:`2^x`.'''
expm1.__doc__ = r'''Computes :math:`\exp x - 1`.'''
log10.__doc__ = r'''Computes the base-10 logarithm :math:`\log_{10}x`.

It is an error to call this function with :math:`x\leqslant0`.'''
log2.__doc__ = r'''Computes the base-2 logarithm :math:`\log_2 x`.

It is an error to call this function with :math:`x\leqslant0`.'''
log1p.__doc__ = r'''Computes the natural logarithm :math:`\log(1+x)`.

It is an error to call this function with :math:`x\leqslant-1`.'''
sqrt.__doc__ = r'''Computes the square root :math:`\sqrt{x}`.

It is an error to call this function with :math:`x<0`.'''
cbrt.__doc__ = r'''Computes the cube root :math:`\sqrt[3]{x}`.'''
pow.__doc__ = r'''Computes the power function :math:`x^y`.

It is an error to call this function if :math:`x=0` and
:math:`y\leqslant0`, or if :math:`x<0` and :math:`y` is not a rational
which can be written with an odd denominator.'''
hypot.__doc__ = r'''Computes the square root of the sum of the squares
of its arguments, e.g. :math:`\sqrt{a^2+b^2}`, or
:math:`\sqrt{a^2+b^2+c^2}.` In the two-argument case, this is the
hypotenuse of a right-angled triangle with legs given by the
arguments.'''
gamma.__doc__ = r'''Computes the gamma function :math:`\Gamma(x)`,
defined for positive :math:`x` via the integral

.. math::

    \Gamma(x) = \int_0^\infty t^{x-1} e^{-t} dt

and for negative non-integer :math:`x` via the recurrence relation
:math:`\Gamma(x+1) = x\Gamma(x)`.

It is an error to call this function if :math:`x` is zero or a
negative integer.
'''
tgamma.__doc__ = r'''Computes the gamma function :math:`\Gamma(x)`.

This function is a synonym for :func:`spigot.gamma`, provided to match
the standard C function of the same name.'''
lgamma.__doc__ = r'''Computes the logarithm of the gamma function,
:math:`\log|\Gamma(x)|`.

It is an error to call this function if :math:`x` is zero or a
negative integer.'''
factorial.__doc__ = r'''Computes :math:`\Gamma(x+1)`, which equals
:math:`x!` when :math:`x` is a non-negative integer.

It is an error to call this function if :math:`x` is a
negative integer.'''
erf.__doc__ = r'''Computes the error function :math:`\operatorname{erf} x`,
defined via the integral

.. math::

    \operatorname{erf} x = \frac1{\sqrt\pi} \int_{-x}^x e^{-t^2} dt
'''
erfc.__doc__ = r'''Computes the complementary error function
:math:`1 - \operatorname{erf} x`.'''
Phi.__doc__ = r'''Computes the cumulative normal distribution function,
defined via the integral

.. math::

    \Phi(x) = \frac1{\sqrt{2\pi}} \int_{-\infty}^x e^{-\frac12 t^2} dt
'''
norm.__doc__ = r'''Computes the cumulative normal distribution function.
This function is a synonym for :func:`spigot.Phi`.'''
erfinv.__doc__ = r'''Computes the inverse of the error function
:func:`spigot.erf`.

It is an error to call this function with :math:`x\not\in(-1,+1)`.'''
inverf.__doc__ = r'''Computes the inverse of the error function.
This function is a synonym for :func:`spigot.erfinv`.'''
erfcinv.__doc__ = r'''Computes the inverse of the complementary error function
:func:`spigot.erfc`.

It is an error to call this function with :math:`x\not\in(0,2)`.'''
inverfc.__doc__ = r'''Computes the inverse of the complementary error function.
This function is a synonym for :func:`spigot.erfcinv`.'''
Phiinv.__doc__ = r'''Computes the inverse of the cumulative normal distribution
function :func:`spigot.Phi`.

It is an error to call this function with :math:`x\not\in(0,1)`.'''
invPhi.__doc__ = r'''Computes the inverse of the cumulative normal distribution
function. This function is a synonym for :func:`spigot.Phiinv`.'''
norminv.__doc__ = r'''Computes the inverse of the cumulative normal distribution
function. This function is a synonym for :func:`spigot.Phiinv`.'''
invnorm.__doc__ = r'''Computes the inverse of the cumulative normal distribution
function. This function is a synonym for :func:`spigot.Phiinv`.'''
probit.__doc__ = r'''Computes the inverse of the cumulative normal distribution
function. This function is a synonym for :func:`spigot.Phiinv`.'''
W.__doc__ = r'''Computes the positive branch of the Lambert W function, i.e.
a value :math:`y\geqslant-1` such that :math:`x=ye^y`.

It is an error to call this function with :math:`x<-\frac1e`.'''
Wn.__doc__ = r'''Computes the negative branch of the Lambert W function, i.e.
a value :math:`y\leqslant-1` such that :math:`x=ye^y`.

It is an error to call this function with :math:`x\not\in(-\frac1e,0)`.'''
Ei.__doc__ = r'''Computes the exponential integral :math:`\operatorname{Ei}(x)`.
This is a function whose derivative is :math:`e^x/x`. It is defined
for :math:`x<0` by choosing the constant of integration to make
:math:`\lim_{x\to-\infty}\operatorname{Ei}(x)=0` (i.e. it can be
regarded as the definite integral from :math:`-\infty` to :math:`x`).
To get past the integrand having a pole at 0, it is defined in the
positive domain by choosing a new constant of integration to arrange
that
:math:`\lim_{\epsilon\to0}\left(\operatorname{Ei}(\epsilon)-\operatorname{Ei}(-\epsilon)\right)=0`.

It is an error to call this function with :math:`x=0`.'''
E1.__doc__ = r'''Computes :math:`\operatorname{En}(1,x)=-\operatorname{Ei}(-x)`.

It is an error to call this function with :math:`x\leqslant0`.'''
Ein.__doc__ = r'''Computes the integral

.. math::

    \operatorname{Ein}(x) = \int_0^x \frac{1-e^{-t}}{t} dt

(This is the only member of the exponential-integral function family
which is defined and continuous on all of :math:`\mathbb{R}`).'''
li.__doc__ = r'''Computes the logarithmic integral :math:`\operatorname{li}(x)`.
This is a function whose derivative is :math:`1/\log x`. It is defined
on :math:`[0,1)` by choosing the constant of integration to make :math:`\operatorname{li}(0)=0` (i.e. it can be
regarded as the definite integral from :math:`0` to :math:`x`).
To get past the integrand having a pole at 1, it is defined in the
domain :math:`x>1` by choosing a new constant of integration to arrange
that
:math:`\lim_{\epsilon\to0}\left(\operatorname{li}(1+\epsilon)-\operatorname{li}(1-\epsilon)\right)=0`.

It is an error to call this function with :math:`x=1`, or with :math:`x<0`.'''
Li.__doc__ = r'''Computes the logarithmic integral :math:`\operatorname{Li}(x)`.
This is equal to :math:`\operatorname{li}(x)-\operatorname{li}(2)`,
i.e. it is :math:`\operatorname{li}` translated vertically by a
constant chosen to make :math:`\operatorname{Li}(2)=0`. Hence, on the
domain :math:`x>1` it can be regarded as

.. math::

    \operatorname{Li}(x) = \int_2^x \frac1{\log t} dt

It is an error to call this function with :math:`x=1`, or with :math:`x<0`.'''
Li2.__doc__ = r'''Computes the dilogarithm, or 'Spence's function', given by the integral

.. math::

    \operatorname{Li}_2(x) = -\int_0^x \frac{\log(1-t)}t dt

It is an error to call this function with :math:`x>1`.'''
Si.__doc__ = r'''Computes the trigonometric integral

.. math::

    \operatorname{Si}(x) = \int_0^x \frac{\sin t}{t} dt
'''
si.__doc__ = r'''Computes the trigonometric integral

.. math::

    \operatorname{si}(x) = -\int_x^\infty \frac{\sin t}{t} dt

(which only differs by a constant from :func:`spigot.Si`, with the
constant chosen to make the limiting value zero as
:math:`x\to+\infty`).'''
Ci.__doc__ = r'''Computes the trigonometric integral

.. math::

    \operatorname{Ci}(x) = -\int_x^\infty \frac{\cos t}{t} dt
'''
Cin.__doc__ = r'''Computes the trigonometric integral

.. math::

    \operatorname{Cin}(x) = -\int_0^x \frac{1-\cos t}{t} dt
'''
FresnelS.__doc__ = r'''Computes the normalised Fresnel integral

.. math::

    \operatorname{FresnelS}(x) = -\int_0^x \sin({\textstyle\frac\pi2}t^2) dt
'''
FresnelC.__doc__ = r'''Computes the normalised Fresnel integral

.. math::

    \operatorname{FresnelC}(x) = -\int_0^x \cos({\textstyle\frac\pi2}t^2) dt
'''
UFresnelS.__doc__ = r'''Computes the unnormalised Fresnel integral

.. math::

    \operatorname{UFresnelS}(x) = -\int_0^x \sin(t^2) dt
'''
UFresnelC.__doc__ = r'''Computes the unnormalised Fresnel integral

.. math::

    \operatorname{UFresnelC}(x) = -\int_0^x \cos(t^2) dt
'''
abs.__doc__ = r'''Computes the absolute value :math:`|x|`.'''
sign.__doc__ = r'''Computes the sign function :math:`\operatorname{sgn} x`, defined as

.. math::

    \operatorname{sgn} x = \begin{cases}
    +1 & x>0 \\
    0 & x=0 \\
    -1 & x<0 \\
    \end{cases}

Unlike :func:`spigot.Spigot.sign_int`, this function delivers its
result still in the form of a value of type ``Spigot``, not a Python
integer.'''
ceil.__doc__ = r'''Computes the ceiling function :math:`\lceil x\rceil`.

Unlike :func:`spigot.Spigot.ceil_int`, this function delivers its
result still in the form of a value of type ``Spigot``, not a Python
integer.'''
floor.__doc__ = r'''Computes the floor function :math:`\lfloor x\rfloor`.

Unlike :func:`spigot.Spigot.floor_int`, this function delivers its
result still in the form of a value of type ``Spigot``, not a Python
integer.'''
frac.__doc__ = r'''Computes the fractional part of :math:`x`, i.e.
:math:`x-\lfloor x\rfloor`.

Note that the fraction has positive sense even for :math:`x<0`.'''
fmod.__doc__ = r'''Computes the remainder of :math:`x` when divided by :math:`y`, in
the same sense as the ``fmod`` function in the C language. The
returned value differs from :math:`x` by an integer multiple of
:math:`y`; it has magnitude strictly less than :math:`|y|`; and if it
is not zero, then it has the same sign as :math:`x`.

This function is equivalent to :func:`spigot.remainder_rz`.'''
algebraic.__doc__ = r'''Computes an algebraic number, i.e. a root of a
polynomial with integer coefficients.

The first two parameters :math:`\mathrm{lo}` and :math:`\mathrm{hi}`
must be rational, and define an interval. The remaining parameters
:math:`a_0,a_1,\ldots,a_n` must also be rational, and are interpreted
as the coefficients of a polynomial
:math:`P(x)=a_0+a_1x+\cdots+a_nx^n`. The returned value is the unique
root of :math:`P` in the interval :math:`(\mathrm{lo},\mathrm{hi})`.

It is an error to call this function if :math:`P` does not have
exactly one root in the interval, or if either of :math:`\mathrm{lo}`
or :math:`\mathrm{hi}` or any coefficient of :math:`P` is not
rational. Moreover, all the arguments must be *obviously* rational:
values computed in a complicated way which *turn out* to be rational
will not do.

For example, the following expression defines the 'Tribonacci
constant', the limiting ratio of the 'Tribonacci sequence', similar to
the Fibonacci sequence but with each term being the sum of the *three*
previous terms. Just as the limiting ratio of the Fibonacci sequence
is a root of :math:`x^2=x+1`, the limiting ratio of the Tribonacci
sequence is a root of :math:`x^3=x^2+x+1`, or equivalently,
:math:`1+x+x^2-x^3=0`. The root we want is in the interval
:math:`(1,2)`, so we can define it by writing

.. doctest::

    >>> spigot.algebraic(1,2,1,1,1,-1)
    <Spigot:1.8392867552141611326>

And here's a root (the smallest positive one) of the quintic equation
:math:`x^5-6x+3`, which can't be written down as any expression in
radicals:

    >>> spigot.algebraic(0,1,3,-6,0,0,0,1)
    <Spigot:0.50550123040552466685>

'''

# For some functions, we need to replace the automated definition
# above so as to assign particular names to the arguments. Otherwise
# the docstrings won't make sense!
def atan2(y,x):
    r'''Computes the dyadic inverse tangent function.

    The returned value is an angle :math:`\theta`, in radians, in the
    range :math:`(-\pi,+\pi]`, such that the vector
    :math:`\begin{pmatrix}\cos\theta \\ \sin\theta \\ \end{pmatrix}`
    is a positive real multiple of the vector :math:`\begin{pmatrix}x
    \\ y \\ \end{pmatrix}`.

    It is an error to call this function with :math:`x=y=0`.
    '''
    return Spigot.eval("atan2(y,x)", {"x":x, "y":y})
def atan2d(y,x):
    r'''Computes the dyadic inverse tangent function.

    The returned value is an angle :math:`\theta`, in degrees, in the
    range :math:`(-180,+180]`, such that the vector
    :math:`\begin{pmatrix}\cos\theta \\ \sin\theta \\ \end{pmatrix}`
    is a positive real multiple of the vector :math:`\begin{pmatrix}x
    \\ y \\ \end{pmatrix}`.

    It is an error to call this function with :math:`x=y=0`.
    '''
    return Spigot.eval("atan2d(y,x)", {"x":x, "y":y})
def log(x, b=None):
    r'''Computes the logarithm of :math:`x` to the base :math:`b`, i.e.
    :math:`\log_b x`.

    If ``b`` is not specified, it defaults to :math:`e`, i.e. the
    function computes the 'natural' logarithm which is the inverse of
    the exponential function.

    If ``b`` is specified, then the answer is the same in principle as
    :math:`\frac{\log x}{\log b}`. However, if the true answer is a
    rational, then calling this function once with two arguments will
    avoid an exactness hazard in many cases where taking the quotient
    of two separate calls to ``log`` with one argument would hang.

    It is an error to call this function with :math:`x\leqslant0`, or
    with :math:`b\leqslant0`, or with :math:`b=1`.'''
    if b is None:
        return Spigot.eval("log(x)", {"x":x})
    else:
        return Spigot.eval("log(x,b)", {"x":x,"b":b})
def En(n,x):
    r'''Computes the :math:`n`\ -times-iterated exponential integral of
    :math:`x`.

    This function is defined inductively as follows:

    .. math::

        \begin{align}
        \operatorname{En}(0,x) &= \frac{e^{-x}}{x} \\
        \operatorname{En}(n,x) &= \int_x^\infty \operatorname{En}(n-1,t) dt
        \end{align}

    It is an error to call this function with :math:`x<0`, or with
    :math:`x=0` and :math:`n\in\{0,1\}`, or with :math:`n` not a
    non-negative integer.

    '''
    return Spigot.eval("En(n,x)", {"x":x,"n":n})
def zeta(s):
    r'''Computes the Riemann zeta function :math:`\zeta(s)`,
    defined for :math:`s>1` by the sum

    .. math::

        \zeta(s) = \sum_{n=1}^\infty \frac{1}{n^s}

    extended to :math:`s>0` by rearranging that sum into the
    equivalent (but convergent on a larger domain)

    .. math::

        \zeta(s) = \frac1{1-2^{1-s}} \sum_{n=1}^\infty (-1)^{n-1} \frac{1}{n^s}

    and defined for :math:`s<0` via the reflection formula

    .. math::

        \frac{\zeta(1-s)}{\zeta(s)} =
        \frac{2 \Gamma(s) \cos(\frac12\pi z)}{(2\pi)^s}

    It is an error to call this function with :math:`s` being an
    integer less than 2.'''
    return Spigot.eval("zeta(s)", {"s":s})
def agm(a,b):
    r'''Computes the arithmetic-geometric mean function.

    This is the limiting value obtained by repeatedly replacing the
    pair of input numbers :math:`(a,b)` with their arithmetic and
    geometric mean, i.e. iterating the transformation

    .. math::

        (a, b) \mapsto \left(\sqrt{ab}, \frac{a+b}{2}\right)

    It is an error to call this function with :math:`a<0` or :math:`b<0`.
    '''
    return Spigot.eval("agm(a,b)", {"a":a, "b":b})
def Hg(nfactors,dfactors,x):
    r'''Computes the generalised hypergeometric function.

    This is a general form of power-series function typically written
    in ordinary mathematical notation as

    .. math::

        _aF_b(n_1,\ldots,n_a; d_1,\ldots,d_b; x)

    and it is defined by the power series :math:`\sum_{i=0}^\infty A_i
    x^i`, in which the coefficient :math:`A_i` of the :math:`x^i` term
    is given by

    .. math::

        A_i = \begin{cases}
        \frac{ n_1\ldots n_a }{ d_1\ldots d_b } & i=0 \\
        \frac{ (n_1+i)\ldots (n_a+i) }{ (d_1+i)\ldots (d_b+i) }
        \frac{A_i}i & i>0 \\
        \end{cases}

    In the Python syntax, the arguments ``nfactors`` and ``dfactors``
    are lists of length :math:`a` and :math:`b`, containing the values
    :math:`(n_j)` and :math:`(d_j)`, respectively.

    It is an error to use this function with :math:`a>b+1`, with any
    denominator factor being zero or a negative integer, or with the
    primary input value :math:`x` being outside the function's domain
    of convergence (which varies with the parameters).

    '''
    nnames = ["n{:d}".format(i) for i in range(len(nfactors))]
    dnames = ["d{:d}".format(i) for i in range(len(dfactors))]
    valdict = {}
    for n, f in zip(nnames, nfactors):
        valdict[n] = f
    for n, f in zip(dnames, dfactors):
        valdict[n] = f
    valdict["x"] = x
    expr = "Hg({}; {}; x)".format(",".join(nnames), ",".join(dnames))
    return Spigot.eval(expr, valdict)
def BesselJ(alpha,x):
    r'''Computes a Bessel function of the first order.

    These functions are a particular family of solutions to the Bessel
    differential equation

    .. math::

        x^2 \frac{d^2y}{dx^2} + x \frac{dy}{dx} + (x^2-\alpha^2) y = 0

    and can also be defined by the integral

    .. math::

        J_\alpha(x) = \frac1\pi \int_0^\pi \cos(\alpha t - x\sin t) dt

    It is an error to call this function with ``alpha`` not obviously
    an integer.
    '''
    return Spigot.eval("BesselJ(a,x)", {"a":alpha, "x":x})

def BesselI(alpha,x):
    r'''Computes a modified Bessel function of the first order.

    The modified Bessel function :math:`I_\alpha` is obtained by
    extending the Bessel function :math:`J_\alpha` to the complex
    number field, and then giving it an imaginary rather than real
    input value. Specifically,

    .. math::

        I_\alpha(x) = i^{-\alpha}J_\alpha(ix)

    (where the factor of :math:`i^{-\alpha}` is needed to ensure that
    the output value is real and has a sensible sign).

    It is an error to call this function with ``alpha`` not obviously
    an integer.

    '''
    return Spigot.eval("BesselI(a,x)", {"a":alpha, "x":x})

class _IteratorStash(object):
    def __init__(self, iterator):
        self.iterator = iterator
        self.stash = []
    def __call__(self, index):
        if self.iterator is not None:
            try:
                while len(self.stash) <= index:
                    self.stash.append(next(self.iterator))
            except StopIteration:
                self.iterator = None
        return self.stash[index] if index < len(self.stash) else None

class BASE_NEG(object):
    r'''Class that wraps an integer so that :func:`spigot.from_digits` will
    recognise it as indicating the integer part of a negative real
    number.

    The constructor parameter ``value`` should be a non-negative
    integer, and will be the integer part of the absolute value of the
    resulting real number. Once a ``BASE_NEG`` object ``bn`` has been
    constructed, this integer can be extracted as ``bn.value``.

    This class is also used by :func:`spigot.Spigot.to_digits` when it
    converts a negative-valued ``Spigot`` object *to* a digit
    sequence: the first 'digit' returned for :math:`x<0` will be
    returned as ``spigot.BASE_NEG(``\ :math:`\lfloor-x\rfloor`\ ``)``.

    '''
    def __init__(self, value):
        self.value = value
    def __repr__(self):
        return "spigot.BASE_NEG({:d})".format(self.value)
    def __eq__(self, rhs):
        return isinstance(rhs, BASE_NEG) and self.value == rhs.value

class BASE_DIGIT(object):
    r'''Class that wraps an integer so that :func:`spigot.from_digits` will
    recognise it as indicating that a fractional digit of a real
    number should be interpreted in a specific number base.

    The constructor parameter ``base`` should be an integer which is
    at least 2, and ``digit`` should be an integer which is at least 0
    and strictly less than ``bases``. Once a ``BASE_DIGIT`` object
    ``bd`` has been constructed, these integers can be extracted as
    ``bd.base`` and ``bd.digit`` respectively.

    '''
    def __init__(self, base, digit):
        self.base = base
        self.digit = digit
        assert self.base >= 2
        assert 0 <= self.digit < self.base
    def __repr__(self):
        return "spigot.BASE_DIGIT({:d},{:d})".format(self.base, self.digit)
    def __eq__(self, rhs):
        return (isinstance(rhs, BASE_DIGIT) and
                self.base == rhs.base and
                self.digit == rhs.digit)

def _base_digit_to_interval(first, base, digit):
    if digit is None:
        return (0, 0, 1)
    elif first:
        if isinstance(digit, BASE_NEG):
            return (-digit.value, -(digit.value+1))
        else:
            return (digit, digit+1)
    else:
        if isinstance(digit, BASE_DIGIT):
            return (digit.digit, digit.digit+1, digit.base)
        else:
            assert 0 <= digit < base
            return (digit, digit+1, base)

class _ToDigitsIterator(object):
    def __init__(self, sp, base, positive_fraction):
        self._sp = sp
        self.base = base
        self.positive_fraction = positive_fraction
        self.first_digit = True
        self.terminated = False
    def get_digit(self, base=None):
        if self.terminated:
            return None

        if self.first_digit:
            digit, constant = self._sp.static_floor()
            if digit >= 0 or self.positive_fraction:
                self._sp.static_premultiply((1, -digit, 0, 1))
            else:
                self._sp.static_premultiply((-1, digit+1, 0, 1))
                digit = BASE_NEG(-1 - digit)
            self.first_digit = False
        else:
            if base is None:
                base = self.base
            self._sp.static_premultiply((base, 0, 0, 1))
            digit, constant = self._sp.static_floor()
            self._sp.static_premultiply((1, -digit, 0, 1))

        if constant:
            self.terminated = True
        return digit
    def __iter__(self):
        return self
    def __next__(self):
        digit = self.get_digit()
        if digit is None:
            raise StopIteration
        return digit
    def next(self):
        return self.__next__()
    def __repr__(self):
        return "<spigot.Spigot.to_digits() object at {:#x}>".format(id(self))

# Export some of Spigot's methods as top-level functions in this
# module, so that users can say (for example) spigot.from_cfrac(...)
# instead of the more cumbersome spigot.Spigot.from_cfrac(), and
# conversely, so that x.floor_int() has an analogue
# spigot.floor_int(x) to match spigot.floor(x).

def fraction(n, d=1):
    r'''Makes a ``Spigot`` object out of an integer numerator and
    denominator.

    This is a convenient shorthand for the same thing you could do
    by converting the numerator and denominator to ``Spigot`` and
    then dividing them.

    .. doctest::

        >>> spigot.fraction(13, 8)
        <Spigot:13/8>

    '''
    return Spigot.fraction(n, d)

def eval(expr, scopes=None):
    r'''Evaluates an arithmetic expression in the full syntax supported by
    the command-line ``spigot`` tool.

    The expression language supports the usual arithmetic
    operators, multiplication by juxtaposition, a wide range of
    mathematical functions (also available as functions in this
    Python module), and a functional-style ``let`` syntax for
    defining variables and subfunctions. Here are a few examples:

    .. doctest::

        >>> spigot.eval("1+2*3")
        <Spigot:7>
        >>> spigot.eval("pi^5/6!")
        <Spigot:0.42502733997955757398>
        >>> spigot.eval("let p=pi+1, f(x,y)=x^2 + 2 x y + y^2"
        ...             " in f(p, p/2)")
        <Spigot:38.593776843605126465>

    For full information on the expression syntax language, see
    the `expressions chapter
    <https://www.chiark.greenend.org.uk/~sgtatham/spigot/spigot.html#expr>`_
    of the manual for command-line ``spigot``.

    Unlike command-line ``spigot``, this Python interface to the
    expression language lets you pass in one or more 'scopes'
    containing variable definitions, so that you can be given an
    expression string containing an undefined variable name (say,
    ``x``), and evaluate it multiple times in scopes where that
    variable is defined to different values.

    A variable scope can be a dictionary or a callable object
    (including a function, a method, or a class instance with a
    ``__call__`` method defined). If it's a dictionary, it should
    map variable names to values (which can have type ``Spigot``
    or any numeric type convertible to ``Spigot``); if it's a
    callable, then when called with a variable name, it should
    return a definition of that variable or ``None`` if the name
    is not defined.

    The optional parameter ``scopes`` can be one of these scopes,
    or a list of them. If it's a list, the scopes first in the
    list take priority.

        >>> spigot.eval("x*y+1", {"x":3, "y":spigot.pi})
        <Spigot:10.424777960769379715>
        >>> def sillyscope(name):
        ...     if name.startswith("squareof"):
        ...         return int(name[8:])**2
        >>> spigot.eval("squareof3 + squareof4", sillyscope)
        <Spigot:25>
        >>> override = {"squareof3":48000}
        >>> spigot.eval("squareof3 + squareof4",
        ...             [override, sillyscope])
        <Spigot:48016>
        >>> spigot.eval("squareof3 + squareof4",
        ...             [sillyscope, override])
        <Spigot:25>

    '''
    return Spigot.eval(expr, scopes)

def from_cfrac(terms):
    r'''Makes a Spigot object out of a sequence of continued fraction
    coefficients.

    The argument ``terms`` can be an iterator that delivers the
    continued fraction coefficients of a number in sequence, or a
    function that returns the :math:`n`\ th coefficient given
    :math:`n` as an argument. (In the latter case, the first
    coefficient, i.e. the integer part of the number, has index
    zero.)

    For example, the well-known mathematical constant :math:`e`
    has a continued fraction consisting of the integer part 2,
    followed by a regular sequence of 3-term segments each of the
    form :math:`1,2n,1` for :math:`n=1,2,3,\ldots`:

    .. math::

        e = [2;1,2,1,1,4,1,1,6,1,1,8,1,\ldots]

    If we wanted to generate :math:`e` from this continued
    fraction, we could write it as a generator like this:

    .. doctest::

        >>> def e_generator():
        ...     yield 2
        ...     for n in itertools.count(1):
        ...         yield 1
        ...         yield 2*n
        ...         yield 1
        >>> spigot.from_cfrac(e_generator())
        <Spigot:2.7182818284590452354>

    or as a function taking an index argument like this:

    .. doctest::

        >>> def e_index_function(i):
        ...     if i == 0:
        ...         return 2
        ...     elif i % 3 == 2:
        ...         return 2 * (i+1) // 3
        ...     else:
        ...         return 1
        >>> spigot.from_cfrac(e_index_function)
        <Spigot:2.7182818284590452354>

    The second approach, using a function, is somewhat more
    memory-efficient. This is because any ``Spigot`` object has to
    be prepared to compute its real number more than once, so the
    iterator-based approach must indefinitely store all the values
    it has previously received from its iterator, so it can return
    them from the beginning again later. The index-function
    approach does not have this problem, because it expects to be
    able to call the function later restarting the index at 0.

    (Of course, this particular number can be computed even more
    easily by simply referring to the constant ``spigot.e``!)

    '''
    return Spigot.from_cfrac(terms)

def from_digits(digits, base=10):
    r'''Makes a Spigot object out of a sequence of decimal digits, or
    digits in some other base.

    Like :func:`spigot.from_cfrac`, the argument ``digits`` can be
    an iterator that delivers the digits of a number in sequence,
    or a function that returns the :math:`n`\ th digit given
    :math:`n` as an argument. (Also like ``from_cfrac``, in the
    latter case, the index counts from zero.)

    The 'digits' delivered by either of these methods are
    integers, in the usual case. The first 'digit' is the integer
    part of the number, and can be as large as necessary. After
    that, all the digits must be at least 0, and strictly less
    than ``base``.

    For example, here's a simple case where we provide a
    terminating sequence of digits:

    .. doctest::

        >>> spigot.from_digits(iter([1,2,3,4]))
        <Spigot:1.234>

    (Note that this is an exception to the usual case where a
    rational-valued ``Spigot`` object is displayed as a fraction,
    because in this case, the ``Spigot`` object didn't know *to
    begin with* that it had a rational value -- when it was handed
    its iterator, it didn't know whether it would go on for ever.)

    If you want to express a negative number, there are two ways
    to do so. You can deliver a negative integer as the first
    'digit', in which case the fraction defined by the remaining
    digits will be *added* to it.

    .. doctest::

        >>> spigot.from_digits(iter([-1,2,3,4]))
        <Spigot:-0.766>

    (In rare contexts in mathematics this notation is sometimes
    used, with an overbar to indicate the integer part only being
    negated. For example, this might be written as
    :math:`\overline{1}.234`.)

    Alternatively, you can replace the first digit (integer part)
    with a special object of type :class:`spigot.BASE_NEG`
    wrapping the integer you want. This will cause the whole
    number to be negated, *including* the fraction part. (It also
    allows you to distinguish an integer part of zero with and
    without negation, i.e. telling ``0.xxx`` and ``-0.xxx`` apart
    -- which would have been difficult if the negativity had been
    expressed by negating the ordinary integer part!)

    .. doctest::

        >>> spigot.from_digits(iter([spigot.BASE_NEG(1),2,3,4]))
        <Spigot:-1.234>

    If you want all the digits to be interpreted in a base other
    than 10, you can arrange that by simply setting the optional
    ``base`` parameter.

    .. doctest::

        >>> spigot.from_digits(iter([1,2,3,4]), base=16) # 0x1.234
        <Spigot:1.1376953125>

    But another thing you might want to do is to override the base
    *differently* for different digits, for example, to input a
    number expressed in a factorial base. For this purpose,
    there's another special kind of object called
    :class:`spigot.BASE_DIGIT`, which wraps a digit in an
    indication that it should be interpreted in a a specified
    base.

    To be clear on what exactly it *means* for different digits of
    a number to be in different bases: after processing a finite
    (and non-zero) prefix of a number expressed in this way, the
    system has an *interval* :math:`[x,y]` that the number might
    still be in. After the integer part, this interval will
    typically be of the form :math:`[n,n+1]`, for some integer
    :math:`n`. Thereafter, receiving a digit :math:`d` interpreted
    in base :math:`b` means that the interval :math:`[x,y]` is
    divided into :math:`b` equally sized subintervals, of which
    the new interval is the :math:`d`\ th. For example, if you
    know the number is currently in the range :math:`[0.31,0.32]`,
    and the next digit is '7', then the interval narrows to
    :math:`[0.317,0.318]`.

    Here's an infinite example. Champernowne's constant
    :math:`C_{10}` is formed by writing the consecutive integers,
    each with no leading zero, after the decimal point:

    .. math::

        C_{10} = 0.\,1\,2\,3\,4\,5\,6\,7\,8\,9\,10\,11\,12\,13\,\ldots\,98\,99\,100\,101\,\ldots

    One way to generate this with a digit iterator would be to
    break each integer up into individual digits. But a simpler
    way is to yield each of the integers as a single 'digit', and
    change the base as you go. So the integers up to 9 are each
    interpreted in base 10; the next ninety integers, 10 to 99,
    are delivered in base 100, the next nine hundred are in base
    1000, and so on.

    To create this constant using an iterator, you could do this:

    .. doctest::

        >>> def C10_iterator():
        ...     yield 0 # integer part
        ...     base = 10
        ...     for i in itertools.count(1):
        ...         if i == base:
        ...             base *= 10
        ...         yield spigot.BASE_DIGIT(base, i)
        >>> spigot.from_digits(C10_iterator())
        <Spigot:0.12345678910111213142>

    Alternatively, you can define the same sequence of digits --
    each still with its attached base -- in random-access style,
    by providing a function that returns one by its index:

    .. doctest::

        >>> def C10_index_function(n):
        ...     if n == 0:
        ...         return 0 # integer part
        ...     powers_of_10 = (10**i for i in itertools.count(1))
        ...     base = next(b for b in powers_of_10 if b > n)
        ...     return spigot.BASE_DIGIT(base, n)
        >>> spigot.from_digits(C10_index_function)
        <Spigot:0.12345678910111213142>

    '''
    return Spigot.from_digits(digits, base)

def from_file(fileobject, base=10, exact=False):
    r'''Makes a ``Spigot`` object by reading a number from a file, or from
    any other Python object that behaves like an open file.

    The file should contain a number expressed in ordinary
    positional notation, i.e. zero or more digits of integer part,
    then optionally a ``'.'`` followed by zero or more digits of
    fractional part.

    The digits in the file are interpreted in base 10 by default,
    or in any other base from 2 to 36 inclusive if the optional
    parameter ``base`` is set.

    The contents of the file need not terminate at all. If it does
    terminate, ``spigot`` may never notice, if it's never asked to
    do anything with the number that requires reading far enough
    in the file to find the end.

    If ``spigot`` does encounter the end of the file, its usual
    response will be to throw :exc:`spigot.EndOfFile`. This is
    appropriate if the file contains an approximation to some
    irrational number: it means ``spigot`` can compute with the
    number as long as it's only asked for answers that can be
    known from the limited information it has, and throws an
    exception if it is ever unable to deliver an answer on the
    information known.

    If the contents of the file are intended to *exactly*
    represent some terminating decimal expansion, you can signal
    this by setting the ``exact`` parameter to ``True``.

    In the following example, we use :class:`io.StringIO` to set
    up a file-like object containing a string that is a close
    approximation to :math:`1.2\overline{789}=\frac{4259}{3330}`.
    As long as we're computing with it at fairly low precision,
    this works fine:

    .. testsetup::

        import io

    .. doctest::

        >>> x = spigot.from_file(io.StringIO(u"1.2" + u"789"*10))
        >>> x
        <Spigot:1.278978978978978979>
        >>> spigot.sin(x)
        <Spigot:0.95772261872598355053>

    But if we try to do something that will require us to know the
    difference between this number and the real
    :math:`1.2\overline{789}`, then we see an exception:

    .. doctest::

        >>> try:
        ...     (x - spigot.fraction(4259,3330)).sign_int()
        ... except spigot.EndOfFile as e:
        ...     type(e)
        <class 'spigot.EndOfFile'>

    However, if we had set the ``exact`` flag, then our number
    would have been taken to be the *exact* number represented by
    our terminating 31-digit decimal expansion, i.e. slightly less
    than :math:`1.2\overline{789}`. In that situation, there would
    be no exception in computing that difference or its sign:

    .. doctest::

        >>> y = spigot.from_file(io.StringIO(u"1.2" + u"789"*10),
        ...                      exact=True)
        >>> (y - spigot.fraction(4259,3330)).sign_int()
        -1

    '''
    return Spigot.from_file(fileobject, base, exact)

def from_cfrac_file(fileobject, exact=False):
    r'''Makes a ``Spigot`` object by reading a sequence of continued
    fraction coefficients from a file, or from any other Python
    object that behaves like an open file.

    This function works very like :func:`spigot.from_file`, except
    for the interpretation of the contents of the file. The format
    expected is quite loose: punctuation and spaces are more or
    less ignored, and contiguous sequences of digits are taken to
    be the continued fraction coefficients in decimal. So it
    doesn't matter whether you separate the coefficients by
    spaces, or commas, or put a semicolon after the first one in
    line with common mathematical style, or use semicolons for
    *all* of them, or whatever.

    Similarly to ``from_file``, the :exc:`spigot.EndOfFile`
    exception will be raised if ``spigot`` reaches the end of the
    file but still needs more data to complete its current task.
    Setting the ``exact`` parameter to ``True`` will instead cause
    ``spigot`` to treat the continued fraction as terminating,
    i.e. to treat the number as a rational.

    For example, this :class:`io.StringIO` contains a string that
    will generate a close approximation to :math:`\sqrt{2}` (whose
    true continued fraction goes on for ever and is
    :math:`[1;2,2,2,2,\ldots]`). Simply generating the first few
    digits of this number works fine:

        >>> x = spigot.from_cfrac_file(io.StringIO(u"1" + u",2" * 100))
        >>> x
        <Spigot:1.4142135623730950488>

    but if we try to square it and take the integer part, then
    ``spigot`` will actually need to know whether the answer is
    more or less than 2, and will hit the end of the file:

        >>> try:
        ...     (x**2).floor_int()
        ... except spigot.EndOfFile as e:
        ...     type(e)
        <class 'spigot.EndOfFile'>

    '''
    return Spigot.from_cfrac_file(fileobject, exact)

def to_int(x, rmode=ROUND_TO_NEAREST_EVEN):
    'Function equivalent of :func:`spigot.Spigot.to_int`.'
    return x.to_int(rmode)
def floor_int(x):
    'Function equivalent of :func:`spigot.Spigot.floor_int`.'
    return x.floor_int()
def ceil_int(x):
    'Function equivalent of :func:`spigot.Spigot.ceil_int`.'
    return x.ceil_int()
def sign_int(x):
    'Function equivalent of :func:`spigot.Spigot.sign_int`.'
    return x.sign_int()

_rmodemap = {}
_roundfnmap = {"round":{}, "fracpart":{}, "remainder":{}}
for _abbr, _rmodename in _rmodeabbrs.items():
    _rmode = globals()[_rmodename]
    _rmodemap[_abbr] = _rmode

    # Shorter aliases for the rounding-mode constants.
    globals()[_abbr.upper()] = _rmode

    # The formulaic round/fracpart/remainder functions.
    globals()["round_" + _abbr].__doc__ = \
r'''Rounds the input number to an integer value, using the rounding mode
``spigot.{rmode}``. The returned value is not a Python intege, but an
object of type Spigot with an integer value.
'''.format(rmode=_rmodename)
    globals()["fracpart_" + _abbr].__doc__ = \
r'''Returns the 'fractional part' of the number, defined as the difference
between the input number itself and its integer part as determined by
:func:`spigot.round_{abbr}`.
'''.format(abbr=_abbr)
    globals()["remainder_" + _abbr].__doc__ = \
r'''Returns the remainder :math:`x - qy`, where :math:`q` is the result
of rounding the true quotient :math:`x/y` to an integer using
:func:`spigot.round_{abbr}`.
'''.format(abbr=_abbr)
    for _fnname in ["round","fracpart","remainder"]:
        _roundfnmap[_fnname][_rmode] = globals()[_fnname+"_"+_abbr]

def round(x, rmode=ROUND_TO_NEAREST_EVEN):
    '''Rounds the input number to an integer value, using the rounding
mode ``rmode``. The returned value is not a Python intege, but an
object of type Spigot with an integer value.
    '''
    return _roundfnmap["round"][rmode](x)
def fracpart(x, rmode=ROUND_TO_NEAREST_EVEN):
    '''Returns the 'fractional part' of the number, defined as the
difference between the input number itself and its integer part as
determined by :func:`spigot.round(x,rmode)`.
    '''
    return _roundfnmap["fracpart"][rmode](x)
def remainder(x,y, rmode=ROUND_TO_NEAREST_EVEN):
    '''Returns the remainder :math:`x - qy`, where :math:`q` is the result
of rounding the true quotient :math:`x/y` to an integer using
:func:`spigot.round()` with the specified rounding mode`.
    '''
    return _roundfnmap["remainder"][rmode](x,y)

# And one last import from _internal. This one is the exception that
# is thrown when spigot runs out of precision reading a number from a
# file.
EndOfFile = _internal.EndOfFile
