#!/usr/bin/env python3

# Test suite for the spigot Python bindings.

import sys
import os
import math
import numbers
import unittest
import itertools
import io
import codecs
import tempfile
import subprocess
import re
from fractions import Fraction

import spigot as real_spigot_module

class spigot_logging_wrapper(object):
    """Wrapper class to log the identifiers we've tested.

    The aim is to make sure we test every function and constant
    defined by the spigot module.

    """
    def __init__(self):
        # We don't want to test internal functions, whose names start
        # with _. But everything other than that should be tested in
        # this file.
        self.namesAvailable = {name for name in dir(real_spigot_module)
                               if not name.startswith("_")}
        self.namesUsed = set()
    def __getattr__(self, name):
        if not name.startswith("_"):
            self.namesUsed.add(name)
        return getattr(real_spigot_module, name)

spigot = spigot_logging_wrapper()

class spigot_tests(unittest.TestCase):
    def __init__(self, *args, **kws):
        super(spigot_tests, self).__init__(*args, **kws)
        if not hasattr(self, "assertRaisesRegex"):
            # Work around Python 2 and 3 calling this method by
            # different names.
            self.assertRaisesRegex = self.assertRaisesRegexp

        # Reusable data tables
        self.roundingModes = [
            spigot.ROUND_DOWN, spigot.ROUND_UP,
            spigot.ROUND_TOWARD_ZERO, spigot.ROUND_AWAY_FROM_ZERO,
            spigot.ROUND_TO_NEAREST_EVEN, spigot.ROUND_TO_NEAREST_ODD,
            spigot.ROUND_TO_NEAREST_DOWN, spigot.ROUND_TO_NEAREST_UP,
            spigot.ROUND_TO_NEAREST_TOWARD_ZERO,
            spigot.ROUND_TO_NEAREST_AWAY_FROM_ZERO
        ]
        self.roundingModeAbbrs = {
            "rz": spigot.ROUND_TOWARD_ZERO,
            "ri": spigot.ROUND_AWAY_FROM_ZERO,
            "ra": spigot.ROUND_AWAY_FROM_ZERO,
            "rn": spigot.ROUND_TO_NEAREST_EVEN,
            "rne": spigot.ROUND_TO_NEAREST_EVEN,
            "rno": spigot.ROUND_TO_NEAREST_ODD,
            "rnz": spigot.ROUND_TO_NEAREST_TOWARD_ZERO,
            "rni": spigot.ROUND_TO_NEAREST_AWAY_FROM_ZERO,
            "rna": spigot.ROUND_TO_NEAREST_AWAY_FROM_ZERO,
            "ru": spigot.ROUND_UP,
            "rp": spigot.ROUND_UP,
            "rd": spigot.ROUND_DOWN,
            "rm": spigot.ROUND_DOWN,
            "rnu": spigot.ROUND_TO_NEAREST_UP,
            "rnp": spigot.ROUND_TO_NEAREST_UP,
            "rnd": spigot.ROUND_TO_NEAREST_DOWN,
            "rnm": spigot.ROUND_TO_NEAREST_DOWN,
        }

    def testzzzFinallyCheckLogs(self):
        """Ensure this test has used every public definition in the module.

        It has 'zzz' in the name to ensure it sorts after all the
        other test methods in this class.

        """
        self.assertEqual(spigot.namesUsed, spigot.namesAvailable)

    def assertRational(self, spig, expectedValue):
        "Check that a spigot has a specific rational value."
        self.assertEqual(spig.known_rational_value(Fraction), expectedValue)

    def assertDecimal(self, spig, expectedExpansion):
        "Check that a spigot's decimal expansion starts with a given string."
        gen = spig.base_format()
        actualExpansion = ""
        while len(actualExpansion) < len(expectedExpansion):
            actualExpansion += next(gen)
        self.assertEqual(actualExpansion, expectedExpansion)

    def assertEqualNDigits(self, x, y, n):
        "Check two spigots' decimal expansions start with the same n digits."
        xd = "".join(x.base_format(digitlimit=n))
        yd = "".join(y.base_format(digitlimit=n))
        self.assertEqual(xd, yd)

    def testConversions(self):
        """Test conversion of Python numeric types to/from Spigot."""

        for z in [0, 1, -1, 3**72, -7**41]:
            self.assertRational(spigot.Spigot(z), Fraction(z))
            self.assertEqual(int(spigot.Spigot(z)), z)

        for q in [Fraction(0), Fraction(1,2), Fraction(-3**71,7**39)]:
            self.assertRational(spigot.Spigot(q), q)

        for f in [0.0, 0.5, 1e+300, 1e-310, -1e+300, -1e-310]:
            self.assertRational(spigot.Spigot(f), Fraction(f))
            self.assertEqual(float(spigot.Spigot(f)), f)

    def testStringLiteral(self):
        """Test conversion from a string by the Spigot constructor.

        This should accept any numeric literal syntax understood by
        the main Spigot parser, and any quotient of two of them with
        an optional leading minus sign (so that you can specify
        rationals directly), but not any other combination of
        arithmetic operators, and also no variables or functions (and
        especially not opening of files!).

        """

        self.assertEqual(spigot.Spigot("1"), spigot.Spigot(1))
        self.assertEqual(spigot.Spigot("1e100"), spigot.Spigot(10**100))
        self.assertEqual(spigot.Spigot("22/7"), spigot.fraction(22, 7))
        self.assertEqual(spigot.Spigot("0x1.8p+1"), spigot.Spigot(3))
        self.assertEqual(spigot.Spigot("-0x1.8p+1"), spigot.Spigot(-3))
        self.assertEqual(spigot.Spigot("-base4:1.23"), spigot.fraction(-27,16))

        with self.assertRaises(ValueError):
            spigot.Spigot("x")
        with self.assertRaises(ValueError):
            spigot.Spigot("1+2")
        with self.assertRaises(ValueError):
            spigot.Spigot("1-2")
        with self.assertRaises(ValueError):
            spigot.Spigot("pi")
        with self.assertRaises(ValueError):
            spigot.Spigot("sin(1)")
        with self.assertRaises(ValueError):
            spigot.Spigot("base10file:foo.txt")
        with self.assertRaises(ValueError):
            spigot.Spigot("base7fd:1")
        with self.assertRaises(ValueError):
            spigot.Spigot("cfracstdin")

    def testScope(self):

        """Test handling of scope parameter in spigot.eval.

        We check that the scope parameter can be a dictionary or a
        function, or a list containing dictionaries and/or functions,
        or None. We also check that each entry in the scope can be
        either an actual Spigot object or one of the Python types that
        we can convert into one, and that if a list of multiple scopes
        are provided and more than one defines the same variable, then
        the first one in the list wins.

        """

        # These two spigot values are going to be used in the
        # following tests, but also, we set one up using spigot.eval
        # with no scope parameter at all, and the other using
        # parameter None, which will also test that _those_ work OK.
        dspig = spigot.eval("base13:123.456")
        fspig = spigot.eval("base11:123.456", None)

        scopeDict = { "dspig":dspig,
                      "dint":123,
                      "dfrac":Fraction(22,7),
                      "dfloat":1234.5,
                      "disputed":100 }

        def scopeFunc(name):
            if name == "fspig":
                return fspig
            elif name == "fint":
                return 456
            elif name == "ffrac":
                return Fraction(355,113)
            elif name == "ffloat":
                return 9876.5
            elif name == "disputed":
                return 200

        dspig_fraction = Fraction(435753,13**3)
        fspig_fraction = Fraction(194871,11**3)

        for scope in [scopeDict, [scopeDict],
                      [scopeDict,scopeFunc], [scopeFunc,scopeDict]]:
            self.assertRational(spigot.eval("dspig", scope), dspig_fraction)
            self.assertRational(spigot.eval("dint", scope), 123)
            self.assertRational(spigot.eval("dfrac", scope), Fraction(22, 7))
            self.assertRational(spigot.eval("dfloat", scope), Fraction(1234.5))

        for scope in [scopeFunc, [scopeFunc],
                      [scopeDict,scopeFunc], [scopeFunc,scopeDict]]:
            self.assertRational(spigot.eval("fspig", scope), fspig_fraction)
            self.assertRational(spigot.eval("fint", scope), 456)
            self.assertRational(spigot.eval("ffrac", scope), Fraction(355,113))
            self.assertRational(spigot.eval("ffloat", scope), Fraction(9876.5))

        for scope in [scopeDict, [scopeDict], [scopeDict,scopeFunc]]:
            self.assertRational(spigot.eval("disputed", scope), 100)

        for scope in [scopeFunc, [scopeFunc], [scopeFunc,scopeDict]]:
            self.assertRational(spigot.eval("disputed", scope), 200)

    def testOperators(self):
        """Test all the Python overloaded operators.

        Includes testing that each one manages to coerce its other
        argument to type Spigot, no matter which side the other
        argument appears on.

        """
        self.assertDecimal(spigot.pi + spigot.e, "5.85987")
        self.assertDecimal(spigot.pi + 1, "4.14159")
        self.assertDecimal(10 + spigot.pi, "13.14159")
        self.assertDecimal(spigot.pi - spigot.e, "0.42331")
        self.assertDecimal(spigot.pi - 1, "2.14159")
        self.assertDecimal(10 - spigot.pi, "6.85840")
        self.assertDecimal(spigot.pi * spigot.e, "8.53973")
        self.assertDecimal(spigot.pi * 2, "6.28318")
        self.assertDecimal(2 * spigot.pi, "6.28318")
        self.assertDecimal(spigot.pi / spigot.e, "1.15572")
        self.assertDecimal(spigot.pi / 2, "1.57079")
        self.assertDecimal(2 / spigot.pi, "0.63661")
        self.assertDecimal(spigot.pi ** spigot.e, "22.45915")
        self.assertDecimal(spigot.pi ** 2, "9.86960")
        self.assertDecimal(2 ** spigot.pi, "8.82497")
        self.assertDecimal(-spigot.pi, "-3.14159")
        self.assertDecimal(+spigot.pi, "3.14159")
        self.assertDecimal(abs(spigot.cos(2)), "0.41614")

    def testFunctions(self):
        """Test all the functions exported from C++ spigot.

        For each one, we check that it exists, has the right arity,
        and looks like the right function. (There's no need to do
        seriously demanding numerical tests; the main spigot test
        suite will do that. Here we're just checking that they've all
        turned up in Python and haven't got the wrong API.)

        This includes constants like pi and e, which for these
        purposes are regarded as nullary functions.

        """

        # Constants
        self.assertDecimal(spigot.apery, "1.20205")
        self.assertDecimal(spigot.e, "2.71828")
        self.assertDecimal(spigot.eulergamma, "0.57721")
        self.assertDecimal(spigot.phi, "1.61803")
        self.assertDecimal(spigot.pi, "3.14159")
        self.assertDecimal(spigot.tau, "6.28318")
        self.assertDecimal(spigot.catalan, "0.91596")
        self.assertDecimal(spigot.gauss, "0.83462")

        # Unary functions
        self.assertDecimal(spigot.abs(-1.25), "1.25")
        self.assertDecimal(spigot.acos(Fraction(1,3)), "1.23095")
        self.assertDecimal(spigot.acosd(Fraction(1,3)), "70.52877")
        self.assertDecimal(spigot.acosh(Fraction(4,3)), "0.79536")
        self.assertDecimal(spigot.asin(Fraction(1,3)), "0.33983")
        self.assertDecimal(spigot.asind(Fraction(1,3)), "19.47122")
        self.assertDecimal(spigot.asinh(Fraction(4,3)), "1.09861")
        self.assertDecimal(spigot.atan(Fraction(1,3)), "0.32175")
        self.assertDecimal(spigot.atand(Fraction(1,3)), "18.43494")
        self.assertDecimal(spigot.atanh(Fraction(1,3)), "0.34657")
        self.assertDecimal(spigot.cbrt(-3), "-1.44224")
        self.assertDecimal(spigot.ceil(spigot.pi), "4")
        self.assertDecimal(spigot.cos(1), "0.54030")
        self.assertDecimal(spigot.cosd(1), "0.99984")
        self.assertDecimal(spigot.cosh(1), "1.54308")
        self.assertDecimal(spigot.Ci(1), "0.33740")
        self.assertDecimal(spigot.Cin(1), "0.23981")
        self.assertDecimal(spigot.E1(1), "0.21938")
        self.assertDecimal(spigot.erf(1), "0.84270")
        self.assertDecimal(spigot.erfc(1), "0.15729")
        self.assertDecimal(spigot.erfcinv(0.25), "0.81341")
        self.assertDecimal(spigot.inverfc(0.25), "0.81341")
        self.assertDecimal(spigot.erfinv(0.25), "0.22531")
        self.assertDecimal(spigot.inverf(0.25), "0.22531")
        self.assertDecimal(spigot.exp(2), "7.38905")
        self.assertDecimal(spigot.exp10(2.25), "177.82794")
        self.assertDecimal(spigot.exp2(2.25), "4.75682")
        self.assertDecimal(spigot.expm1(0.0625), "0.06449")
        self.assertDecimal(spigot.Ei(1), "1.89511")
        self.assertDecimal(spigot.Ein(1), "0.79659")
        self.assertDecimal(spigot.factorial(1.75), "1.60835")
        self.assertDecimal(spigot.floor(spigot.pi), "3")
        self.assertDecimal(spigot.frac(spigot.pi), "0.14159")
        self.assertDecimal(spigot.FresnelC(1), "0.77989")
        self.assertDecimal(spigot.FresnelS(1), "0.43825")
        self.assertDecimal(spigot.Wn(-0.125), "-3.26168")
        self.assertDecimal(spigot.W(-0.125), "-0.14442")
        self.assertDecimal(spigot.lgamma(10), "12.80182")
        self.assertDecimal(spigot.log10(120), "2.07918")
        self.assertDecimal(spigot.log1p(-0.0625), "-0.06453")
        self.assertDecimal(spigot.log2(120), "6.90689")
        self.assertDecimal(spigot.Li(20), "8.86013")
        self.assertDecimal(spigot.Li2(0.1), "0.10261")
        self.assertDecimal(spigot.Phi(6), "0.999999999013")
        self.assertDecimal(spigot.norm(6), "0.999999999013")
        self.assertDecimal(spigot.Phiinv(Fraction(1,10**9)), "-5.99780")
        self.assertDecimal(spigot.invPhi(Fraction(1,10**9)), "-5.99780")
        self.assertDecimal(spigot.norminv(Fraction(1,10**9)), "-5.99780")
        self.assertDecimal(spigot.invnorm(Fraction(1,10**9)), "-5.99780")
        self.assertDecimal(spigot.probit(Fraction(1,10**9)), "-5.99780")
        self.assertDecimal(spigot.sign(-spigot.pi), "-1")
        self.assertDecimal(spigot.sin(1), "0.84147")
        self.assertDecimal(spigot.sind(1), "0.01745")
        self.assertDecimal(spigot.sinh(1), "1.17520")
        self.assertDecimal(spigot.sinc(0.5), "0.95885")
        self.assertDecimal(spigot.sincn(0.5), "0.63661")
        self.assertDecimal(spigot.sqrt(2), "1.41421")
        self.assertDecimal(spigot.Si(1), "0.94608")
        self.assertDecimal(spigot.tan(1), "1.55740")
        self.assertDecimal(spigot.tand(1), "0.01745")
        self.assertDecimal(spigot.tanh(1), "0.76159")
        self.assertDecimal(spigot.gamma(0.5), "1.77245")
        self.assertDecimal(spigot.tgamma(0.5), "1.77245")
        self.assertDecimal(spigot.UFresnelC(1), "0.90452")
        self.assertDecimal(spigot.UFresnelS(1), "0.31026")
        self.assertDecimal(spigot.li(20), "9.90529")
        self.assertDecimal(spigot.si(20), "-0.02255")
        self.assertDecimal(spigot.zeta(Fraction(4,3)), "3.60093")

        # Binary functions
        self.assertDecimal(spigot.atan2(3, 4), "0.64350")
        self.assertDecimal(spigot.atan2d(3, 4), "36.86989")
        self.assertDecimal(spigot.En(3, 4), "0.00276")
        self.assertDecimal(spigot.pow(10, Fraction(1,3)), "2.15443")
        self.assertDecimal(spigot.agm(1,10), "4.25040")
        self.assertDecimal(spigot.Hg([0.5,-0.5],[0.5],.25), "0.86602")
        self.assertDecimal(spigot.BesselJ(1,2), "0.57672")
        self.assertDecimal(spigot.BesselI(2,3), "2.24521")
        self.assertDecimal(spigot.fmod(-8.9,3), "-2.9")

        # Variadic functions
        self.assertDecimal(spigot.algebraic(1,2,1,1,-1), "1.61803")
        self.assertDecimal(spigot.algebraic(1,2,1,1,1,-1), "1.83928")
        self.assertDecimal(spigot.hypot(2,3), "3.60555")
        self.assertDecimal(spigot.hypot(2,3,4), "5.38516")
        self.assertDecimal(spigot.log(2), "0.69314")
        self.assertDecimal(spigot.log(2,3), "0.63092")

    def testBaseFormat(self):
        """Test the base_format() method of the Spigot class.

        We must check that all the optional parameters have the right
        effects. But, as with testFunctions, we're not after a
        rigorous numerical check of the hard corner cases; we're only
        testing the plumbing between client Python code and the spigot
        core, so we just need to make sure that each piece of Python
        API connects the right piece of underlying functionality.

        """

        # Test that all the rounding modes correspond to the right
        # ones.
        halfway1 = spigot.fraction(1000005, 1000000)
        halfway2 = spigot.fraction(1000015, 1000000)
        roundingTests = [
            # input, digits, values, directions
            (spigot.pi,  5, {'d': "3.14159", 'u': "3.14160"}, "dududddddd"),
            (spigot.pi,  4, {'d': "3.1415",  'u': "3.1416"},  "duduuuuuuu"),
            (-spigot.pi, 5, {'d':"-3.14159", 'u':"-3.14160"}, "uddudddddd"),
            (halfway1,   5, {'d': "1.00000", 'u': "1.00001"}, "dududududu"),
            (halfway2,   5, {'d': "1.00001", 'u': "1.00002"}, "duduuddudu"),
            (-halfway1,  5, {'d':"-1.00000", 'u':"-1.00001"}, "udduduuddu"),
            (-halfway2,  5, {'d':"-1.00001", 'u':"-1.00002"}, "udduududdu"),
        ]

        for value, digits, outputs, directions in roundingTests:
            for mode, direction in zip(self.roundingModes, directions):
                output = value.base_format_str(digitlimit=digits, rmode=mode)
                self.assertEqual(output, outputs[direction])

        # Check that all the other optional parameters to base_format
        # (well, base_format_str) do the expected thing in at least
        # one case.
        self.assertEqual(spigot.pi.base_format_str(
            base=10, digitlimit=5), "3.14159")
        self.assertEqual(spigot.pi.base_format_str(
            base=16, digitlimit=5), "3.243f6")
        self.assertEqual(spigot.pi.base_format_str(
            base=16, uppercase=True, digitlimit=5), "3.243F6")
        self.assertEqual(spigot.pi.base_format_str(
            digitlimit=5, minintdigits=5), "00003.14159")
        self.assertEqual((1000000 * spigot.pi).base_format_str(
            digitlimit=5, minintdigits=5), "3141592.65358")

    def testFormat(self):
        "Test all the features of Spigot.__format__."

        # Most of these format strings are tested against both spigot
        # and ordinary floats, because partly what I'm checking is
        # that they're handled the same way as ordinary floats would,
        # so that people used to standard float formatting won't find
        # things behaving unexpectedly.
        for pi, e, two, which in [(spigot.pi, spigot.e, spigot.Spigot(2), "s"),
                                  (math.pi, math.exp(1), 2.0, "f")]:
            self.assertEqual("{:.6}".format(pi), "3.14159")

            if which == "s":
                # Check that this really is calling _spigot's_ format
                # function, and hasn't just coerced everything to float
                # first. (Otherwise, all the rest of these tests might
                # turn out to be testing the standard float formatter
                # against itself!)
                self.assertEqual("{:.40}".format(pi),
                                 "3.141592653589793238462643383279502884197")

            self.assertEqual("{:.6}".format(e), "2.71828")
            self.assertEqual("{:.6}".format(e*10), "27.1828")
            self.assertEqual("{:.6}".format(e/10), "0.271828")
            self.assertEqual("{:.6}".format(e/100000), "2.71828e-05")
            self.assertEqual("{:.6g}".format(e), "2.71828")
            self.assertEqual("{:.6g}".format(e*10), "27.1828")
            self.assertEqual("{:.6g}".format(e/10), "0.271828")
            self.assertEqual("{:.6g}".format(e/100000), "2.71828e-05")
            self.assertEqual("{:.6G}".format(e/100000), "2.71828E-05")
            self.assertEqual("{:.6f}".format(e), "2.718282")
            self.assertEqual("{:.6f}".format(e*10), "27.182818")
            self.assertEqual("{:.6f}".format(e/10), "0.271828")
            self.assertEqual("{:.6f}".format(e/100000), "0.000027")
            self.assertEqual("{:.6F}".format(e/100000), "0.000027")
            self.assertEqual("{:.6e}".format(e), "2.718282e+00")
            self.assertEqual("{:.6e}".format(e*10), "2.718282e+01")
            self.assertEqual("{:.6e}".format(e/10), "2.718282e-01")
            self.assertEqual("{:.6e}".format(e/100000), "2.718282e-05")
            self.assertEqual("{:.6E}".format(e/100000), "2.718282E-05")

            self.assertEqual("{:.12}".format(pi), "3.14159265359")
            self.assertEqual("{:.12}".format(-pi), "-3.14159265359")
            self.assertEqual("{:+.12}".format(pi), "+3.14159265359")
            self.assertEqual("{:+.12}".format(-pi), "-3.14159265359")
            self.assertEqual("{: .12}".format(pi), " 3.14159265359")
            self.assertEqual("{: .12}".format(-pi), "-3.14159265359")
            self.assertEqual("{:-.12}".format(pi), "3.14159265359")
            self.assertEqual("{:-.12}".format(-pi), "-3.14159265359")

            self.assertEqual("{:15.12}".format(pi), "  3.14159265359")
            self.assertEqual("{:>15.12}".format(pi), "  3.14159265359")
            self.assertEqual("{:<15.12}".format(pi), "3.14159265359  ")
            self.assertEqual("{:^15.12}".format(pi), " 3.14159265359 ")
            self.assertEqual("{:^16.12}".format(pi), " 3.14159265359  ")
            self.assertEqual("{:=15.12}".format(pi), "  3.14159265359")

            self.assertEqual("{:=15.12}".format(-pi), "- 3.14159265359")
            self.assertEqual("{:>15.12}".format(-pi), " -3.14159265359")
            self.assertEqual("{:15.12}".format(-pi), " -3.14159265359")
            self.assertEqual("{:=+15.12}".format(-pi), "- 3.14159265359")
            self.assertEqual("{:>+15.12}".format(-pi), " -3.14159265359")
            self.assertEqual("{:+15.12}".format(-pi), " -3.14159265359")
            self.assertEqual("{:=+15.12}".format(pi), "+ 3.14159265359")
            self.assertEqual("{:>+15.12}".format(pi), " +3.14159265359")
            self.assertEqual("{:+15.12}".format(pi), " +3.14159265359")

            self.assertEqual("{:*<+16.12}".format(pi), "+3.14159265359**")
            self.assertEqual("{:*>+16.12}".format(pi), "**+3.14159265359")
            self.assertEqual("{:*^+16.12}".format(pi), "*+3.14159265359*")
            self.assertEqual("{:*=+16.12}".format(pi), "+**3.14159265359")

            self.assertEqual("{:+016.12}".format(pi), "+003.14159265359")

            self.assertEqual("{:.12}".format(pi*100), "314.159265359")
            self.assertEqual("{:.12}".format(pi*1000), "3141.59265359")
            self.assertEqual("{:.12}".format(pi*1000000), "3141592.65359")
            self.assertEqual("{:,.12}".format(pi*100), "314.159265359")
            self.assertEqual("{:,.12}".format(pi*1000), "3,141.59265359")
            self.assertEqual("{:,.12}".format(pi*1000000), "3,141,592.65359")

            self.assertEqual("{:.13g}".format(pi), "3.14159265359")
            self.assertEqual("{:.3g}".format(two), "2")
            self.assertEqual("{:.1g}".format(two), "2")
            if which == "s" or sys.version_info.major >= 3:
                # Python 2 standard float formatting doesn't support
                # #, so omit these tests.
                self.assertEqual("{:#.13g}".format(pi), "3.141592653590")
                self.assertEqual("{:#.3g}".format(two), "2.00")
                self.assertEqual("{:#.1g}".format(two), "2.")

        # Our default precision is set just a little higher than
        # float, to make the point that we have more precision :-)
        self.assertEqual("{}".format(spigot.pi), "3.1415926535897932385")

        # And only spigot supports the rounding-mode extension to the
        # format-specifier syntax, so we test that outside the above
        # loop.
        halfway1 = spigot.fraction(1000005, 1000000)
        halfway2 = spigot.fraction(1000015, 1000000)
        roundingModes = ["d","u","z","a","ne","no","nd","nu","nz","na"]
        roundingFormatTests = [
            # input, digits, values, directions
            (spigot.pi,  5, {'d': "3.14159", 'u': "3.14160"}, "dududddddd"),
            (spigot.pi,  4, {'d': "3.1415",  'u': "3.1416"},  "duduuuuuuu"),
            (-spigot.pi, 5, {'d':"-3.14159", 'u':"-3.14160"}, "uddudddddd"),
            (halfway1,   5, {'d': "1.00000", 'u': "1.00001"}, "dududududu"),
            (halfway2,   5, {'d': "1.00001", 'u': "1.00002"}, "duduuddudu"),
            (-halfway1,  5, {'d':"-1.00000", 'u':"-1.00001"}, "udduduuddu"),
            (-halfway2,  5, {'d':"-1.00001", 'u':"-1.00002"}, "udduududdu"),
        ]
        for value, digits, outputs, directions in roundingFormatTests:
            for i, rmode in enumerate(roundingModes):
                self.assertEqual("{{:.{:d}r{}f}}".format(
                    digits, rmode).format(value), outputs[directions[i]])

    def testRound(self):
        irrat = spigot.pi * 100000
        halfway1 = spigot.fraction(1000005, 10)
        halfway2 = spigot.fraction(1000015, 10)
        roundToIntegerTests = [
            # input, values, directions
            (irrat,      {'d': 314159, 'u': 314160}, "dududddddd"),
            (-irrat,     {'d':-314159, 'u':-314160}, "uddudddddd"),
            (halfway1,   {'d': 100000, 'u': 100001}, "dududududu"),
            (halfway2,   {'d': 100001, 'u': 100002}, "duduuddudu"),
            (-halfway1,  {'d':-100000, 'u':-100001}, "udduduuddu"),
            (-halfway2,  {'d':-100001, 'u':-100002}, "udduududdu"),
        ]
        for inputVal, outputVals, directionList in roundToIntegerTests:
            directionDict = dict(zip(self.roundingModes, directionList))
            for abbr, rmode in self.roundingModeAbbrs.items():
                expectedOutput = outputVals[directionDict[rmode]]
                actualOutput = getattr(spigot, "round_" + abbr)(inputVal)
                self.assertEqual(expectedOutput, actualOutput)
                actualOutput = spigot.round(inputVal, rmode)
                self.assertEqual(expectedOutput, actualOutput)

                fracOutput = getattr(spigot, "fracpart_" + abbr)(inputVal)
                self.assertEqualNDigits(fracOutput, inputVal - actualOutput, 3)
                fracOutput = spigot.fracpart(inputVal, rmode)
                self.assertEqualNDigits(fracOutput, inputVal - actualOutput, 3)

                remainderOutput = getattr(spigot, "remainder_" + abbr)(
                    3 * inputVal, 3)
                self.assertEqualNDigits(remainderOutput, 3 * fracOutput, 3)
                remainderOutput = spigot.remainder(3 * inputVal, 3, rmode)
                self.assertEqualNDigits(remainderOutput, 3 * fracOutput, 3)

    def testToDigits(self):
        """Test Spigot.to_digits()."""

        self.assertEqual(list(spigot.Spigot("0.123").to_digits()),
                         [0,1,2,3])
        self.assertEqual(list(spigot.Spigot("-0.123").to_digits()),
                         [spigot.BASE_NEG(0),1,2,3])
        self.assertEqual(list(spigot.Spigot("-0.123").to_digits(
            positive_fraction=True)),
                         [-1,8,7,7])
        self.assertEqual(list(spigot.Spigot("0.123").to_digits(base=100)),
                         [0,12,30])

        ds = spigot.e.to_digits()
        self.assertEqual([ds.get_digit(i) for i in range(1,50)],
                         [2] + [1] * 48)

    def testToIEEE(self):
        """Test the to_ieee_* method family."""
        self.assertEqual(spigot.fraction(1,3).to_ieee_d(),
                         "3fd5555555555555")
        self.assertEqual(spigot.fraction(1,3).to_ieee_d(4),
                         "3fd5555555555555.5")
        self.assertEqual(spigot.fraction(1,3).to_ieee_d(5),
                         "3fd5555555555555.58") # rounds up
        self.assertEqual(spigot.fraction(1,3).to_ieee_d(6),
                         "3fd5555555555555.54") # rounds down
        self.assertEqual(spigot.fraction(1,3).to_ieee_d(-4),
                         "3fd5555555555550")

        self.assertEqual(spigot.fraction(1,3).to_ieee_s(), "3eaaaaab")
        self.assertEqual(spigot.fraction(1,3).to_ieee_s(4), "3eaaaaaa.b")
        self.assertEqual(spigot.fraction(1,3).to_ieee_h(), "3555")
        self.assertEqual(spigot.fraction(1,3).to_ieee_h(4), "3555.5")
        self.assertEqual(spigot.fraction(1,3).to_ieee_q(),
                         "3ffd5555555555555555555555555555")
        self.assertEqual(spigot.fraction(1,3).to_ieee_q(4),
                         "3ffd5555555555555555555555555555.5")

        self.assertEqual(spigot.fraction(1,3).to_ieee_d(
            5, rmode=spigot.ROUND_DOWN),
                         "3fd5555555555555.50")
        self.assertEqual(spigot.fraction(1,3).to_ieee_s(
            4, rmode=spigot.ROUND_DOWN), "3eaaaaaa.a")
        self.assertEqual(spigot.fraction(1,3).to_ieee_h(
            4, rmode=spigot.ROUND_UP), "3555.6")
        self.assertEqual(spigot.fraction(1,3).to_ieee_q(
            4, rmode=spigot.ROUND_UP),
                         "3ffd5555555555555555555555555555.6")

    def testContinuedFractionOutput(self):
        """Test the continued-fraction generators."""

        # Test taking some elements from the start of an infinite
        # stream of continued-fraction coefficients or convergents.
        self.assertEqual(
            list(itertools.islice(spigot.pi.to_cfrac(), 0, 20)),
            [3,7,15,1,292,1,1,1,2,1,3,1,14,2,1,1,2,2,2,2])
        self.assertEqual(
            list(itertools.islice(spigot.phi.to_convergents(), 0, 9)),
            [(1,1),(2,1),(3,2),(5,3),(8,5),(13,8),(21,13),(34,21),(55,34)])

        # Test that _terminating_ sequences really do terminate.
        self.assertEqual(
            list(spigot.fraction(57,22).to_cfrac()),
            [2, 1, 1, 2, 4])
        self.assertEqual(
            list(spigot.fraction(57,22).to_convergents()),
            [(2,1),(3,1),(5,2),(13,5),(57,22)])

        # Test that integers larger than a machine word are passed
        # through correctly.
        HUGE = 0x2873648376498713465928734659827465987365834645
        value = spigot.Spigot(1+1/(2+1/(3+1/(HUGE+1/Fraction(5)))))
        self.assertEqual(
            list(value.to_cfrac()),
            [1, 2, 3, HUGE, 5])
        self.assertEqual(
            list(value.to_convergents()),
            [(1,1),(3,2),(10,7),(HUGE*10+3,HUGE*7+2),(HUGE*50+25,HUGE*35+17)])

        # Test retrieval of convergents in the form of a Fraction.
        self.assertEqual(
            list(spigot.fraction(13,9).to_convergents(Fraction)),
            [Fraction(1), Fraction(3,2), Fraction(13,9)])

    def testContinuedFractionInput(self):
        "Test constructing a spigot object from a continued fraction."

        # Test case is the 'continued fraction constant' whose
        # continued fraction terms are simply the consecutive
        # integers, i.e. 0;1,2,3,4,5,6... and whose decimal expansion
        # is given by https://oeis.org/A052119 .
        cfc_func = spigot.from_cfrac(lambda n: n)
        cfc_itertools = spigot.from_cfrac(itertools.count())
        cfc_expected = "0.697774657964007982006790592551752599486658262998021"
        self.assertDecimal(cfc_func, cfc_expected)
        self.assertDecimal(cfc_itertools, cfc_expected)

        # Now test _again_, to ensure that the iterator-based spigot
        # object hasn't been destructively modified by consuming
        # elements of its iterator.
        self.assertDecimal(cfc_func, cfc_expected)
        self.assertDecimal(cfc_itertools, cfc_expected)

        # And just to be sure, do some arithmetic on both.
        self.assertDecimal(cfc_func + 1, "1." + cfc_expected[2:])
        self.assertDecimal(cfc_itertools + 2, "2." + cfc_expected[2:])
        self.assertDecimal(cfc_func + 3, "3." + cfc_expected[2:])
        self.assertDecimal(cfc_itertools + 4, "4." + cfc_expected[2:])

        # Check we can also access from_cfrac as a class method on
        # Spigot itself.
        cfc_classmethod = spigot.Spigot.from_cfrac(lambda n: n)
        self.assertDecimal(cfc_classmethod, cfc_expected)

    def testBaseNotationInput(self):
        "Test constructing a spigot object from a digit iterator."

        # Simple examples
        self.assertDecimal(spigot.from_digits(
            itertools.chain([1,2,3], itertools.cycle([4,5,6]))),
                           "1.23456456456")
        self.assertDecimal(spigot.from_digits(
            itertools.chain([0,2,3], itertools.cycle([4,5,6]))),
                           "0.23456456456")

        # Giving a negative integer part means the fractional digits
        # are still treated with positive sense: (-1) + 0.23456456...
        self.assertDecimal(spigot.from_digits(
            itertools.chain([-1,2,3], itertools.cycle([4,5,6]))),
                           "-0.76543543543")
        # Wrapping the integer part with spigot.BASE_NEG (a notional
        # 'minus sign') means the whole number is negated including
        # the fractional digits
        self.assertDecimal(spigot.from_digits(
            itertools.chain([spigot.BASE_NEG(1),2,3],
                            itertools.cycle([4,5,6]))),
                           "-1.23456456456")

        # You can specify individual digits to be interpreted in a
        # different base
        self.assertDecimal(spigot.from_digits(
            itertools.chain([1,2,3], itertools.cycle(
                [4,5,6,spigot.BASE_DIGIT(100, 78)]))),
                           "1.234567845678456784567845678")

        # And you can specify the base up front
        self.assertDecimal(spigot.from_digits(
            itertools.chain([1,2,3], itertools.cycle([4,5,6])), base=7),
                           "1.36108127461510920157") # or exactly 7603/5586

        # Check that this function also works as a class method of
        # spigot.Spigot
        self.assertDecimal(spigot.Spigot.from_digits(
            itertools.chain([1,2,3], itertools.cycle([4,5,6])), base=7),
                           "1.36108127461510920157")

        # You can also provide a function returning the nth digit, in
        # place of an iterator yielding the digits one by one
        self.assertDecimal(spigot.from_digits(
            lambda n: (1+n if n<3 else 4+(n%3))),
                           "1.23456456456")
        self.assertDecimal(spigot.from_digits(
            lambda n: (-1 if n==0 else
                       1+n if n<3 else 4+(n%3))),
                           "-0.76543543543")
        self.assertDecimal(spigot.from_digits(
            lambda n: (spigot.BASE_NEG(1) if n==0 else
                       1+n if n<3 else 4+(n%3))),
                           "-1.23456456456")
        # A stunt example for base-changing: express e as 1.1111111...
        # in a factorial base
        self.assertDecimal(spigot.from_digits(
            lambda n: (2 if n == 0 else spigot.BASE_DIGIT(n+1, 1))),
                           "2.71828182845904523536")

    def testExceptions(self):
        "Test that exceptions in spigot are handled sensibly."

        # Lexer error
        with self.assertRaisesRegex(
                ValueError, "expected.*digits after.*ieee"):
            spigot.eval("ieee:12345")

        # Parser error
        with self.assertRaisesRegex(
                ValueError, "parameter name 'x' repeated"):
            spigot.eval("let f(x,y,x)=3 in 2")

        # Setup error
        with self.assertRaisesRegex(
                ValueError, "log of zero"):
            spigot.log(0)

        # Error that doesn't happen until after setup
        bad_value = spigot.W(-spigot.exp(-0.99999999999))
        with self.assertRaisesRegex(
                ValueError, "W of less than -1/e"):
            "{}".format(bad_value)
        with self.assertRaisesRegex(
                ValueError, "W of less than -1/e"):
            list(itertools.islice(bad_value.to_cfrac(), 0, 10))

    def testNoFileReading(self):
        "Test that file- and fd-reading expression syntax is forbidden."

        with self.assertRaises(ValueError):
            spigot.eval("base10file:thing.txt")
        with self.assertRaises(ValueError):
            spigot.eval("cfracfile:thing.txt")
        with self.assertRaises(ValueError):
            spigot.eval("base10fd:0")
        with self.assertRaises(ValueError):
            spigot.eval("cfracfd:0")
        with self.assertRaises(ValueError):
            spigot.eval("base10stdin")
        with self.assertRaises(ValueError):
            spigot.eval("cfracstdin")

    def testFileObject(self):
        """Test that we can make a spigot out of a file-like object.

        We test this using lots of different file-like objects: a real
        disk file with and without codecs front end, an io.StringIO
        and an io.BytesIO, and completely made-up user-defined object
        types returning both Unicode and raw data bytes.

        """

        class FakeFile(object):
            def __init__(self, text, converter=lambda x:x):
                self.pos = 0
                self.text = text
                self.converter = converter
            def read(self, n):
                n = min(n, len(self.text) - self.pos)
                oldpos = self.pos
                self.pos += n
                return self.converter(self.text[oldpos:self.pos])
            def close(self):
                pass

        def make_tempfile(u):
            tf = tempfile.NamedTemporaryFile()
            tf.write(u.encode('ascii'))
            tf.seek(0)
            return tf

        file_constructors = [
            lambda u: FakeFile(u),
            lambda u: FakeFile(u.encode('ascii')),
            lambda u: io.StringIO(u),
            lambda u: io.BytesIO(u.encode('ascii')),
            lambda u: make_tempfile(u),
            lambda u: codecs.getreader('ascii')(make_tempfile(u)),
        ]

        for fcons in file_constructors:
            u = u"1.3333333333333333333333333333333333333333333"
            self.assertDecimal(spigot.from_file(fcons(u)),
                               "1.333333333333")
            self.assertDecimal(spigot.from_file(fcons(u), base=8),
                               "1.42857142857")

            # Check that spigot.Spigot.from_file does the same as
            # spigot.from_file
            self.assertDecimal(spigot.Spigot.from_file(fcons(u), base=8),
                               "1.42857142857")

            u = u"1.333333333"
            self.assertEqual(spigot.from_file(fcons(u), exact=True),
                             spigot.eval(u))
            with self.assertRaises(spigot.EndOfFile):
                self.assertEqual(spigot.from_file(fcons(u), exact=False),
                                 spigot.eval(u))
                self.assertEqual(spigot.from_file(fcons(u)),
                                 spigot.eval(u))

            u = u"1;1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1"
            self.assertDecimal(spigot.from_cfrac_file(fcons(u)), "1.61803")

            u = u"1;2,3,4,5"
            self.assertEqual(spigot.from_cfrac_file(fcons(u), exact=True),
                             spigot.fraction(225, 157))
            with self.assertRaises(spigot.EndOfFile):
                self.assertEqual(spigot.from_cfrac_file(fcons(u), exact=False),
                                 spigot.fraction(225, 157))
                self.assertEqual(spigot.from_cfrac_file(fcons(u)),
                                 spigot.fraction(225, 157))

            # Again, check the corresponding class method of spigot.Spigot
            self.assertEqual(spigot.Spigot.from_cfrac_file(fcons(u),
                                                           exact=True),
                             spigot.fraction(225, 157))

    def testToint(self):
        "Test the Spigot methods to_int, floor_int, ceil_int and sign_int."

        half = spigot.fraction(1,2)
        roundingTests = [
            # input, values, directions
            (spigot.pi  * 10**5,  {'d': 314159, 'u': 314160}, "dududddddd"),
            (spigot.pi  * 10**4,  {'d':  31415, 'u':  31416}, "duduuuuuuu"),
            (-spigot.pi * 10**5,  {'d':-314159, 'u':-314160}, "uddudddddd"),
            (10**5 + half,        {'d': 100000, 'u': 100001}, "dududududu"),
            (10**5 + 1 + half,    {'d': 100001, 'u': 100002}, "duduuddudu"),
            (-(10**5 + half),     {'d':-100000, 'u':-100001}, "udduduuddu"),
            (-(10**5 + 1 + half), {'d':-100001, 'u':-100002}, "udduududdu"),
        ]

        for value, outputs, directions in roundingTests:
            for mode, direction in zip(self.roundingModes, directions):
                output = value.to_int(rmode=mode)
                self.assertIsInstance(output, numbers.Integral)
                self.assertEqual(output, outputs[direction])

                self.assertEqual(output, spigot.to_int(value, rmode=mode))

            floor = value.floor_int()
            ceil = value.ceil_int()
            self.assertIsInstance(floor, numbers.Integral)
            self.assertIsInstance(ceil, numbers.Integral)
            self.assertEqual(floor, min(outputs.values()))
            self.assertEqual(ceil, max(outputs.values()))

            self.assertEqual(floor, spigot.floor_int(value))
            self.assertEqual(ceil, spigot.ceil_int(value))

        # Test Spigot.sign_int
        self.assertEqual(spigot.cos(1).sign_int(), +1)
        self.assertEqual(spigot.cos(2).sign_int(), -1)
        self.assertEqual(spigot.sin(0).sign_int(), 0)
        self.assertEqual(spigot.sign_int(spigot.cos(1)), +1)
        self.assertEqual(spigot.sign_int(spigot.cos(2)), -1)
        self.assertEqual(spigot.sign_int(spigot.sin(0)), 0)

    def testMisc(self):
        "Miscellaneous tests that didn't fit nicely anywhere else."

        # Test spigot.fraction
        self.assertRational(spigot.fraction(3,7), Fraction(3,7))
        self.assertRational(spigot.fraction(-3,7), Fraction(-3,7))

        # Ensure spigot.fraction can be used as an argument to
        # to_convergents
        cs = spigot.phi.to_convergents(spigot.fraction)
        self.assertEqual(next(cs), spigot.Spigot(1))
        self.assertEqual(next(cs), spigot.Spigot(2))
        self.assertEqual(next(cs), spigot.Spigot(3)/2)
        self.assertEqual(next(cs), spigot.Spigot(5)/3)

        # Check the abbreviated rounding mode names
        self.assertEqual(self.roundingModes, [
            spigot.RD, spigot.RU, spigot.RZ, spigot.RA,
            spigot.RNE, spigot.RNO,
            spigot.RND, spigot.RNU, spigot.RNZ, spigot.RNA,
        ])
        # And the various alternative names
        self.assertEqual(self.roundingModes, [
            spigot.RM, spigot.RP, spigot.RZ, spigot.RI,
            spigot.RN, spigot.RNO,
            spigot.RNM, spigot.RNP, spigot.RNZ, spigot.RNI,
        ])

class demo_program_tests(unittest.TestCase):
    def __init__(self, *args, **kws):
        super(demo_program_tests, self).__init__(*args, **kws)
        if not hasattr(self, "assertRegex"):
            # Work around Python 2 and 3 calling this method by
            # different names.
            self.assertRegex = self.assertRegexpMatches

    def demoOutput(self, command):
        "Run a demo-program command and return its output."

        programPath = os.path.join(
            os.path.dirname(os.path.abspath(__file__)),
            "..", "python-demos", command[0])

        realCommand = [sys.executable, programPath] + command[1:]

        return subprocess.check_output(realCommand).decode('ASCII')

    def expectDemoOutput(self, command, expectedOutput):
        actualOutput = self.demoOutput(command)
        self.assertEqual(actualOutput, expectedOutput)

    def expectDemoOutputRegexp(self, command, regexp):
        actualOutput = self.demoOutput(command)
        # I'd like to use re.match here, not re.search. So I prefix
        # and suffix the regexp with \A and \Z.
        reObject = re.compile(r'\A(' + regexp + r'\Z)', re.S)
        self.assertRegex(actualOutput, reObject)

    def testDemo(self):
        "Test demo"
        self.expectDemoOutput(["demo"], r'''
Real pi: 3.14159265358979323846264338327950288419716939937511
IEEE pi: 3.14159265358979311599796346854418516159057617187500
sin(exp(apery)) = -0.18430082724521179457
With f(x) = sin(x+1):
  f(1) = 0.9092974268256816954
  f(2) = 0.1411200080598672221
Sign of pi^e - exp(pi) = -1
tan of spigot's  10^100 =  0.4012319619908143541857543436532949583239
tan of Python's 10**100 =  0.4012319619908143541857543436532949583239
tan of IEEE    1.0e+100 = -0.4116229628832497988834983009940174394893
math.tan of IEEE 1e+100 = -0.4116229628832497877688467724510701373219
Decimal digits of e: 2,7,1,8,2,8,1,8,2,8,4,5,9,0,4,5,2,3,5,3,6,0,2,8,7,...
Base 10^20 digits of e: 2,71828182845904523536,...
Factorial-base digits of e: 2,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,...
Continued fraction of e: 2,1,2,1,1,4,1,1,6,1,1,8,1,1,10,1,1,12,1,1,14,1,1,...
Convergents of pi: 3/1,22/7,333/106,355/113,103993/33102,104348/33215,...
Continued fraction 0;1,2,3,4,5,6,... (https://oeis.org/A052119):
  lambda n: n     -> 0.69777465796400798200679059255175259948665826299802
  itertools.count -> 0.69777465796400798200679059255175259948665826299802
Continued fraction 0;1,1,2,3,5,8,... (https://oeis.org/A073822):
  generator       -> 0.58887395254893350767123112124678738407999084839132
Thue-Morse constant (http://oeis.org/A014571) =
  0.4124540336401075977833613682584552830894783744557695575733794153487936
Continued fraction of Champernowne constant (https://oeis.org/A030167):
  0,8,9,1,149083,1,1,1,4,1,1,1,3,4,1,1,1,15, then a 166-digit number
'''[1:])

    def testPowbegin(self):
        "Test powbegin"
        # powbegin has its own internal test suite. Run that (or
        # rather, run the reasonably quick -t part of it, not the
        # enormous soak-test suite).
        self.expectDemoOutputRegexp(["powbegin", "-t"], r'''
passed \d+ failed 0
'''[1:])

        # But there are parts of the Python spigot API that that
        # actively avoids exercising, namely the use of spigot.eval in
        # command-line argument evaluation. So we should check that
        # too, with both simple integer input strings and ones
        # complicated enough to really need spigot.eval.
        self.expectDemoOutput(["powbegin", "2", "5"], "9\n")
        self.expectDemoOutput(["powbegin", "pi", "315"], "176\n")
        self.expectDemoOutput(["powbegin",
                               "phi", "1234", "-M", "1/sqrt(5)"], "1629\n")

    def testMediant(self):
        "Test mediant"
        self.expectDemoOutput(["mediant", "eulergamma^2", "1/3"],
                              "715/2146\n")

    def testPythangle(self):
        "Test pythangle"
        self.expectDemoOutput(["pythangle",
                               "-l10", "-a", "-d", "cosh(4.43)"], r'''
3 4 5 angle=53.13010235415597870314
21 20 29 angle=43.60281897270362354049
55 48 73 angle=41.11209043916692861659
72 65 97 angle=42.07502205084363352026
4545 4088 6113 angle=41.96979567176438499623
151984 136713 204425 angle=41.97208782384589403726
235412 211755 316637 angle=41.97163741719610677956
382851 344380 514949 angle=41.97183808361007049148
16399101 14751140 22057349 angle=41.97166807776027225764
38590984 34712937 51906185 angle=41.97166568305859880912
'''[1:])

if __name__ == "__main__":
    unittest.main()
