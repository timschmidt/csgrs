/***************************************************************************
 *            test.hpp
 *
 *  Copyright  2022  Luca Geretti
 ****************************************************************************/

/*
 * This file is part of ConcLog, under the MIT license.
 *
 * Permission is hereby granted, free of charge, to any person obtaining a copy
 * of this software and associated documentation files (the "Software"), to deal
 * in the Software without restriction, including without limitation the rights
 * to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
 * copies of the Software, and to permit persons to whom the Software is furnished
 * to do so, subject to the following conditions:
 *
 * The above copyright notice and this permission notice shall be included in all
 * copies or substantial portions of the Software.
 *
 * THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
 * IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY, FITNESS
 * FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE AUTHORS OR
 * COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER LIABILITY, WHETHER
 * IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM, OUT OF OR IN CONNECTION
 * WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE SOFTWARE.
 */

/*!\file test.hpp
 * \brief Macros for test suite.
 */

#ifndef CONCLOG_TEST_HPP
#define CONCLOG_TEST_HPP

#include <cstdlib>
#include <cstring>
#include <iostream>
#include <exception>

int CONCLOG_TEST_FAILURES = 0;
int CONCLOG_TEST_SKIPPED = 0;
std::string CONCLOG_CURRENT_TESTING_CLASS = "???";

// This needs to be a function since we do not want to evaluate the result twice,
// and can't store it in a variable since we don't know it's type.
template<class R, class ER>
bool
CONCLOG_check(std::ostream& os, const R& r, const ER& er) {
    os << r << std::flush; return (r==er);
}


//This is the variable that stores counter for the number of test cases
//The value is used and updated in the next two macro definitions
int test_case_counter = 0;

/*! \brief Tests a class function */
#define CONCLOG_TEST_CLASS(classname,testclassconstruct)                       \
    { \
        std::cout << "****************************************\n"       \
                  << "TESTING CLASS " << #classname << "\n"                    \
                  << "****************************************\n" << std::endl; \
        CONCLOG_CURRENT_TESTING_CLASS=#classname; \
        testclassconstruct.test(); \
    } \

/*! \brief Print the title for the test case */
#define CONCLOG_PRINT_TEST_CASE_TITLE( pTitle )                         \
    {                                                                   \
        std::cout << "*** " << ++test_case_counter << ": "<< pTitle << " ***" << std::endl << std::endl; \
        std::cout.flush();                                                   \
    }                                                                   \

/*! \brief Print the comment for the test */
#define CONCLOG_PRINT_TEST_COMMENT( pComment )                          \
    {                                                                   \
        std::cout << "* COMMENT: " << pComment << "" << std::endl;                \
        std::cout.flush();                                                   \
    }                                                                   \


/*! \brief Provide a warning message */
#define CONCLOG_TEST_WARN( message )                                    \
    {                                                                   \
        std::cout << "WARNING: " << message << "" << std::endl;                \
        std::cerr << "WARNING: " << message << "" << std::endl;                \
    }                                                                   \


/*! \brief Notify the user about a possibly unintuitive feature */
#define CONCLOG_TEST_NOTIFY( message )                                    \
    {                                                                   \
        std::cout << "NOTIFICATION: " << message << "" << std::endl;                \
        std::cerr << "NOTIFICATION: " << message << "" << std::endl;                \
    }                                                                   \


/*! \brief Catches an exception and writes a diagnostic to standard output and standard error. */
#define CONCLOG_TEST_CATCH(message)                                     \
    catch(const std::exception& except) {                                    \
        ++CONCLOG_TEST_FAILURES;                                        \
        std::cout << "exception: \"" << except.what() << "\"\n" << std::endl; \
        std::cerr << "ERROR: " << __FILE__ << ":" << __LINE__ << ": " << CONCLOG_PRETTY_FUNCTION << ": " << message << " throwed \"" << except.what() << "\"." << std::endl;     \
    }                                                                   \
    catch(...) {                                                        \
        ++CONCLOG_TEST_FAILURES;                                        \
        std::cout << "unknown exception\n" << std::endl;                \
        std::cerr << "ERROR: " << __FILE__ << ":" << __LINE__ << ": " << CONCLOG_PRETTY_FUNCTION << ": " << message << " throwed an unknown exception." << std::endl;       \
    }                                                                   \


/*! \brief Calls a function */
#define CONCLOG_TEST_CALL(function)                                     \
    {                                                                   \
        std::cout << "****************************************\n"       \
                  << "CALLING " << #function << "\n"                    \
                  << "****************************************\n" << std::endl; \
        try {                                                           \
            function;                                                   \
        } catch(const std::exception& except) {                              \
            ++CONCLOG_TEST_FAILURES;                                    \
            std::cout << "ERROR: exception '" << except.what() << "' in " << #function << ": "    \
                      << except.what() << std::endl;                         \
            std::cerr << "ERROR: " << __FILE__ << ":" << __LINE__ << ": calling " \
                      << #function << ": " << except.what() << std::endl; \
            std::cout << std::endl;                                     \
        }                                                               \
    }                                                                   \


/*! \brief Omits a test, with a warning message */
#define CONCLOG_TEST_SKIP(function)                                     \
    {                                                                   \
        std::cout << "****************************************\n"       \
                  << "SKIPPING " << #function << "\n"                   \
                  << "****************************************\n" << std::endl; \
        ++CONCLOG_TEST_SKIPPED;                                         \
        std::cout << std::endl;                                         \
    }                                                                   \


/*! \brief Executes \a statement, writing the statement to standard output. Does not check for any errors. */
#define CONCLOG_TEST_EXECUTE(statement)                                 \
    {                                                                   \
        std::cout << #statement << ": " << std::flush;                  \
        statement;                                                      \
        std::cout << " (ok)\n" << std::endl;                            \
    }                                                                   \


/*! \brief Tries to execute \a statement, writing the statement to standard output. Writes a diagnostic report to standard error if an exception is thrown. <br> <b>Important:</b> Use the CONCLOG_TEST_CONSTRUCT() macro if \a statement declares a variable and calls a constructor. */
#define CONCLOG_TEST_TRY(statement)                                     \
    {                                                                   \
        std::cout << #statement << ": " << std::flush;                  \
        try {                                                           \
            statement;                                                  \
            std::cout << " (ok)\n" << std::endl;                        \
        }                                                               \
            CONCLOG_TEST_CATCH("Statement `" << #statement << "'")      \
        }                                                               \


/*! \brief Writes the expression to the output. Does not catch errors. */
#define CONCLOG_TEST_PRINT(expression)                                  \
    {                                                                   \
        std::cout << #expression << " = " << std::flush;                \
        std::cout << (expression) << "\n" << std::endl;                 \
    }                                                                   \


/*! \brief Tries to evaluate \a expression, writing the expression and the result to standard ouput. Writes a diagnostic report to standard error if an exception is thrown. */
#define CONCLOG_TEST_EVALUATE(expression)                               \
    {                                                                   \
        std::cout << #expression << ": " << std::flush;                 \
        try {                                                           \
            std::cout << (expression) << "\n" << std::endl;             \
        }                                                               \
            CONCLOG_TEST_CATCH("Expression `" << #expression << "'")    \
        }                                                               \


/*! \brief Evaluates \a expression in a boolean context and checks if the result is \a true. */
#define CONCLOG_TEST_ASSERT(expression)                                 \
    {                                                                   \
        std::cout << #expression << ": " << std::flush;                 \
        auto result = (expression);                                     \
        if(result) {                                                    \
            std::cout << "true\n" << std::endl;                         \
        } else {                                                        \
            ++CONCLOG_TEST_FAILURES;                                    \
            std::cout << "\nERROR: false" << std::endl;                 \
            std::cerr << "ERROR: " << __FILE__ << ":" << __LINE__ << ": " << __FUNCTION__ << ": Assertion `" << #expression << "' failed." << std::endl; \
        }                                                               \
    }                                                                   \


/*! \brief Evaluates \a expression and checks if the result is equal to \a expected. */
#define CONCLOG_TEST_CHECK_WARN(expression,expected)                         \
    {                                                                   \
        std::cout << #expression << ": " << std::flush; \
        bool ok = CONCLOG_check(std::cout,expression,expected);         \
        if(ok) {                                                        \
            std::cout << "\n" << std::endl;                             \
        } else {                                                        \
            std::cout << "\nWARNING: expected " << #expression << " = " << #expected << " == " << (expected) << " \n" << std::endl; \
            std::cerr << "WARNING: " << __FILE__ << ":" << __LINE__ << ": " << CONCLOG_PRETTY_FUNCTION << ": Check `" << #expression << "==" << #expected << "' failed; obtained " << (expression) << std::endl; \
        }                                                               \
    }                                                                   \


/*! \brief Evaluates \a expression and checks if the result is equal to \a expected. */
#define CONCLOG_TEST_CHECK(expression,expected)                         \
    {                                                                   \
        std::cout << #expression << ": " << std::flush; \
        bool ok = CONCLOG_check(std::cout,expression,expected);         \
        if(ok) {                                                        \
            std::cout << "\n" << std::endl;                             \
        } else {                                                        \
            ++CONCLOG_TEST_FAILURES;                                    \
            std::cout << "\nERROR: expected " << #expression << " = " << #expected << " == " << (expected) << " \n" << std::endl; \
            std::cerr << "ERROR: " << __FILE__ << ":" << __LINE__ << ": " << CONCLOG_PRETTY_FUNCTION << ": Check `" << #expression << "==" << #expected << "' failed; obtained " << (expression) << std::endl; \
        }                                                               \
    }                                                                   \


/*! \brief Evaluates \a expression1 and expression2 and checks if the results are equal. */
#define CONCLOG_TEST_SAME(expression1,expression2)                         \
    {                                                                   \
        std::cout << "same(" << #expression1 << "," << #expression2 << "): " << std::flush; \
        bool ok = same((expression1), (expression2));                       \
        if(ok) {                                                        \
            std::cout << "true\n" << std::endl;                         \
        } else {                                                        \
            ++CONCLOG_TEST_FAILURES;                                    \
            std::cout << "\nERROR: " << #expression1 << ":\n           " << (expression1) \
                      << "\n     : " << #expression2 << ":\n           " << (expression2) << std::endl; \
            std::cerr << "ERROR: " << __FILE__ << ":" << __LINE__ << ": " << CONCLOG_PRETTY_FUNCTION << ": Identity `" << #expression1 << " === " << #expression2 << "' failed; " << #expression1 << "=" << (expression1) << "; " << #expression2 << "=" << (expression2) << std::endl; \
        }                                                               \
    }                                                                   \

/*! \brief Evaluates \a expression and checks if the result is the same as \a expected. */
#define CONCLOG_TEST_SAME_AS(expression,expected)                         \
    {                                                                   \
        std::cout << #expression << " == " << #expected << ": " << std::flush; \
        bool ok = same((expression), (expected));                       \
        if(ok) {                                                        \
            std::cout << "true\n" << std::endl;                         \
        } else {                                                        \
            ++CONCLOG_TEST_FAILURES;                                    \
            std::cout << "\nERROR: " << #expression << ":\n           " << (expression) << std::endl; \
            std::cerr << "ERROR: " << __FILE__ << ":" << __LINE__ << ": " << CONCLOG_PRETTY_FUNCTION << ": Sameness of `" << #expression << " and " << #expected << "' failed;" << std::endl; \
            std::cerr << "  " << #expression << "=" << (expression) << std::endl; \
            std::cerr << "  " << #expected << "=" << (expected) << std::endl; \
        }                                                               \
    }                                                                   \


/*! \brief Evaluates \a expression1 and expression2 and checks if the results are equal. */
#define CONCLOG_TEST_EQUAL(expression1,expression2)                         \
    {                                                                   \
        std::cout << #expression1 << " == " << #expression2 << ": " << std::flush; \
        bool ok = (expression1) == (expression2);                       \
        if(ok) {                                                        \
            std::cout << "true\n" << std::endl;                         \
        } else {                                                        \
            ++CONCLOG_TEST_FAILURES;                                    \
            std::cout << "\nERROR: " << #expression1 << ":\n           " << (expression1) \
                      << "\n     : " << #expression2 << ":\n           " << (expression2) << std::endl; \
            std::cerr << "ERROR: " << __FILE__ << ":" << __LINE__ << ": " << CONCLOG_PRETTY_FUNCTION << ": Equality `" << #expression1 << " == " << #expression2 << "' failed; " << #expression1 << "=" << (expression1) << "; " << #expression2 << "=" << (expression2) << std::endl; \
        }                                                               \
    }                                                                   \

/*! \brief Evaluates \a expression1 and expression2 and checks if the results are not equal. */
#define CONCLOG_TEST_NOT_EQUAL(expression1,expression2)                 \
    {                                                                   \
        std::cout << #expression1 << " != " << #expression2 << ": " << std::flush; \
        bool ok = (expression1) == (expression2);               \
        if(ok) {                                                        \
            ++CONCLOG_TEST_FAILURES;                                    \
            std::cout << "\nERROR: " << #expression1 << ":\n           " << (expression1) \
                      << "\n     : " << #expression2 << ":\n           " << (expression2) << std::endl; \
            std::cerr << "ERROR: " << __FILE__ << ":" << __LINE__ << ": " << CONCLOG_PRETTY_FUNCTION << ": Inequality `" << #expression1 << " != " << #expression2 << "' failed; " << #expression1 << "=" << (expression1) << "; " << #expression2 << "=" << (expression2) << std::endl; \
        } else {                                                        \
            std::cout << "true\n" << std::endl;                         \
        }                                                               \
    }                                                                   \

/*! \brief Evaluates \a expression and checks if the result is equal to \a expected. */
#define CONCLOG_TEST_EQUALS(expression,expected)                         \
    {                                                                   \
        std::cout << #expression << " == " << #expected << ": " << std::flush; \
        bool ok = (expression) == (expected);                       \
        if(ok) {                                                        \
            std::cout << "true\n" << std::endl;                         \
        } else {                                                        \
            ++CONCLOG_TEST_FAILURES;                                    \
            std::cout << "\nERROR: " << #expression << ":\n           " << (expression) << std::endl; \
            std::cerr << "ERROR: " << __FILE__ << ":" << __LINE__ << ": " << CONCLOG_PRETTY_FUNCTION << ": Equality `" << #expression << " == " << #expected << "' failed;" << std::endl; \
            std::cerr << "  " << #expression << "=" << (expression) << std::endl; \
            std::cerr << "  " << #expected << "=" << (expected) << std::endl; \
        }                                                               \
    }                                                                   \

/*! \brief Evaluates \a expression and checks if the result is within \a tolerance of \a expected. */
#define CONCLOG_TEST_WITHIN(expression,expected,tolerance)                         \
    {                                                                   \
        std::cout << #expression << " ~ " << #expected << ": " << std::flush; \
        auto error=mag(expression-expected); \
        bool ok = (error <= tolerance);                       \
        if(ok) {                                                        \
            std::cout << "true\n" << std::endl;                         \
        } else {                                                        \
            ++CONCLOG_TEST_FAILURES;                                    \
            std::cout << "\nERROR: " << #expression << ":\n           " << (expression) \
                      << "\n     : " << #expected << ":\n           " << (expected) \
                      << "\n     : error: " << (error) \
                      << "\n     : tolerance " << (tolerance) << std::endl; \
            std::cerr << "ERROR: " << __FILE__ << ":" << __LINE__ << ": " << CONCLOG_PRETTY_FUNCTION << ": ApproximateTag equality `" << #expression << " ~ " << #expected << "' failed; " << #expression << "=" << (expression) << "; " << #expected << "=" << (expected)<< "; error=" << (error) << "; tolerance=" << (tolerance) << std::endl; \
        }                                                               \
    }                                                                   \
                                                                   \


/*! \brief Evaluates \a expression and checks if the result is less than \a expected. */
#define CONCLOG_TEST_LESS(expression,expected)                         \
    {                                                                   \
        std::cout << #expression << " < " << #expected << ": " << std::flush; \
        bool ok = (expression) < (expected);                       \
        if(ok) {                                                        \
            std::cout << "true\n" << std::endl;                         \
        } else {                                                        \
            ++CONCLOG_TEST_FAILURES;                                    \
            std::cout << "\nERROR: " << #expression << ":\n           " << (expression) << std::endl; \
            std::cerr << "ERROR: " << __FILE__ << ":" << __LINE__ << ": " << CONCLOG_PRETTY_FUNCTION << ": Equality `" << #expression << " < " << #expected << "' failed; " << #expression << "=" << (expression) << std::endl; \
        }                                                               \
    }                                                                   \


/*! \brief Evaluates \a predicate(\a argument) and checks if the result is \tt true. */
#define CONCLOG_TEST_UNARY_PREDICATE(predicate,argument)    \
    {                                                                   \
        std::cout << #predicate << "(" << #argument << ") with " << #argument << "=" << (argument) << ": " << std::flush; \
        bool ok = (predicate((argument)));                  \
        if(ok) {                                                        \
            std::cout << "true\n" << std::endl;                         \
        } else {                                                        \
            ++CONCLOG_TEST_FAILURES;                                    \
            std::cout << "\nERROR: false" << std::endl;                 \
            std::cerr << "ERROR: " << __FILE__ << ":" << __LINE__ << ": " << CONCLOG_PRETTY_FUNCTION << ": Predicate `" << #predicate << "(" << #argument << ")' with " << #argument << "=" << (argument) << " is false." << std::endl; \
        }                                                               \
    }


/*! \brief Evaluates \a predicate(argument1,argument2) and checks if the result is \tt true. */
#define CONCLOG_TEST_BINARY_PREDICATE(predicate,argument1,argument2)    \
    {                                                                   \
        std::cout << #predicate << "(" << (#argument1) << "," << (#argument2) << ") with " << #argument1 << "=" << (argument1) << ", " << #argument2 << "=" << (argument2) << ": " << std::flush; \
        bool ok = predicate((argument1),(argument2));                  \
        if(ok) {                                                        \
            std::cout << "true\n" << std::endl;                         \
        } else {                                                        \
            ++CONCLOG_TEST_FAILURES;                                    \
            std::cout << "\nERROR: false" << std::endl;                 \
            std::cerr << "ERROR: " << __FILE__ << ":" << __LINE__ << ": " << CONCLOG_PRETTY_FUNCTION << ": Predicate `" << #predicate << "(" << #argument1 << "," << #argument2 << ")' with\n  " << #argument1 << "=" << (argument1) << ";\n  " << #argument2 << "=" << (argument2) << " is false." << std::endl; \
        }                                                               \
    }


/*! \brief Evaluates \a expression and checks if the result compares correctly with \a expected. */
#define CONCLOG_TEST_COMPARE(expression,comparison,expected)           \
    {                                                                   \
        std::cout << #expression << ": " << (expression) << std::flush; \
        bool ok = ((expression) comparison (expected));               \
        if(ok) {                                                        \
            std::cout << " " << #comparison << " " << (expected) << ": true\n" << std::endl; \
        } else {                                                        \
            ++CONCLOG_TEST_FAILURES;                                    \
            std::cout << "\nERROR: expected: " << #expression << #comparison << #expected << "=" << (expected) << std::endl; \
            std::cerr << "ERROR: " << __FILE__ << ":" << __LINE__ << ": " << CONCLOG_PRETTY_FUNCTION << ": Comparison `" << #expression << #comparison << #expected << "' failed; " << #expression << "=" << (expression) << "; " << #expected << "=" << (expected) << std::endl; \
        }                                                               \
    }                                                                   \


/*! \brief Evaluates \a expression, converts to \a Type, and checks if the result compares correctly with \a expected. */
#define CONCLOG_TEST_RESULT_COMPARE(Type,expression,comparison,expected) \
    {                                                                   \
        Type result=(expression);                                       \
        std::cout << #expression << ": " << result << std::flush; \
        bool ok = result comparison (expected);               \
        if(ok) {                                                        \
            std::cout << " " << #comparison << " " << (expected) << "\n" << std::endl; \
        } else {                                                        \
            ++CONCLOG_TEST_FAILURES;                                    \
            std::cout << "\nERROR: expected: " << #expression << #comparison << #expected << std::endl; \
            std::cerr << "ERROR: " << __FILE__ << ":" << __LINE__ << ": " << CONCLOG_PRETTY_FUNCTION << ": Comparison `" << #expression << #comparison << #expected << "' failed; " << #expression << "=" << result << "; " << #expected << "=" << (expected) << std::endl; \
        }                                                               \
    }                                                                   \


/*! \brief Declares an object \a variable of type \a Class (uses the default constructor). */
#define CONCLOG_TEST_DECLARE(Class,variable)                            \
    {                                                                   \
        std::cout << #Class << " " << #variable << ": " << std::flush;  \
        try {                                                           \
            Class variable;                                             \
            std::cout << #variable << "==" << variable << "\n" << std::endl; \
        }                                                               \
        CONCLOG_TEST_CATCH("Constructor `" << #Class << "" << #variable << "'") \
    }                                                                   \
    Class variable;                                                     \


/*! \brief Constructs object \a variable of type \a Class from \a expression. */
#define CONCLOG_TEST_CONSTRUCT(Class,variable,expression)               \
    std::cout << #Class << " " << #variable << "" << #expression << ": " << std::flush; \
    Class variable expression;                                          \
    std::cout << #variable << "==" << variable << "\n" << std::endl;    \

/*! \brief Constructs default object \a variable of type \a Class. */
#define CONCLOG_TEST_DEFAULT_CONSTRUCT(Class,variable)               \
    std::cout << #Class << " " << #variable << ": " << std::flush; \
    Class variable;                                                \
    std::cout << #variable << "==" << (variable) << "\n" << std::endl;    \

/*
#define CONCLOG_TEST_CONSTRUCT(Class,variable,expression)               \
    {                                                                   \
        std::cout << #Class << " " << #variable << "" << #expression << ": " << std::flush; \
        try {                                                           \
            Class variable expression;                                  \
            std::cout << #variable << "==" << variable << "\n" << std::endl; \
        }                                                               \
        CONCLOG_TEST_CATCH("Constructor `" << #Class << " " << #variable << "" << #expression << "'") \
    }                                                                   \
    Class variable expression;                                          \
*/

/*! \brief Constructs object \a variable of type \a Class from \a expression. */
#define CONCLOG_TEST_NAMED_CONSTRUCT(Class,variable,expression)               \
    {                                                                   \
        std::cout << #Class << " " << #variable << " = " << #Class << "::" << #expression << ": " << std::flush; \
        try {                                                           \
            Class variable = Class :: expression;                                  \
            std::cout << #variable << "==" << variable << "\n" << std::endl; \
        }                                                               \
        CONCLOG_TEST_CATCH("Named constructor `" << #variable << "=" << #Class << "::" << #expression << "'") \
    }                                                                   \
    Class variable = Class :: expression;                                          \


/*! \brief Construct object \a variable of type \a Class from \a expression using assignment syntax. */
#define CONCLOG_TEST_ASSIGN_CONSTRUCT(Class,variable, expression)       \
    {                                                                   \
        std::cout << #Class << " " << #variable << " = " << #expression << ": " << std::flush; \
        try {                                                           \
            Class variable = expression;                                \
            std::cout << #variable << "==" << variable << "\n" << std::endl;                 \
        }                                                               \
        CONCLOG_TEST_CATCH("Assignment `" << #variable << "=" << #expression << "'") \
    }                                                                   \
    Class variable = expression;                                        \

/*! \brief Assigns object \a variable from \a expression. */
#define CONCLOG_TEST_ASSIGN(variable, expression)                       \
    {                                                                   \
        std::cout << #variable << " = " << #expression << ": " << std::flush; \
        try {                                                           \
            variable=(expression);                                      \
            std::cout << variable << "\n" << std::endl;                 \
        }                                                               \
        CONCLOG_TEST_CATCH("Assignment `" << #variable << "=" << #expression << "'") \
            }                                                           \


/*! \brief Evaluates expression and expects an exception. */
#define CONCLOG_TEST_THROWS(statement,error)                             \
    {                                                                   \
        std::cout << #statement << ": " << std::flush;                  \
        try {                                                           \
            statement;                                                  \
            ++CONCLOG_TEST_FAILURES;                                    \
            std::cout << "\nERROR: expected " << #error << "; no exception thrown\n"; \
            std::cerr << "ERROR: " << __FILE__ << ":" << __LINE__ << ": " << CONCLOG_PRETTY_FUNCTION << ": expected " << #error << "; no exception thrown." << std::endl; \
        }                                                               \
        catch(const error& err) {                                         \
            std::cout << "caught " << #error << " as expected\n" << std::endl; \
        }                                                               \
        catch(const std::exception& except) {                                \
            ++CONCLOG_TEST_FAILURES;                                    \
            std::cout << "\nERROR: caught exception " << except.what() << "; expected " << #error << "\n"; \
            std::cerr << "ERROR: " << __FILE__ << ":" << __LINE__ << ": " << CONCLOG_PRETTY_FUNCTION << ": caught exception " << except.what() << "; expected " << #error << std::endl; \
        }                                                               \
    }                                                                   \


/*! \brief Evaluates expression and expects an exception. */
#define CONCLOG_TEST_FAIL(statement)                                    \
    {                                                                   \
        std::cout << #statement << ": " << std::flush;                  \
        try {                                                           \
            statement;                                                  \
            ++CONCLOG_TEST_FAILURES;                                    \
            std::cout << "\nERROR: expected exception; none thrown\n";  \
            std::cerr << "ERROR: " << __FILE__ << ":" << __LINE__ << ": " << CONCLOG_PRETTY_FUNCTION << ": expected exception; no exception thrown." << std::endl; \
        }                                                               \
        catch(...) {                                                    \
            std::cout << "caught exception as expected\n" << std::endl; \
        }                                                               \
    }                                                                   \


/*! \brief Evaluates \a expression in a boolean context and checks if the result is \a true. */
/*! Use variadic macro argument to allow template parameters */
#define CONCLOG_TEST_STATIC_ASSERT(...)                          \
    {                                                                   \
        std::cout << #__VA_ARGS__ << ": " << std::flush;                 \
        bool result = ((__VA_ARGS__::value));                                   \
        if(result) {                                                    \
            std::cout << "true\n" << std::endl;                         \
        } else {                                                        \
            ++CONCLOG_TEST_FAILURES;                                    \
            std::cout << "false\n" << std::endl;                 \
            std::cout << "ERROR: " << __FILE__ << ":" << __LINE__ << ": " << __FUNCTION__ << ": " << CONCLOG_CURRENT_TESTING_CLASS << ": Static assertion `" << #__VA_ARGS__ << "' failed." << "\n" << std::endl; \
            std::cerr << "ERROR: " << __FILE__ << ":" << __LINE__ << ": " << __FUNCTION__ << ": " << CONCLOG_CURRENT_TESTING_CLASS << ": Static assertion `" << #__VA_ARGS__ << "' failed." << std::endl; \
        }                                                               \
    }                                                                   \

/*! \brief Evaluates \a concept in a boolean context and checks if the result is \a true. */
/*! Use variadic macro argument to allow template parameters */
#define CONCLOG_TEST_CONCEPT(...)                          \
    {                                                                   \
        std::cout << #__VA_ARGS__ << ": " << std::flush;                 \
        bool result = ((__VA_ARGS__));                                   \
        if(result) {                                                    \
            std::cout << "true\n" << std::endl;                         \
        } else {                                                        \
            ++CONCLOG_TEST_FAILURES;                                    \
            std::cout << "false\n" << std::endl;                 \
            std::cout << "ERROR: " << __FILE__ << ":" << __LINE__ << ": " << __FUNCTION__ << ": " << CONCLOG_CURRENT_TESTING_CLASS << ": Concept `" << #__VA_ARGS__ << "' failed." << "\n" << std::endl; \
            std::cerr << "ERROR: " << __FILE__ << ":" << __LINE__ << ": " << __FUNCTION__ << ": " << CONCLOG_CURRENT_TESTING_CLASS << ": Concept `" << #__VA_ARGS__ << "' failed." << std::endl; \
        }                                                               \
    }                                                                   \



/*! \brief Evaluates \a expression in a boolean context and checks if the result is \a true. */
/*! Use variadic macro argument to allow template parameters */
#define CONCLOG_TEST_SAME_TYPE(...)                          \
    {                                                                   \
        std::cout << "Same<" << #__VA_ARGS__ << ">: " << std::flush;                 \
        bool result = ((Same<__VA_ARGS__>));                                   \
        if(result) {                                                    \
            std::cout << "true\n" << std::endl;                         \
        } else {                                                        \
            ++CONCLOG_TEST_FAILURES;                                    \
            std::cout << "false\n" << std::endl;                 \
            std::cout << "ERROR: " << __FILE__ << ":" << __LINE__ << ": " << __FUNCTION__ << ": " << CONCLOG_CURRENT_TESTING_CLASS << ": Static assertion `Same<" << #__VA_ARGS__ << ">' failed." << " First type is " << class_name<typename First<__VA_ARGS__>::Type>() << "\n" << std::endl; \
            std::cerr << "ERROR: " << __FILE__ << ":" << __LINE__ << ": " << __FUNCTION__ << ": " << CONCLOG_CURRENT_TESTING_CLASS << ": Static assertion `Same<" << #__VA_ARGS__ << ">' failed." << " First type is " << class_name<typename First<__VA_ARGS__>::Type>() << std::endl; \
        }                                                               \
    }                                                                   \


/*! \brief Tests if two types are equivalent. */
/*! Use variadic macro argument to allow template parameters */
#define CONCLOG_TEST_EQUIVALENT_TYPE(...)                          \
    {                                                                   \
        std::cout << "IsEquivalent<" << #__VA_ARGS__ << ">: " << std::flush;                 \
        bool result = ((IsEquivalent<__VA_ARGS__>::value));                                   \
        if(result) {                                                    \
            std::cout << "true\n" << std::endl;                         \
        } else {                                                        \
            ++CONCLOG_TEST_FAILURES;                                    \
            std::cout << "false\n" << std::endl;                 \
            std::cout << "ERROR: " << __FILE__ << ":" << __LINE__ << ": " << __FUNCTION__ << ": " << CONCLOG_CURRENT_TESTING_CLASS << ": Static assertion `IsEquivalent<" << #__VA_ARGS__ << ">' failed." << " First type is " << class_name<typename First<__VA_ARGS__>::Type>() << "\n" << std::endl; \
            std::cerr << "ERROR: " << __FILE__ << ":" << __LINE__ << ": " << __FUNCTION__ << ": " << CONCLOG_CURRENT_TESTING_CLASS << ": Static assertion `IsEquivalent<" << #__VA_ARGS__ << ">' failed." << " First type is " << class_name<typename First<__VA_ARGS__>::Type>() << std::endl; \
        }                                                               \
    }                                                                   \


/*! \brief Check the Iterator of the GridTreeSubpaving by iterating through all it's values and
 * comparing them with the valus in the vector \a expected_result, the total number of iterated
 * elements should coincide with the value of \a expected_number_elements
 */
#define CONCLOG_TEST_GRID_TREE_SUBPAVING_ITERATOR( expected_result, theGridTreeSubpaving, expected_number_elements ) \
    {                                                                   \
        SizeType elements_count = 0;                                         \
        for (GridTreeSubpaving::ConstIterator it = theGridTreeSubpaving.begin(), end = theGridTreeSubpaving.end(); it != end; it++, elements_count++) { \
            if( elements_count < expected_number_elements ) {           \
                CONCLOG_PRINT_TEST_COMMENT("The next Iterator node is: "); \
                CONCLOG_TEST_COMPARE( (*expected_result[elements_count]), == , (*it) ); \
            }                                                           \
        }                                                               \
        CONCLOG_PRINT_TEST_COMMENT("Test that we iterated through the right number of nodes"); \
        CONCLOG_TEST_EQUAL( elements_count , expected_number_elements ); \
    }                                                                   \


/*! \brief clean std::vector, i.e. delete memory of it's non NULL elements and set them to NULL in the vector */
#define CONCLOG_CLEAN_TEST_VECTOR( vector ) \
    { \
        for(SizeType i = 0; i < vector.size(); i++ ) { \
            if( vector[i] != NULL ) { \
                delete vector[i]; vector[i] = NULL; \
            } \
        } \
    } \


#endif // CONCLOG_TEST_HPP

