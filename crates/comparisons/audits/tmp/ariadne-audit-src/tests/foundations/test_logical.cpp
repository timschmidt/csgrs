/***************************************************************************
 *            test_logical.cpp
 *
 *  Copyright  2015-20  Pieter Collins
 *
 ****************************************************************************/

/*
 *  This file is part of Ariadne.
 *
 *  Ariadne is free software: you can redistribute it and/or modify
 *  it under the terms of the GNU General Public License as published by
 *  the Free Software Foundation, either version 3 of the License, or
 *  (at your option) any later version.
 *
 *  Ariadne is distributed in the hope that it will be useful,
 *  but WITHOUT ANY WARRANTY; without even the implied warranty of
 *  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 *  GNU General Public License for more details.
 *
 *  You should have received a copy of the GNU General Public License
 *  along with Ariadne.  If not, see <https://www.gnu.org/licenses/>.
 */

#include "utility/metaprogramming.hpp"
#include "foundations/paradigm.hpp"
#include "foundations/logical.hpp"
#include "numeric/integer.hpp"
#include "numeric/sequence.hpp"

#include "../test.hpp"

using namespace Ariadne;


class TestParadigm
{
  public:
    Void test();
  private:
    Void test_concept();
};

class TestLogical
{
  public:
    Void test();
  private:
    Void test_concept();
    Void test_conversion_to_bool();
    Void test_conversion();
    Void test_disjunction();
};


Int main() {
    std::cout<<std::setprecision(20);
    std::cerr<<std::setprecision(20);

    ARIADNE_TEST_CLASS(TestParadigm,TestParadigm());
    ARIADNE_TEST_CLASS(TestLogical,TestLogical());

    return ARIADNE_TEST_FAILURES;
}


Void
TestParadigm::test()
{
}

// Test that the type implements all operations of
// the FloatDP concept without testing correctness
Void
TestParadigm::test_concept()
{
    ARIADNE_TEST_CONCEPT(WeakerThan<ApproximateTag,ApproximateTag>);
    ARIADNE_TEST_CONCEPT(WeakerThan<ApproximateTag,ValidatedTag>);
    ARIADNE_TEST_CONCEPT(WeakerThan<ApproximateTag,EffectiveTag>);
    ARIADNE_TEST_CONCEPT(WeakerThan<ApproximateTag,ExactTag>);
    ARIADNE_TEST_CONCEPT(WeakerThan<ValidatedTag,ValidatedTag>);
    ARIADNE_TEST_CONCEPT(WeakerThan<ValidatedTag,EffectiveTag>);
    ARIADNE_TEST_CONCEPT(WeakerThan<ValidatedTag,ExactTag>);
    ARIADNE_TEST_CONCEPT(WeakerThan<EffectiveTag,EffectiveTag>);
    ARIADNE_TEST_CONCEPT(WeakerThan<EffectiveTag,ExactTag>);
    ARIADNE_TEST_CONCEPT(WeakerThan<ExactTag,ExactTag>);

    ARIADNE_TEST_CONCEPT(not WeakerThan<ValidatedTag,ApproximateTag>);
    ARIADNE_TEST_CONCEPT(not WeakerThan<EffectiveTag,ApproximateTag>);
    ARIADNE_TEST_CONCEPT(not WeakerThan<EffectiveTag,ValidatedTag>);
    ARIADNE_TEST_CONCEPT(not WeakerThan<ExactTag,ApproximateTag>);
    ARIADNE_TEST_CONCEPT(not WeakerThan<ExactTag,ValidatedTag>);
    ARIADNE_TEST_CONCEPT(not WeakerThan<ExactTag,EffectiveTag>);
}

Void
TestLogical::test()
{
    ARIADNE_TEST_CALL(test_conversion_to_bool());
    ARIADNE_TEST_CALL(test_conversion());
    ARIADNE_TEST_CALL(test_disjunction());
}

Void
TestLogical::test_concept()
{
    // Check to see if we can perform operations on computational and specification logical types
    LogicalType<ExactTag> xl(true);
    LogicalType<EffectiveTag> el(true);
    LogicalType<ValidatedTag> vl(true);
    Effort eff(0);

    vl=el.check(eff);
    vl=check(el,eff);

    vl=indeterminate;
    vl=LogicalType<ValidatedTag>(LogicalValue::LIKELY);

    xl = xl && xl;
    el = xl && el;
    vl = xl && vl;
    el = el && el;
    vl = vl && vl;
}

Void
TestLogical::test_conversion_to_bool()
{
    ARIADNE_TEST_CONCEPT(Convertible<Boolean,Bool>);
    ARIADNE_TEST_CONCEPT(not Convertible<Sierpinskian,Bool>);
    ARIADNE_TEST_CONCEPT(not Convertible<NegatedSierpinskian,Bool>);
    ARIADNE_TEST_CONCEPT(not Convertible<Kleenean,Bool>);
    ARIADNE_TEST_CONCEPT(not Convertible<LowerKleenean,Bool>);
    ARIADNE_TEST_CONCEPT(not Convertible<UpperKleenean,Bool>);
    ARIADNE_TEST_CONCEPT(not Convertible<ValidatedKleenean,Bool>);
    ARIADNE_TEST_CONCEPT(not Convertible<ValidatedUpperKleenean,Bool>);
    ARIADNE_TEST_CONCEPT(not Convertible<ValidatedLowerKleenean,Bool>);
    ARIADNE_TEST_CONCEPT(not Convertible<ApproximateKleenean,Bool>);
}

Void
TestLogical::test_conversion()
{
    if(Convertible<LogicalType<EffectiveTag>,LogicalType<ValidatedTag>>) {
        ARIADNE_TEST_NOTIFY("EffectiveTag logical types may be converted to values using default Effort.");
    } else if(Convertible<LogicalType<EffectiveTag>,LogicalType<ValidatedTag>>) {
        ARIADNE_TEST_NOTIFY("EffectiveTag logical types may be explicitly converted to values using default Effort.");
    } else {
        ARIADNE_TEST_NOTIFY("EffectiveTag logical types cannot be converted to values; the Effort used must be specified.");
    }

    try {
        if(decide(indeterminate)) {
            ARIADNE_TEST_NOTIFY("decide(...) is true on INDETERMINATE value.");
        } else {
            ARIADNE_TEST_NOTIFY("decide(...) is false on INDETERMINATE value.");
        }
    } catch(...) {
        ARIADNE_TEST_NOTIFY("decide(...) is throws error on INDETERMINATE value.");
    }

    ARIADNE_TEST_CONCEPT(not Convertible<Indeterminate,Boolean>);
    ARIADNE_TEST_CONCEPT(Convertible<Indeterminate,Sierpinskian>);
    ARIADNE_TEST_CONCEPT(Convertible<Indeterminate,NegatedSierpinskian>);
    ARIADNE_TEST_CONCEPT(Convertible<Indeterminate,Kleenean>);
    ARIADNE_TEST_CONCEPT(Convertible<Indeterminate,LowerKleenean>);
    ARIADNE_TEST_CONCEPT(Convertible<Indeterminate,UpperKleenean>);
//    ARIADNE_TEST_CONCEPT(Same<decltype(indeterminate and true),Kleenean>);
//    ARIADNE_TEST_CONCEPT(Same<decltype(indeterminate and Boolean(true)),Kleenean>);
    ARIADNE_TEST_CONCEPT(Same<decltype(indeterminate and Sierpinskian(true)),Sierpinskian>);
    ARIADNE_TEST_CONCEPT(Same<decltype(indeterminate and Kleenean(true)),Kleenean>);

    ARIADNE_TEST_CONSTRUCT(LogicalType<ValidatedTag>,vl,(LogicalValue::LIKELY))
    ARIADNE_TEST_EQUAL(definitely(vl),false);
    ARIADNE_TEST_EQUAL(possibly(vl),true);
    ARIADNE_TEST_EQUAL(decide(vl),true);

    ARIADNE_TEST_CONSTRUCT(LogicalType<ValidatedTag>,vi,(LogicalValue::INDETERMINATE))
    ARIADNE_TEST_EQUAL(definitely(vl),false);
    ARIADNE_TEST_EQUAL(possibly(vl),true);
}

Void
TestLogical::test_disjunction()
{
    Sequence<LowerKleenean> seq([](Natural n){return n==2 ? LowerKleenean(true) : LowerKleenean(indeterminate);});
    ARIADNE_TEST_ASSIGN_CONSTRUCT(LowerKleenean, some, disjunction(seq));
    ARIADNE_TEST_ASSERT(possibly(not some.check(2_eff)));
    ARIADNE_TEST_ASSERT(definitely(some.check(3_eff)));
    ARIADNE_TEST_ASSERT(definitely(some.check(4_eff)));

    ARIADNE_TEST_ASSIGN_CONSTRUCT(
        UpperKleenean, all, conjunction(Sequence<UpperKleenean>([](Natural n){return n==2 ? UpperKleenean(false) : UpperKleenean(indeterminate);})));
    ARIADNE_TEST_ASSERT(possibly(all.check(2_eff)));
    ARIADNE_TEST_ASSERT(not possibly(all.check(3_eff)));
    ARIADNE_TEST_ASSERT(definitely(not all.check(4_eff)));

}

