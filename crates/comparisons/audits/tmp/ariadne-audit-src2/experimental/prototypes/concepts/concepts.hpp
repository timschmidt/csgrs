namespace Ariadne {

template<class X> class ConcreteConcept {
  public:
    typedef typename X::GenericType Y;
    typedef typename X::PropertiesType PR;

    void usage() {
        { PR pr = x.properties(); }
        { Y y = x.generic(); }
        { x.operator Y(); }
        { X x(pr); }
        { X x(y,pr); }
        { X x=create(y,pr); }
        { x_nc=y; }
    }
  private:
    X x_nc;
    X const x;
    Y const y;
    PR const pr;
};

class ConcreteArchetype {
  private:
    typedef ConcreteArchetype X;
    struct Y { };
    struct PR { };
    friend X create(Y const&, PR);
  public:
    typedef Y GenericType;
    typedef PR PropertiesType;
    PR properties() const;
    Y generic() const;
    operator Y() const;
    ConcreteArchetype(PR);
    ConcreteArchetype(Y const&, PR);
    X& operator=(Y const&);
};
ConcreteArchetype::X create(ConcreteArchetype::Y const&, ConcreteArchetype::PR);

template<class R> class FieldConcept {
  private:
    R r;
  public:
    void usage() {
        R rr(r);
        r=neg(r);
        r=add(r,r);
        r=sub(r,r);
        r=mul(r,r);
        r=div(r,r);
        r=rec(r);
    }
};

class Rational;

template<class R> class RealConcept
    : public FieldConcept<R>
{
  private:
    Rational* q;

    void usage() {
        R r(*q);
    }
};

template<class A> class AlgebraConcept
    : FieldConcept<A>
{
  public:
    typedef typename A::NumericType X;

    void usage() {
        a_nc = x;
        a = add(a,x);
        a = sub(a,x);
        a = mul(a,x);
        a = div(a,x);
        a = add(x,a);
        a = sub(x,a);
        a = mul(x,a);
        a = div(x,a);
    }
  private:
    A a_nc;
    A const a;
    X const x;
};

class RealArchetype {
  private:
    typedef RealArchetype Self;
  public:
    RealArchetype(Self const&);
    RealArchetype(Rational const&);
    friend Self neg(Self const&);
    friend Self add(Self const&, Self const&);
    friend Self sub(Self const&, Self const&);
    friend Self mul(Self const&, Self const&);
    friend Self div(Self const&, Self const&);
    friend Self rec(Self const&);
};

} // namespace Ariadne
