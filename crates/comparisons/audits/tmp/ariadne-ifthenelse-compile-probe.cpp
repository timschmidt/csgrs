#include "config.hpp"
#include "symbolic/templates.hpp"

struct Condition {
    bool operator()(int value) const { return value > 0; }
};

struct Identity {
    int operator()(int value) const { return value; }
};

int main() {
    Ariadne::Symbolic<Ariadne::IfThnEls, Condition, Identity> expression{
        Ariadne::IfThnEls(), Condition(), Identity(), Identity()};
    return expression(1);
}
