// The counted adapters only measure which public metadata methods are called.
// Their cost and representation do not model a production BigInt or scalar node.
#[derive(Default)]
struct Counters(std::cell::Cell<crate::Calls>);

impl Counters {
    fn update(&self, f: impl FnOnce(&mut crate::Calls)) {
        let mut calls = self.0.get();
        f(&mut calls);
        self.0.set(calls);
    }
}

struct BigInt<'a> {
    value: &'a ActualBigInt,
    calls: &'a Counters,
}

struct Magnitude<'a> {
    value: &'a ActualMagnitude,
    calls: &'a Counters,
}

impl BigInt<'_> {
    fn magnitude(&self) -> Magnitude<'_> {
        self.calls.update(|calls| calls.magnitude += 1);
        Magnitude {
            value: self.value.magnitude(),
            calls: self.calls,
        }
    }

    fn sign(&self) -> Sign {
        self.calls.update(|calls| calls.sign += 1);
        self.value.sign()
    }
}

impl Magnitude<'_> {
    fn bits(&self) -> u64 {
        self.calls.update(|calls| calls.bits += 1);
        self.value.bits()
    }

    fn trailing_ones(&self) -> u64 {
        self.calls.update(|calls| calls.ones += 1);
        self.value.trailing_ones()
    }

    fn trailing_zeros(&self) -> Option<u64> {
        self.calls.update(|calls| calls.zeros += 1);
        self.value.trailing_zeros()
    }
}
