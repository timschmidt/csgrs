use vstd::prelude::*;

verus! {
fn original_sign_selection<T>(retained: Option<T>, incoming: Option<T>) -> (result: Option<T>)
    ensures result == match retained { Some(_) => retained, None => incoming },
{
    retained.or(incoming)
}

fn original_magnitude_selection(retained: i32, incoming: i32) -> (result: i32)
    ensures result == if retained < incoming { retained } else { incoming },
{
    retained.min(incoming)
}
}

fn main() {}
