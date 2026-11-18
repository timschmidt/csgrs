extern crate std;
use super::array_result;
use std::{cell::RefCell, rc::Rc, vec::Vec};

struct Owned {
    index: usize,
    events: Rc<RefCell<Vec<(bool, usize)>>>,
}

impl Drop for Owned {
    fn drop(&mut self) {
        self.events.borrow_mut().push((false, self.index));
    }
}

#[test]
fn construction_stops_at_first_error_and_drops_owned_prefix_in_order() {
    for failure in 0..=4 {
        let events = Rc::new(RefCell::new(Vec::new()));
        let result: Result<[Owned; 4], usize> = array_result(|index| {
            events.borrow_mut().push((true, index));
            if index == failure {
                Err(index)
            } else {
                Ok(Owned {
                    index,
                    events: Rc::clone(&events),
                })
            }
        });
        let attempted = if failure < 4 { failure + 1 } else { 4 };
        if failure < 4 {
            assert!(matches!(result, Err(index) if index == failure));
        } else {
            let values = result.unwrap_or_else(|_| panic!("all entries succeed"));
            assert_eq!(values.each_ref().map(|value| value.index), [0, 1, 2, 3]);
            assert_eq!(events.borrow().len(), 4, "no premature destruction");
            drop(values);
        }
        let expected = (0..attempted)
            .map(|index| (true, index))
            .chain((0..failure).map(|index| (false, index)))
            .collect::<Vec<_>>();
        assert_eq!(*events.borrow(), expected);
    }
    let unit: Result<[Owned; 0], usize> = array_result(|_| panic!("unit array has no entries"));
    assert!(matches!(unit, Ok([])));
}

#[test]
fn nested_failure_drops_partial_row_before_completed_rows() {
    let events = Rc::new(RefCell::new(Vec::new()));
    let result: Result<[[Owned; 3]; 2], usize> = array_result(|row| {
        array_result(|column| {
            let index = 3 * row + column;
            events.borrow_mut().push((true, index));
            if index == 4 {
                Err(index)
            } else {
                Ok(Owned {
                    index,
                    events: Rc::clone(&events),
                })
            }
        })
    });
    assert!(matches!(result, Err(4)));
    assert_eq!(
        *events.borrow(),
        [
            (true, 0),
            (true, 1),
            (true, 2),
            (true, 3),
            (true, 4),
            (false, 3),
            (false, 0),
            (false, 1),
            (false, 2)
        ]
    );
}
