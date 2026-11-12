use alumina_interface_core::{BoundedMeasurement, Millimetres};
use interval_measurement_conformance::sample;

fn main() {
    let index = std::env::args().nth(1).unwrap().parse::<usize>().unwrap();
    let (nominal, uncertainty) = sample(index);
    match BoundedMeasurement::<Millimetres>::from_nominal_uncertainty(nominal, uncertainty) {
        Ok(result) => println!("{} {} mm", result.lower(), result.upper()),
        Err(error) => println!("{error}"),
    }
}
