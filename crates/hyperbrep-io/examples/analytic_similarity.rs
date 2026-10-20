//! Exact native scaling followed by STEP output in millimetres and inches.
use hyperbrep::{Matrix4, OperationContext, Real, WorkBudget, builder};
use hyperbrep_io::step::{StepBody, StepWriteOptions, WriteLengthUnit, write_advanced};

fn main() -> Result<(), Box<dyn std::error::Error>> {
    let directory = std::env::args_os()
        .nth(1)
        .map(std::path::PathBuf::from)
        .unwrap_or_else(|| std::env::temp_dir().join("hyperbrep-scaled-step"));
    std::fs::create_dir_all(&directory)?;
    let source = builder::cylinder(5.into(), 7.into())?;
    let context = OperationContext::strict(WorkBudget {
        refinements: 10_000,
        promotions: 1000,
    });
    for (name, scale) in [
        ("mm", (Real::one() / Real::from(1000))?),
        ("inch", (Real::from(127) / Real::from(5000))?),
    ] {
        for reflected in [false, true] {
            let model = source.transformed(&Matrix4::affine_nonuniform_scale([
                if reflected {
                    -scale.clone()
                } else {
                    scale.clone()
                },
                scale.clone(),
                scale.clone(),
            ]))?;
            let file_name = format!(
                "scaled-{name}-{}.step",
                if reflected { "reflected" } else { "positive" }
            );
            let mut options = StepWriteOptions::new(&file_name, "2026-10-16T00:00:00");
            options.length_unit = WriteLengthUnit {
                name,
                metres_per_unit: scale.clone(),
            };
            let output = write_advanced(
                &[StepBody {
                    solid: model.bodies().next().unwrap(),
                    name: "scaled cylinder",
                }],
                &options,
                &context,
            )
            .result
            .map_err(|e| format!("solver budget: {e:?}"))??;
            std::fs::write(directory.join(file_name), output.text())?;
        }
    }
    println!(
        "Wrote four exact scaled-cylinder STEP files to {}",
        directory.display()
    );
    Ok(())
}
