                    #[cfg(feature = "svg")]
                    if let Some(path) = std::env::var_os("HYPERCURVE_CAPTURE_CIRCLE_ENDPOINT") {
                        static CAPTURE: std::sync::Once = std::sync::Once::new();
                        CAPTURE.call_once(|| {
                            let mut values = Vec::<(String, Real)>::new();
                            for (name, point) in [
                                ("chord_start", self.start().coordinates().expect("represented start")),
                                ("chord_end", self.end().coordinates().expect("represented end")),
                                ("contact", point.coordinates().expect("represented contact")),
                                ("arc_start", arc.start()),
                                ("arc_end", arc.end()),
                                ("arc_center", arc.center()),
                            ] {
                                values.push((format!("{name}_x"), point.x().clone()));
                                values.push((format!("{name}_y"), point.y().clone()));
                            }
                            values.push(("arc_radius_squared".into(), arc.radius_squared_ref().clone()));
                            for (index, control) in source.homogeneous_controls().iter().enumerate() {
                                values.push((format!("control_{index}_x"), control.x.clone()));
                                values.push((format!("control_{index}_y"), control.y.clone()));
                                values.push((format!("control_{index}_w"), control.weight.clone()));
                            }
                            let endpoint = self.end().coordinates().unwrap();
                            let contact = point.coordinates().unwrap();
                            values.push(("difference_x".into(), contact.x() - endpoint.x()));
                            values.push(("difference_y".into(), contact.y() - endpoint.y()));
                            let mut output = String::new();
                            for (name, value) in values {
                                let serialized = value.to_json();
                                eprintln!("CAPTURE_SCALAR name={name} bytes={}", serialized.len());
                                assert!(serialized.len() < 2_000_000, "scalar capture exceeded its bound");
                                output.push_str(&format!("{{\"name\":{name:?},\"value\":{serialized}}}\n"));
                            }
                            std::fs::write(path, output).expect("write owned exact scalar capture");
                            eprintln!("CAPTURE_CIRCLE_ENDPOINT clockwise={}", arc.is_clockwise());
                        });
                    }
