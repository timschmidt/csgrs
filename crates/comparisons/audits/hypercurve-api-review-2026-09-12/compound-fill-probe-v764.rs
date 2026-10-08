use hypercurve::{import_svg_document, Classification, CurveContext, Point2, RegionPointLocation};
fn main() {
    let mut wrong = 0;
    let mut rejected = 0;
    let mut controls = 0;
    for (layout, outer, same, opposite, query) in [
        ("nested", "M0 0 H10 V10 H0 Z", "M2 2 H8 V8 H2 Z", "M2 2 V8 H8 V2 Z", (5,5)),
        ("overlap", "M0 0 H4 V4 H0 Z", "M2 0 H6 V4 H2 Z", "M2 0 V4 H6 V0 Z", (3,2)),
    ] {
        for (orientation, second) in [("same",same),("opposite",opposite)] {
            for fill in ["nonzero","evenodd"] {
                let expected_inside=fill=="nonzero" && orientation=="same";
                let document=format!(r#"<svg xmlns="http://www.w3.org/2000/svg"><path fill-rule="{fill}" d="{outer} {second}"/></svg>"#);
                match import_svg_document(&document) {
                    Err(_) => { rejected+=1;println!("layout={layout} orientation={orientation} fill={fill} rejected=true"); }
                    Ok(geometry) => {
                        let result=geometry.region().classify_point(&Point2::from_values(query.0,query.1).into(),&CurveContext::STRICT).unwrap().into_value();
                        let expected=if expected_inside {RegionPointLocation::Inside} else {RegionPointLocation::Outside};
                        let correct=matches!(result,Classification::Decided(location) if location==expected);
                        if correct {controls+=1;} else {wrong+=1;}
                        println!("layout={layout} orientation={orientation} fill={fill} correct={correct} expected_inside={expected_inside}");
                    }
                }
            }
        }
    }
    println!("wrong={wrong} rejected={rejected} controls={controls}");
    assert_eq!((wrong,rejected,controls),(1,4,3));
}
