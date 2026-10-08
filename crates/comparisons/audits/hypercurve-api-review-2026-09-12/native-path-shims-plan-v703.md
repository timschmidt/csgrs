# Isolated native-path forwarder removal candidate

Five copied files, unformatted/unbuilt/unpromoted. Remove CurveString2::extend_line_endpoint_to_point, which already delegates to the general segment operation, and the borrowed ordered-link alias. The remaining link operation consumes an IntoIterator<Item=Self>, so owned vectors and explicitly cloned borrowed iterators share one interface without an eager temporary Vec. The linking kernel itself is unchanged.

Remove CurveString2::trim_inside_region, which reconstructs a CurvePath2 on every query. Its three mathematical integration cases now author CurvePath2 directly; the editing benchmark retains the general path outside timing and labels its workload curve_path_region_trim; the fuzzer uses its existing path for clipping. Hyperbrep parameter_lines_face_trim_intervals now constructs CurvePath2 directly, removing its sole production CurveString2 dependency. No compatibility alias/conversion method is added. These constructors produce exactly the same Curve2 values that the deleted shim produced.

Expected qualification: existing complete curve-string integration inventory (three region test names migrate, none deleted), targeted general path/region clipping guards, and Hyperbrep plane/extrusion and axial-cone-ray trim callers; all-target checks for both changed crates, formatting, docs and affected fuzz target. Native path representation has other distinct capabilities and is not wholesale hidden by this bounded migration. The generic overlap-orientation rename remains deferred (V698).

Do not promote, format or build before V701 is exactly reaped and committed/sealed. V701 production/mirror/archive/driver remain untouched. Next unused artifactV704.
