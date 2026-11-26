//! Exact regularized-set identities through ordinary transformed box workflows.
//!
//! See `docs/workflow-corpus.md` for the finite domain and equality criterion.
//! The independent oracle checks each boundary-free open grid cell with exact
//! rational winding; native PWN validation is a separate topology requirement.

/// Shared exact fixtures, inverse charts, independent oracle and workflow assertions.
#[path = "support/strict_boolean_workflows/mod.rs"]
mod corpus;

/// Instantiates nine box workflows and both oracle controls for one affine frame.
macro_rules! frame_cases {
    ($module:ident, $frame:literal) => {
        /// Runs the same finite set contract in this named affine coordinate frame.
        mod $module {
            /// Binds a named box relation to the enclosing frame and 21-output workflow.
            macro_rules! fixture_case {
                ($name:ident, $fixture:literal) => {
                    /// Requires certified completion and independent set checks for this pair.
                    #[test]
                    fn $name() {
                        crate::corpus::assert_workflow($fixture, $frame);
                    }
                };
            }
            fixture_case!(overlap, "overlap");
            fixture_case!(coplanar, "coplanar");
            fixture_case!(face_touch, "face-touch");
            fixture_case!(edge_touch, "edge-touch");
            fixture_case!(point_touch, "point-touch");
            fixture_case!(disconnected, "disconnected");
            fixture_case!(coincident, "coincident");
            fixture_case!(nested, "nested");
            fixture_case!(near_coplanar, "near-coplanar");

            /// Rejects an equal-volume wrong set and forces a nondegenerate-ray retry.
            #[test]
            fn oracle_rejects_wrong_sets_and_retries_degenerate_rays() {
                crate::corpus::assert_oracle_controls($frame);
            }
        }
    };
}

frame_cases!(identity, "identity");
frame_cases!(shear, "shear");
frame_cases!(reflection, "reflection");
frame_cases!(root_rotation, "root-rotation");
frame_cases!(symbolic_translation, "symbolic-translation");
