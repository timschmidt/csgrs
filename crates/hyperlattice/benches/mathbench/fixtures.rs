/// Three binary64 coordinates shared by benchmark comparison adapters.
///
/// These are already rounded machine values, including decimal literals and standard
/// floating constants. They are not exact authored decimal, pi or e expressions.
/// Adapters choose whether to retain binary values (s), parse displayed decimals (qr),
/// or use another caller-supplied conversion. No geometric or norm validation occurs.
#[derive(Clone, Copy, Debug)]
struct SampleVec3 {
    /// First binary64 coordinate, converted before y and z.
    x: f64,
    /// Second binary64 coordinate.
    y: f64,
    /// Third binary64 coordinate.
    z: f64,
}

/// Four binary64 lanes for Euclidean and homogeneous benchmark inputs.
///
/// The w lane is unrestricted: these fixtures are not uniformly points (w=1) or
/// directions (w=0). Conversion and mathematical interpretation belong to each row.
#[derive(Clone, Copy, Debug)]
struct SampleVec4 {
    /// First binary64 coordinate.
    x: f64,
    /// Second binary64 coordinate.
    y: f64,
    /// Third binary64 coordinate.
    z: f64,
    /// Fourth binary64 lane; unrestricted unless a consuming workload assumes a weight.
    w: f64,
}

/// Row-major binary64 3x3 matrix fixture, before comparison-engine conversion.
///
/// Stores coefficients without checking invertibility, affine form or conditioning;
/// floating expressions in fixture constructors are evaluated before conversion.
#[derive(Clone, Copy, Debug)]
struct SampleMat3 {
    /// Nine binary64 entries indexed by row then column, without validity metadata.
    m: [[f64; 3]; 3],
}

/// Row-major binary64 4x4 matrix fixture for dense, sparse and affine workloads.
///
/// No rank or transform convention is certified by this carrier. Its bottom row can
/// be general; only specifically affine fixture families promise the authored pattern.
#[derive(Clone, Copy, Debug)]
struct SampleMat4 {
    /// Sixteen binary64 entries indexed by row then column, without validity metadata.
    m: [[f64; 4]; 4],
}

/// Return the fixed moderate-scale mixed-sign decimal-literal 3-vector.
///
/// The long decimal spellings are rounded to binary64 here; direct rational counterparts
/// are constructed separately by blas_vec3_rational for decimal-authored comparisons.
fn sample_vec3() -> SampleVec3 {
    SampleVec3 {
        x: 1.23456789012345,
        y: -2.34567890123456,
        z: 3.45678901234567,
    }
}

/// Return four left 3-vector cases in fixed order: base, tiny, cancellation-scale, constants.
///
/// Includes coordinates near 1e-9, the (1e9,1,-1e9) scale mixture, and binary64 pi/-e
/// with a 1e-12 lane. This finite corpus is not random or exhaustive; constants are
/// floating approximations before any Real adapter is selected.
fn sample_vec3_cases() -> [SampleVec3; 4] {
    [
        sample_vec3(),
        SampleVec3 {
            x: 1.0e-9,
            y: -2.0e-9,
            z: 3.0e-9,
        },
        SampleVec3 {
            x: 1.0e9,
            y: 1.0,
            z: -1.0e9,
        },
        SampleVec3 {
            x: std::f64::consts::PI,
            y: -std::f64::consts::E,
            z: 1.0e-12,
        },
    ]
}

/// Return the distinct moderate-scale mixed-sign right 3-vector of rounded decimal literals.
fn sample_vec3_b() -> SampleVec3 {
    SampleVec3 {
        x: -0.98765432101234,
        y: 4.21098765432109,
        z: -5.67890123456789,
    }
}

/// Return four right 3-vectors aligned by index with sample_vec3_cases.
///
/// Uses the right base fixture, small signed coordinates, (-1e9,2,1e9), and floating
/// -1/pi, sqrt(2), -1e-12. Index pairing selects workloads, not orthogonality or an
/// independent expected-result oracle.
fn sample_vec3_b_cases() -> [SampleVec3; 4] {
    [
        sample_vec3_b(),
        SampleVec3 {
            x: -3.0e-9,
            y: 5.0e-9,
            z: -7.0e-9,
        },
        SampleVec3 {
            x: -1.0e9,
            y: 2.0,
            z: 1.0e9,
        },
        SampleVec3 {
            x: -std::f64::consts::FRAC_1_PI,
            y: std::f64::consts::SQRT_2,
            z: -1.0e-12,
        },
    ]
}

/// Return four sparse left 3-vectors with zero, one and two active coordinates.
///
/// Cases are zero, (3,0,0), (0,-4,7) and (1,2,0), exposing empty/short dot reductions.
/// They include a zero vector and are not all valid normalization inputs.
fn sample_vec3_sparse_cases() -> [SampleVec3; 4] {
    [
        SampleVec3 {
            x: 0.0,
            y: 0.0,
            z: 0.0,
        },
        SampleVec3 {
            x: 3.0,
            y: 0.0,
            z: 0.0,
        },
        SampleVec3 {
            x: 0.0,
            y: -4.0,
            z: 7.0,
        },
        SampleVec3 {
            x: 1.0,
            y: 2.0,
            z: 0.0,
        },
    ]
}

/// Return right sparse partners: zero, (-8,0,0), (0,9,-11) and (0,5,0).
///
/// The last pair has only one shared active lane; the others exercise empty, one-term
/// and two-term products. The fixture itself does not assert a reduction schedule.
fn sample_vec3_sparse_b_cases() -> [SampleVec3; 4] {
    [
        SampleVec3 {
            x: 0.0,
            y: 0.0,
            z: 0.0,
        },
        SampleVec3 {
            x: -8.0,
            y: 0.0,
            z: 0.0,
        },
        SampleVec3 {
            x: 0.0,
            y: 9.0,
            z: -11.0,
        },
        SampleVec3 {
            x: 0.0,
            y: 5.0,
            z: 0.0,
        },
    ]
}

/// Return (3,4,5,1), exactly representable binary64 lanes including unit homogeneous weight.
fn sample_vec4() -> SampleVec4 {
    SampleVec4 {
        x: 3.0,
        y: 4.0,
        z: 5.0,
        w: 1.0,
    }
}

/// Return four left 4-vectors: base, tiny, large cancellation-scale and floating constants.
///
/// Weights are 1, -4e-9, -1 and 1e-12 respectively. They remain general fourth lanes;
/// only the first is a caller-established homogeneous point.
fn sample_vec4_cases() -> [SampleVec4; 4] {
    [
        sample_vec4(),
        SampleVec4 {
            x: 1.0e-9,
            y: -2.0e-9,
            z: 3.0e-9,
            w: -4.0e-9,
        },
        SampleVec4 {
            x: 1.0e9,
            y: -1.0e9,
            z: 1.0,
            w: -1.0,
        },
        SampleVec4 {
            x: std::f64::consts::PI,
            y: -std::f64::consts::E,
            z: std::f64::consts::SQRT_2,
            w: 1.0e-12,
        },
    ]
}

/// Return zero, one-active, two-active and dense integer 4-vectors.
///
/// The cases are zero, (2,0,0,0), (0,3,0,-4) and (1,2,3,4); the last is deliberately
/// dense despite the family name. Zero remains in-domain for dot work, not normalization.
fn sample_vec4_sparse_cases() -> [SampleVec4; 4] {
    [
        SampleVec4 {
            x: 0.0,
            y: 0.0,
            z: 0.0,
            w: 0.0,
        },
        SampleVec4 {
            x: 2.0,
            y: 0.0,
            z: 0.0,
            w: 0.0,
        },
        SampleVec4 {
            x: 0.0,
            y: 3.0,
            z: 0.0,
            w: -4.0,
        },
        SampleVec4 {
            x: 1.0,
            y: 2.0,
            z: 3.0,
            w: 4.0,
        },
    ]
}

/// Return the aligned right sparse fixtures, including a three-active final vector.
///
/// Uses zero, (2,0,0,0), (0,-3,0,1), (7,0,-8,9). These support-pattern choices exercise
/// different overlap counts without certifying the implementation's chosen kernel.
fn sample_vec4_sparse_b_cases() -> [SampleVec4; 4] {
    [
        SampleVec4 {
            x: 0.0,
            y: 0.0,
            z: 0.0,
            w: 0.0,
        },
        SampleVec4 {
            x: 2.0,
            y: 0.0,
            z: 0.0,
            w: 0.0,
        },
        SampleVec4 {
            x: 0.0,
            y: -3.0,
            z: 0.0,
            w: 1.0,
        },
        SampleVec4 {
            x: 7.0,
            y: 0.0,
            z: -8.0,
            w: 9.0,
        },
    ]
}

/// Return four distinct right 4-vectors paired by index with sample_vec4_cases.
///
/// Includes integer, tiny, large-magnitude and rounded pi-related-constant lanes;
/// weights 4, 1e-9, 2 and 1e-12 are not assumed point/direction weights.
fn sample_vec4_b_cases() -> [SampleVec4; 4] {
    [
        SampleVec4 {
            x: -1.0,
            y: 2.0,
            z: -3.0,
            w: 4.0,
        },
        SampleVec4 {
            x: -4.0e-9,
            y: 3.0e-9,
            z: -2.0e-9,
            w: 1.0e-9,
        },
        SampleVec4 {
            x: -1.0e9,
            y: 1.0e9,
            z: -2.0,
            w: 2.0,
        },
        SampleVec4 {
            x: -std::f64::consts::FRAC_1_PI,
            y: std::f64::consts::FRAC_2_PI,
            z: -std::f64::consts::FRAC_2_SQRT_PI,
            w: 1.0e-12,
        },
    ]
}

/// Return the fixed dense 3x3 decimal-literal left matrix used by general comparisons.
///
/// Non-dyadic decimal spellings are rounded here; blas_mat3_rational separately authors
/// the displayed tenths as exact ratios rather than importing these binary values.
fn sample_mat3() -> SampleMat3 {
    SampleMat3 {
        m: [[1.2, 0.3, -0.7], [2.1, -1.5, 0.9], [0.4, 3.3, 2.2]],
    }
}

/// Return base, perturbed rank-one, mixed-scale and floating-constant left 3x3 matrices.
///
/// The second starts from all ones with two diagonal perturbations near 1e-6; those
/// additions happen in binary64 before conversion. Other cases mix 1e6/1e-6 scales or
/// rounded pi/e/sqrt(2). No determinant or condition-number check runs here.
fn sample_mat3_cases() -> [SampleMat3; 4] {
    [
        sample_mat3(),
        SampleMat3 {
            m: [
                [1.0, 1.0, 1.0],
                [1.0, 1.0 + 1.0e-6, 1.0],
                [1.0, 1.0, 1.0 + 2.0e-6],
            ],
        },
        SampleMat3 {
            m: [[1.0e6, 2.0, -3.0], [4.0, -1.0e-6, 6.0], [-7.0, 8.0, 9.0e3]],
        },
        SampleMat3 {
            m: [
                [std::f64::consts::PI, -std::f64::consts::E, 1.0e-9],
                [std::f64::consts::SQRT_2, 1.0, -2.0],
                [3.0, -5.0, 8.0],
            ],
        },
    ]
}

/// Return the distinct dense 3x3 decimal-literal right matrix for paired comparisons.
///
/// The corresponding authored-tenths matrix is blas_mat3_b_rational, not necessarily
/// the same exact values as direct binary64 import of this fixture.
fn sample_mat3_b() -> SampleMat3 {
    SampleMat3 {
        m: [[-0.8, 1.1, 0.5], [2.7, 0.6, -1.4], [3.2, -0.9, 1.8]],
    }
}

/// Return right 3x3 partners: base, perturbed all-twos, mixed-scale and floating constants.
///
/// Perturbations 3e-6/5e-6 are added in binary64. Fixed rows stress magnitude and
/// near-dependence patterns; the constructor does not measure conditioning or validity.
fn sample_mat3_b_cases() -> [SampleMat3; 4] {
    [
        sample_mat3_b(),
        SampleMat3 {
            m: [
                [2.0, 2.0, 2.0],
                [2.0, 2.0 + 3.0e-6, 2.0],
                [2.0, 2.0, 2.0 + 5.0e-6],
            ],
        },
        SampleMat3 {
            m: [
                [-1.0e5, 3.0, 5.0],
                [7.0, 1.0e-6, -11.0],
                [13.0, -17.0, 1.0e4],
            ],
        },
        SampleMat3 {
            m: [
                [
                    -std::f64::consts::FRAC_1_PI,
                    std::f64::consts::FRAC_2_PI,
                    -1.0e-9,
                ],
                [std::f64::consts::FRAC_2_SQRT_PI, -1.0, 2.0],
                [-3.0, 5.0, -8.0],
            ],
        },
    ]
}

/// Return four left 2D affine homogeneous fixtures in 3x3 storage.
///
/// Each authored bottom row is (0,0,1); the last column supplies translation.
/// The first has identity linear action; later cases combine scaling and off-diagonal
/// linear terms. Coefficients are binary64 values, and no invertibility or condition
/// check is performed. Arbitrary downstream scalar conversion need not preserve form.
fn sample_mat3_affine_cases() -> [SampleMat3; 4] {
    [
        SampleMat3 {
            m: [[1.0, 0.0, 1.2], [0.0, 1.0, -0.8], [0.0, 0.0, 1.0]],
        },
        SampleMat3 {
            m: [[1.2, 0.1, -2.0], [0.3, 0.9, 5.5], [0.0, 0.0, 1.0]],
        },
        SampleMat3 {
            m: [[0.95, 0.2, 10.0], [-0.1, 1.03, -6.0], [0.0, 0.0, 1.0]],
        },
        SampleMat3 {
            m: [[1.4, 0.0, 0.5], [0.0, 1.25, -1.5], [0.0, 0.0, 1.0]],
        },
    ]
}

/// Return four right 2D affine homogeneous fixtures in 3x3 storage.
///
/// Each authored bottom row is (0,0,1); the last column supplies translation.
/// The first has identity linear action; later cases combine scaling and off-diagonal
/// linear terms. Coefficients are binary64 values, and no invertibility or condition
/// check is performed. Arbitrary downstream scalar conversion need not preserve form.
fn sample_mat3_affine_b_cases() -> [SampleMat3; 4] {
    [
        SampleMat3 {
            m: [[1.0, 0.0, -3.0], [0.0, 1.0, 2.5], [0.0, 0.0, 1.0]],
        },
        SampleMat3 {
            m: [[1.1, 0.2, 1.5], [0.0, 0.95, -2.5], [0.0, 0.0, 1.0]],
        },
        SampleMat3 {
            m: [[0.9, -0.1, -7.0], [0.12, 1.15, 4.0], [0.0, 0.0, 1.0]],
        },
        SampleMat3 {
            m: [[1.0, -0.3, 6.5], [0.0, 0.88, 2.5], [0.0, 0.0, 1.0]],
        },
    ]
}

/// Return four left pure 2D translations with identity linear block.
///
/// Keeps the translation columns of sample_mat3_affine_cases while replacing
/// the linear block by identity and retaining bottom row (0,0,1). The authored
/// matrices have determinant one; no measured dispatch or downstream conversion
/// invariance follows from that pattern alone.
fn sample_mat3_affine_translation_cases() -> [SampleMat3; 4] {
    [
        SampleMat3 {
            m: [[1.0, 0.0, 1.2], [0.0, 1.0, -0.8], [0.0, 0.0, 1.0]],
        },
        SampleMat3 {
            m: [[1.0, 0.0, -2.0], [0.0, 1.0, 5.5], [0.0, 0.0, 1.0]],
        },
        SampleMat3 {
            m: [[1.0, 0.0, 10.0], [0.0, 1.0, -6.0], [0.0, 0.0, 1.0]],
        },
        SampleMat3 {
            m: [[1.0, 0.0, 0.5], [0.0, 1.0, -1.5], [0.0, 0.0, 1.0]],
        },
    ]
}

/// Return four right pure 2D translations with identity linear block.
///
/// Keeps the translation columns of sample_mat3_affine_b_cases while replacing
/// the linear block by identity and retaining bottom row (0,0,1). The authored
/// matrices have determinant one; no measured dispatch or downstream conversion
/// invariance follows from that pattern alone.
fn sample_mat3_affine_translation_b_cases() -> [SampleMat3; 4] {
    [
        SampleMat3 {
            m: [[1.0, 0.0, -3.0], [0.0, 1.0, 2.5], [0.0, 0.0, 1.0]],
        },
        SampleMat3 {
            m: [[1.0, 0.0, 1.5], [0.0, 1.0, -2.5], [0.0, 0.0, 1.0]],
        },
        SampleMat3 {
            m: [[1.0, 0.0, -7.0], [0.0, 1.0, 4.0], [0.0, 0.0, 1.0]],
        },
        SampleMat3 {
            m: [[1.0, 0.0, 6.5], [0.0, 1.0, 2.5], [0.0, 0.0, 1.0]],
        },
    ]
}

/// Return the fixed small-integer 4x4 left matrix for general comparisons.
///
/// All entries are exactly representable in binary64; the last row (2,7,1,3) is general,
/// so this fixture does not encode an affine homogeneous transform.
fn sample_mat4() -> SampleMat4 {
    SampleMat4 {
        m: [
            [1.0, 2.0, 3.0, 4.0],
            [0.0, 1.0, 4.0, 2.0],
            [5.0, 6.0, 0.0, 1.0],
            [2.0, 7.0, 1.0, 3.0],
        ],
    }
}

/// Return base, perturbed all-ones, mixed-scale and floating-constant left 4x4 matrices.
///
/// Three diagonal additions near 1e-6 are evaluated in binary64; another case ranges
/// from 1e-6 to 1e6. The constant-rich case includes rounded pi/e/sqrt(2). The finite
/// array provides selected stress inputs, not a rank or conditioning certificate.
fn sample_mat4_cases() -> [SampleMat4; 4] {
    [
        sample_mat4(),
        SampleMat4 {
            m: [
                [1.0, 1.0, 1.0, 1.0],
                [1.0, 1.0 + 1.0e-6, 1.0, 1.0],
                [1.0, 1.0, 1.0 + 2.0e-6, 1.0],
                [1.0, 1.0, 1.0, 1.0 + 3.0e-6],
            ],
        },
        SampleMat4 {
            m: [
                [1.0e6, 2.0, -3.0, 4.0],
                [5.0, -1.0e-6, 7.0, -8.0],
                [9.0, -10.0, 1.0e4, 12.0],
                [-13.0, 14.0, -15.0, 1.0e-3],
            ],
        },
        SampleMat4 {
            m: [
                [std::f64::consts::PI, -std::f64::consts::E, 1.0e-9, 2.0],
                [std::f64::consts::SQRT_2, 1.0, -2.0, 3.0],
                [5.0, -8.0, 13.0, -21.0],
                [34.0, -55.0, 89.0, 144.0],
            ],
        },
    ]
}

/// Return the fixed small-integer 4x4 right matrix with general last row (23,29,31,37).
///
/// The integer coefficients survive exact binary64 import unchanged; no determinant
/// or affine-transform precondition is checked during construction.
fn sample_mat4_b() -> SampleMat4 {
    SampleMat4 {
        m: [
            [2.0, 0.0, 1.0, 3.0],
            [3.0, 5.0, 7.0, 11.0],
            [11.0, 13.0, 17.0, 19.0],
            [23.0, 29.0, 31.0, 37.0],
        ],
    }
}

/// Return right 4x4 partners: base, perturbed all-twos, mixed-scale and floating constants.
///
/// The perturbed case adds 5e-6, 7e-6 and 11e-6 in binary64. Later cases vary scale/sign
/// and use rounded pi-related constants; no inverse or output expectation is computed.
fn sample_mat4_b_cases() -> [SampleMat4; 4] {
    [
        sample_mat4_b(),
        SampleMat4 {
            m: [
                [2.0, 2.0, 2.0, 2.0],
                [2.0, 2.0 + 5.0e-6, 2.0, 2.0],
                [2.0, 2.0, 2.0 + 7.0e-6, 2.0],
                [2.0, 2.0, 2.0, 2.0 + 11.0e-6],
            ],
        },
        SampleMat4 {
            m: [
                [-1.0e5, 3.0, 5.0, -7.0],
                [11.0, 1.0e-6, -13.0, 17.0],
                [-19.0, 23.0, 1.0e4, -29.0],
                [31.0, -37.0, 41.0, -1.0e-3],
            ],
        },
        SampleMat4 {
            m: [
                [
                    -std::f64::consts::FRAC_1_PI,
                    std::f64::consts::FRAC_2_PI,
                    -1.0e-9,
                    -2.0,
                ],
                [std::f64::consts::FRAC_2_SQRT_PI, -1.0, 2.0, -3.0],
                [-5.0, 8.0, -13.0, 21.0],
                [-34.0, 55.0, -89.0, -144.0],
            ],
        },
    ]
}

/// Return four left 3D affine homogeneous fixtures in 4x4 storage.
///
/// Each authored bottom row is (0,0,0,1); the last column supplies translation.
/// The first has identity linear action; later cases combine scaling and off-diagonal
/// linear terms. Coefficients are binary64 values, and no invertibility or condition
/// check is performed. Arbitrary downstream scalar conversion need not preserve form.
fn sample_mat4_affine_cases() -> [SampleMat4; 4] {
    [
        SampleMat4 {
            m: [
                [1.0, 0.0, 0.0, 1.2],
                [0.0, 1.0, 0.0, -0.8],
                [0.0, 0.0, 1.0, 3.4],
                [0.0, 0.0, 0.0, 1.0],
            ],
        },
        SampleMat4 {
            m: [
                [1.2, 0.1, -0.2, -2.0],
                [0.3, 0.9, 0.05, 5.5],
                [0.2, -0.1, 1.1, -4.2],
                [0.0, 0.0, 0.0, 1.0],
            ],
        },
        SampleMat4 {
            m: [
                [0.95, 0.2, 0.1, 10.0],
                [-0.1, 1.03, 0.4, -6.0],
                [0.07, -0.02, 1.01, 8.0],
                [0.0, 0.0, 0.0, 1.0],
            ],
        },
        SampleMat4 {
            m: [
                [1.4, 0.0, 0.2, 0.5],
                [0.0, 1.25, 0.0, -1.5],
                [0.0, 0.0, 0.8, 2.5],
                [0.0, 0.0, 0.0, 1.0],
            ],
        },
    ]
}

/// Return four right 3D affine homogeneous fixtures in 4x4 storage.
///
/// Each authored bottom row is (0,0,0,1); the last column supplies translation.
/// The first has identity linear action; later cases combine scaling and off-diagonal
/// linear terms. Coefficients are binary64 values, and no invertibility or condition
/// check is performed. Arbitrary downstream scalar conversion need not preserve form.
fn sample_mat4_affine_b_cases() -> [SampleMat4; 4] {
    [
        SampleMat4 {
            m: [
                [1.0, 0.0, 0.0, -3.0],
                [0.0, 1.0, 0.0, 2.5],
                [0.0, 0.0, 1.0, -1.0],
                [0.0, 0.0, 0.0, 1.0],
            ],
        },
        SampleMat4 {
            m: [
                [1.1, 0.2, 0.0, 1.5],
                [0.0, 0.95, 0.3, -2.5],
                [0.0, -0.2, 1.05, 3.3],
                [0.0, 0.0, 0.0, 1.0],
            ],
        },
        SampleMat4 {
            m: [
                [0.9, -0.1, 0.05, -7.0],
                [0.12, 1.15, -0.05, 4.0],
                [-0.2, 0.03, 1.02, -3.5],
                [0.0, 0.0, 0.0, 1.0],
            ],
        },
        SampleMat4 {
            m: [
                [1.0, 0.0, -0.3, 6.5],
                [0.0, 0.88, 0.0, 2.5],
                [0.0, 0.0, 1.0, -1.5],
                [0.0, 0.0, 0.0, 1.0],
            ],
        },
    ]
}

/// Return four left pure 3D translations with identity linear block.
///
/// Keeps the translation columns of sample_mat4_affine_cases while replacing
/// the linear block by identity and retaining bottom row (0,0,0,1). The authored
/// matrices have determinant one; no measured dispatch or downstream conversion
/// invariance follows from that pattern alone.
fn sample_mat4_affine_translation_cases() -> [SampleMat4; 4] {
    [
        SampleMat4 {
            m: [
                [1.0, 0.0, 0.0, 1.2],
                [0.0, 1.0, 0.0, -0.8],
                [0.0, 0.0, 1.0, 3.4],
                [0.0, 0.0, 0.0, 1.0],
            ],
        },
        SampleMat4 {
            m: [
                [1.0, 0.0, 0.0, -2.0],
                [0.0, 1.0, 0.0, 5.5],
                [0.0, 0.0, 1.0, -4.2],
                [0.0, 0.0, 0.0, 1.0],
            ],
        },
        SampleMat4 {
            m: [
                [1.0, 0.0, 0.0, 10.0],
                [0.0, 1.0, 0.0, -6.0],
                [0.0, 0.0, 1.0, 8.0],
                [0.0, 0.0, 0.0, 1.0],
            ],
        },
        SampleMat4 {
            m: [
                [1.0, 0.0, 0.0, 0.5],
                [0.0, 1.0, 0.0, -1.5],
                [0.0, 0.0, 1.0, 2.5],
                [0.0, 0.0, 0.0, 1.0],
            ],
        },
    ]
}

/// Return four right pure 3D translations with identity linear block.
///
/// Keeps the translation columns of sample_mat4_affine_b_cases while replacing
/// the linear block by identity and retaining bottom row (0,0,0,1). The authored
/// matrices have determinant one; no measured dispatch or downstream conversion
/// invariance follows from that pattern alone.
fn sample_mat4_affine_translation_b_cases() -> [SampleMat4; 4] {
    [
        SampleMat4 {
            m: [
                [1.0, 0.0, 0.0, -3.0],
                [0.0, 1.0, 0.0, 2.5],
                [0.0, 0.0, 1.0, -1.0],
                [0.0, 0.0, 0.0, 1.0],
            ],
        },
        SampleMat4 {
            m: [
                [1.0, 0.0, 0.0, 1.5],
                [0.0, 1.0, 0.0, -2.5],
                [0.0, 0.0, 1.0, 3.3],
                [0.0, 0.0, 0.0, 1.0],
            ],
        },
        SampleMat4 {
            m: [
                [1.0, 0.0, 0.0, -7.0],
                [0.0, 1.0, 0.0, 4.0],
                [0.0, 0.0, 1.0, -3.5],
                [0.0, 0.0, 0.0, 1.0],
            ],
        },
        SampleMat4 {
            m: [
                [1.0, 0.0, 0.0, 6.5],
                [0.0, 1.0, 0.0, 2.5],
                [0.0, 0.0, 1.0, -1.5],
                [0.0, 0.0, 0.0, 1.0],
            ],
        },
    ]
}

/// Return four integer 4x4 matrices with diagonal or triangular sparse support.
///
/// Includes diag(1,2,3,4), an upper-triangular matrix with one off-diagonal five, a unit
/// lower-bidiagonal matrix, and diag(3,5,7,11) with entry (row 3,column 1)=1 (zero-based).
/// The final weight row is general, not affine. No sparse dispatch is asserted.
fn sample_mat4_sparse_cases() -> [SampleMat4; 4] {
    [
        SampleMat4 {
            m: [
                [1.0, 0.0, 0.0, 0.0],
                [0.0, 2.0, 0.0, 0.0],
                [0.0, 0.0, 3.0, 0.0],
                [0.0, 0.0, 0.0, 4.0],
            ],
        },
        SampleMat4 {
            m: [
                [2.0, 5.0, 0.0, 0.0],
                [0.0, -3.0, 0.0, 0.0],
                [0.0, 0.0, 7.0, 0.0],
                [0.0, 0.0, 0.0, 11.0],
            ],
        },
        SampleMat4 {
            m: [
                [1.0, 0.0, 0.0, 0.0],
                [4.0, 1.0, 0.0, 0.0],
                [0.0, 8.0, 1.0, 0.0],
                [0.0, 0.0, 12.0, 1.0],
            ],
        },
        SampleMat4 {
            m: [
                [3.0, 0.0, 0.0, 0.0],
                [0.0, 5.0, 0.0, 0.0],
                [0.0, 0.0, 7.0, 0.0],
                [0.0, 1.0, 0.0, 11.0],
            ],
        },
    ]
}

/// Borrow the current case and advance a Cell cursor cyclically by the slice length.
///
/// Requires a nonempty slice and an initial cursor strictly below its length. Empty
/// input panics on remainder by zero; an out-of-range cursor can advance before indexing
/// panics, and index+1 uses ordinary usize arithmetic. No recovery or synchronization
/// is supplied. Callers use this to rotate fixed cases, not to randomize selection.
fn next_case<'a, T>(cases: &'a [T], cursor: &Cell<usize>) -> &'a T {
    let index = cursor.get();
    cursor.set((index + 1) % cases.len());
    &cases[index]
}

/// Import a finite binary64 fixture as its exact binary value, panicking on NaN/infinity.
///
/// For example s(0.1) is the dyadic value of 0.1_f64, not the rational 1/10. This edge
/// conversion performs no decimal reconstruction or symbolic constant recognition.
fn s(value: f64) -> Real {
    Real::try_from(value).expect("finite benchmark scalar")
}

/// Author an exact ratio from integer numerator and nonzero denominator, reducing n/d.
///
/// A zero denominator panics at expect. No float conversion occurs; all current fixed
/// ratio fixtures supply positive denominators.
fn q(numerator: i64, denominator: u64) -> Real {
    Real::from(Rational::fraction(numerator, denominator).expect("valid benchmark rational"))
}

/// Author a rational from the input float's displayed decimal string, panicking on parse failure.
///
/// Uses f64::to_string then Rational parsing. This is a different input model from s:
/// qr(0.1) is 1/10, while s(0.1) retains the exact dyadic float. Formatting follows the
/// already-evaluated float, not the original source expression or intended constant;
/// qr of floating pi is still a decimal approximation to pi. NaN/infinity are not
/// admitted rational strings. This helper is fixture construction, not exact float import.
fn qr(value: f64) -> Real {
    Real::from(
        value
            .to_string()
            .parse::<Rational>()
            .expect("finite benchmark decimal rational"),
    )
}

/// Map the stored 3-vector's x/y/z lanes through s, preserving exact binary64 values.
///
/// Nonfinite lanes panic in s; this performs no normalization or geometric validation.
fn blas_vec3(value: SampleVec3) -> Vector3 {
    blas_vec3_with(value, s)
}

/// Apply the caller's converter once to x, y, z in that order and construct Vector3.
///
/// Exactness, errors and side effects belong to make_scalar. Copy/Fn does not guarantee
/// purity; no independent validation or rescaling is added.
fn blas_vec3_with<F>(value: SampleVec3, make_scalar: F) -> Vector3
where
    F: Copy + Fn(f64) -> Real,
{
    Vector3::new([
        make_scalar(value.x),
        make_scalar(value.y),
        make_scalar(value.z),
    ])
}

/// Map all four stored lanes through s, including the unrestricted w coordinate.
///
/// Nonfinite lanes panic; no homogeneous-weight check or perspective division occurs.
fn blas_vec4(value: SampleVec4) -> Vector4 {
    blas_vec4_with(value, s)
}

/// Convert x/y/z/w in order with the supplied function and construct Vector4.
///
/// The converter controls scalar meaning and failure; the fourth lane is not forced
/// to zero or one. No point/direction precondition is established.
fn blas_vec4_with<F>(value: SampleVec4, make_scalar: F) -> Vector4
where
    F: Copy + Fn(f64) -> Real,
{
    Vector4::new([
        make_scalar(value.x),
        make_scalar(value.y),
        make_scalar(value.z),
        make_scalar(value.w),
    ])
}

/// Convert the stored 3x3 matrix through s in row-major order, retaining binary values.
///
/// Nonfinite entries panic; shape storage does not imply invertibility or affine validity.
fn blas_mat3(value: SampleMat3) -> Matrix3 {
    blas_mat3_with(value, s)
}

/// Map each row and its three entries through make_scalar, then store Matrix3.
///
/// Row/column order is preserved. The caller chooses conversion semantics and failure;
/// there is no implicit transpose, exactness certification or matrix validation.
fn blas_mat3_with<F>(value: SampleMat3, make_scalar: F) -> Matrix3
where
    F: Copy + Fn(f64) -> Real,
{
    Matrix3::new(value.m.map(|row| row.map(make_scalar)))
}

/// Convert the stored 4x4 matrix through s in row-major order, retaining binary values.
///
/// No last-row normalization or matrix-domain validation occurs; nonfinite entries panic.
fn blas_mat4(value: SampleMat4) -> Matrix4 {
    blas_mat4_with(value, s)
}

/// Map the 4x4 entries in row-major order with the caller's scalar converter.
///
/// Preserves index layout without transposition. The converter owns input meaning,
/// purity and failure; no affine or invertibility check is added.
fn blas_mat4_with<F>(value: SampleMat4, make_scalar: F) -> Matrix4
where
    F: Copy + Fn(f64) -> Real,
{
    Matrix4::new(value.m.map(|row| row.map(make_scalar)))
}

/// Author the left base vector's displayed decimal coordinates as exact ratios over 10^14.
///
/// Ratios reduce independently. They express the source decimal spellings rather than
/// the exact binary64 values produced by sample_vec3; equal authored denominators do
/// not promise equal reduced denominators or a shared-scale schedule.
fn blas_vec3_rational() -> Vector3 {
    Vector3::new([
        q(123_456_789_012_345, 100_000_000_000_000),
        q(-234_567_890_123_456, 100_000_000_000_000),
        q(345_678_901_234_567, 100_000_000_000_000),
    ])
}

/// Author the right base vector's long decimal spellings as exact ratios over 10^14.
///
/// Each ratio is independently reduced; this is the decimal model corresponding to
/// sample_vec3_b, not a claim of equality to its binary64-imported coordinates.
fn blas_vec3_b_rational() -> Vector3 {
    Vector3::new([
        q(-98_765_432_101_234, 100_000_000_000_000),
        q(421_098_765_432_109, 100_000_000_000_000),
        q(-567_890_123_456_789, 100_000_000_000_000),
    ])
}

/// Construct exact integer (3,4,5,1), equal in value to direct import of sample_vec4.
fn blas_vec4_rational() -> Vector4 {
    Vector4::new([3.into(), 4.into(), 5.into(), 1.into()])
}

/// Construct the left base 3x3 matrix from authored tenths rather than rounded floats.
///
/// Reduction can change each denominator independently; this does not promise a
/// common reduced denominator even though every supplied denominator is ten.
fn blas_mat3_rational() -> Matrix3 {
    Matrix3::new([
        [q(12, 10), q(3, 10), q(-7, 10)],
        [q(21, 10), q(-15, 10), q(9, 10)],
        [q(4, 10), q(33, 10), q(22, 10)],
    ])
}

/// Construct the right base 3x3 matrix from authored tenths, reduced entry by entry.
///
/// Matches the decimal spellings of sample_mat3_b, not necessarily its binary64 values.
fn blas_mat3_b_rational() -> Matrix3 {
    Matrix3::new([
        [q(-8, 10), q(11, 10), q(5, 10)],
        [q(27, 10), q(6, 10), q(-14, 10)],
        [q(32, 10), q(-9, 10), q(18, 10)],
    ])
}

/// Construct the exact integer left 4x4 matrix, matching sample_mat4 entry values.
fn blas_mat4_rational() -> Matrix4 {
    Matrix4::new([
        [1.into(), 2.into(), 3.into(), 4.into()],
        [0.into(), 1.into(), 4.into(), 2.into()],
        [5.into(), 6.into(), 0.into(), 1.into()],
        [2.into(), 7.into(), 1.into(), 3.into()],
    ])
}

/// Construct the exact integer right 4x4 matrix, matching sample_mat4_b entry values.
fn blas_mat4_b_rational() -> Matrix4 {
    Matrix4::new([
        [2.into(), 0.into(), 1.into(), 3.into()],
        [3.into(), 5.into(), 7.into(), 11.into()],
        [11.into(), 13.into(), 17.into(), 19.into()],
        [23.into(), 29.into(), 31.into(), 37.into()],
    ])
}

/// Convert nine supplied integer pairs in row-major order into a Matrix3.
///
/// Does not divide itself or reject zero denominators: make_ratio determines pair
/// meaning, validation and failures. No matrix shape fact beyond dimension is certified.
fn ratio_matrix3_with<F>(entries: [[(i64, u64); 3]; 3], make_ratio: F) -> Matrix3
where
    F: Copy + Fn(i64, u64) -> Real,
{
    Matrix3::new(entries.map(|row| row.map(|(n, d)| make_ratio(n, d))))
}

/// Convert sixteen integer pairs in row-major order through the supplied ratio function.
///
/// The callback owns exactness and denominator validation. No transpose, common-scale
/// normalization or invertibility check occurs before storing Matrix4.
fn ratio_matrix4_with<F>(entries: [[(i64, u64); 4]; 4], make_ratio: F) -> Matrix4
where
    F: Copy + Fn(i64, u64) -> Real,
{
    Matrix4::new(entries.map(|row| row.map(|(n, d)| make_ratio(n, d))))
}

/// Paired 3x3/4x4 operands labeled by their authored coefficient patterns.
///
/// The label describes the input ratio table, not verified reduced denominators,
/// actual internal storage, invertibility or an observed dispatch schedule. Conversion
/// is supplied by targeted_matrix_forms_with and can change represented values.
#[derive(Clone)]
struct TargetedMatrixForm {
    /// Static report label for the authored coefficient pattern.
    name: &'static str,
    /// Left 3x3 operand produced by the chosen ratio converter.
    lhs3: Matrix3,
    /// Right 3x3 operand produced by the chosen ratio converter.
    rhs3: Matrix3,
    /// Left 4x4 operand produced by the chosen ratio converter.
    lhs4: Matrix4,
    /// Right 4x4 operand produced by the chosen ratio converter.
    rhs4: Matrix4,
}

/// Build four paired-size coefficient-pattern fixtures using the caller's ratio converter.
///
/// Order is dyadic_dense (power-of-two denominators), equal_decimal_den (authored
/// denominator ten), mixed_prime_den (small prime denominators) and sparse_integer
/// (denominator one with explicit zeros). Each contains left/right 3x3 and 4x4 matrices.
/// The labels describe the ratio tables; fraction reduction can make decimal
/// denominators unequal. An arbitrary converter can also alter zeros, ones or values,
/// so labels do not establish actual structure, dispatch, nonsingularity or equality
/// between exact-ratio and rounded-float comparisons. No operation is executed here.
fn targeted_matrix_forms_with<F>(make_ratio: F) -> [TargetedMatrixForm; 4]
where
    F: Copy + Fn(i64, u64) -> Real,
{
    [
        TargetedMatrixForm {
            name: "dyadic_dense",
            lhs3: ratio_matrix3_with(
                [
                    [(9, 8), (3, 16), (-5, 8)],
                    [(7, 4), (-11, 8), (13, 16)],
                    [(5, 8), (17, 16), (19, 8)],
                ],
                make_ratio,
            ),
            rhs3: ratio_matrix3_with(
                [
                    [(11, 8), (-3, 16), (7, 8)],
                    [(5, 4), (13, 8), (-9, 16)],
                    [(1, 8), (15, 16), (17, 8)],
                ],
                make_ratio,
            ),
            lhs4: ratio_matrix4_with(
                [
                    [(9, 8), (3, 16), (-5, 8), (7, 4)],
                    [(11, 8), (-13, 16), (15, 8), (5, 16)],
                    [(-7, 8), (17, 16), (19, 8), (-3, 4)],
                    [(5, 8), (21, 16), (-11, 8), (23, 16)],
                ],
                make_ratio,
            ),
            rhs4: ratio_matrix4_with(
                [
                    [(13, 8), (5, 16), (7, 8), (-9, 4)],
                    [(3, 8), (17, 16), (-5, 8), (11, 16)],
                    [(19, 8), (-7, 16), (23, 8), (5, 4)],
                    [(-1, 8), (9, 16), (15, 8), (25, 16)],
                ],
                make_ratio,
            ),
        },
        TargetedMatrixForm {
            name: "equal_decimal_den",
            lhs3: ratio_matrix3_with(
                [
                    [(12, 10), (3, 10), (-7, 10)],
                    [(21, 10), (-15, 10), (9, 10)],
                    [(4, 10), (33, 10), (22, 10)],
                ],
                make_ratio,
            ),
            rhs3: ratio_matrix3_with(
                [
                    [(-8, 10), (11, 10), (5, 10)],
                    [(27, 10), (6, 10), (-14, 10)],
                    [(32, 10), (-9, 10), (18, 10)],
                ],
                make_ratio,
            ),
            lhs4: ratio_matrix4_with(
                [
                    [(11, 10), (2, 10), (3, 10), (4, 10)],
                    [(5, 10), (17, 10), (7, 10), (-8, 10)],
                    [(9, 10), (-10, 10), (23, 10), (12, 10)],
                    [(-13, 10), (14, 10), (-15, 10), (19, 10)],
                ],
                make_ratio,
            ),
            rhs4: ratio_matrix4_with(
                [
                    [(20, 10), (3, 10), (5, 10), (-7, 10)],
                    [(11, 10), (25, 10), (-13, 10), (17, 10)],
                    [(-19, 10), (23, 10), (31, 10), (-29, 10)],
                    [(31, 10), (-37, 10), (41, 10), (43, 10)],
                ],
                make_ratio,
            ),
        },
        TargetedMatrixForm {
            name: "mixed_prime_den",
            lhs3: ratio_matrix3_with(
                [
                    [(7, 3), (-5, 7), (11, 13)],
                    [(13, 5), (17, 11), (-19, 17)],
                    [(-23, 19), (29, 23), (31, 29)],
                ],
                make_ratio,
            ),
            rhs3: ratio_matrix3_with(
                [
                    [(5, 3), (7, 5), (-11, 7)],
                    [(13, 11), (-17, 13), (19, 17)],
                    [(23, 19), (-29, 23), (37, 31)],
                ],
                make_ratio,
            ),
            lhs4: ratio_matrix4_with(
                [
                    [(7, 3), (-5, 7), (11, 13), (13, 5)],
                    [(17, 11), (-19, 17), (23, 19), (-29, 23)],
                    [(31, 29), (37, 31), (-41, 37), (43, 41)],
                    [(-47, 43), (53, 47), (59, 53), (61, 59)],
                ],
                make_ratio,
            ),
            rhs4: ratio_matrix4_with(
                [
                    [(11, 3), (13, 5), (-17, 7), (19, 11)],
                    [(23, 13), (-29, 17), (31, 19), (37, 23)],
                    [(-41, 29), (43, 31), (47, 37), (-53, 41)],
                    [(59, 43), (-61, 47), (67, 53), (71, 59)],
                ],
                make_ratio,
            ),
        },
        TargetedMatrixForm {
            name: "sparse_integer",
            lhs3: ratio_matrix3_with(
                [
                    [(2, 1), (0, 1), (1, 1)],
                    [(0, 1), (3, 1), (1, 1)],
                    [(1, 1), (0, 1), (2, 1)],
                ],
                make_ratio,
            ),
            rhs3: ratio_matrix3_with(
                [
                    [(3, 1), (1, 1), (0, 1)],
                    [(1, 1), (2, 1), (1, 1)],
                    [(0, 1), (1, 1), (4, 1)],
                ],
                make_ratio,
            ),
            lhs4: ratio_matrix4_with(
                [
                    [(2, 1), (0, 1), (1, 1), (0, 1)],
                    [(1, 1), (3, 1), (0, 1), (1, 1)],
                    [(0, 1), (2, 1), (1, 1), (0, 1)],
                    [(1, 1), (0, 1), (0, 1), (2, 1)],
                ],
                make_ratio,
            ),
            rhs4: ratio_matrix4_with(
                [
                    [(3, 1), (0, 1), (1, 1), (0, 1)],
                    [(0, 1), (2, 1), (1, 1), (1, 1)],
                    [(1, 1), (0, 1), (4, 1), (0, 1)],
                    [(0, 1), (1, 1), (0, 1), (5, 1)],
                ],
                make_ratio,
            ),
        },
    ]
}
