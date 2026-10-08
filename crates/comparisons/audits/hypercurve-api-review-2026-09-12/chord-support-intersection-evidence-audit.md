# Retained chord pair replay and endpoint transport

The common curve/open-path dispatcher now consumes chord/chord and chord/rational-source pairs. Native rational fast paths remain available. Region and open-curve callers share the finite geometry kernels; authored adjacency remains a region decision, so open curves retain their endpoint contacts. The duplicate region exact-linear adapter is removed.

The independent fixture is the unit-setback chamfer between the line ending at `(0,0)` and `Q(t)=(t²,2t)`. Its chord runs from `A=(-1,0)` to `B=(s²,2s)`, where `s²=sqrt(5)-2`. An independently authored line uses those exact radical coordinates. A separate intersection selects `alpha=sqrt(1/2)` as a non-scalar parameter for cutting that line.

## Reusing the overlap proof

Clipping the line at alpha originally blocked with `Predicate` when the chord/source mapper reconstructed Cartesian finite membership in independent coefficient and parameter fields. The monotone overlap branch already certifies that interval. The mapper now compares against its retained source boundaries, reuses paired endpoints, rejects exterior source parameters, and certifies strict interior chord membership from strict source-interval membership. Selected and recursive parameters use the existing native point-evaluation helper without promotion to a Bezier root.

The public evaluation consumer also reuses a named endpoint or certified strict-interior parameter on the identical finite chord. Parameters from another chord retain the full incidence check. No approximate geometry substitutes for a certificate.

## One overlap endpoint contract

The new common adapter initially assumed that chord/chord overlap endpoints were paired. That older internal type instead stored each range in its own traversal order. The reversed-chord selected-tail test exposed the mismatch by evaluating a published boundary to A rather than the required selected cut. A second regression exposed it after a contact parameter passed through chord splitting, overlap publication, native-line splitting, and another collinear intersection.

Chord/chord overlap ranges now both follow the first chord's traversal: the second range descends for opposite orientations. The private type documents that contract, its unit-test caller checks paired point identities and the correct order, and all shared consumers use it directly. Region topology already determines endpoint correspondence from both range directions and overlap orientation. No compatibility adapter or alternative legacy convention remains.

## Retaining tangent replay through the affine-line shortcut

The first broad qualification found one actual evidence regression: a recursive scalar produced by the finite chord shortcut lacked the chord/rational tangent certificate expected by later corner predicates. The shortcut now attaches the existing certificate, including the finite contact location, to a fresh recursive scalar. If that scalar already carries specialized identity, the optional shortcut declines and the complete kernel retains that evidence; it never replaces a prior identity.

The existing third-field contact test remains unchanged and checks reusable tangent replay. Two trim tests now check independent exact endpoints and source-parameter evaluation instead of requiring AlgebraicEndpointImages and the absence of scalar views. The retained-offset test explicitly exercises the recursive kernel and also checks shared dispatch against the independent closed-form offset/line intersection. The first broad run and its four failures are preserved in chord-support-intersection-pre-evidence-qualification/.

## Regression scope

Three public tests cover 224 primary queries under both policies, both operand orders and independent traversal reversals: 128 curve/open-path crossing, endpoint and disjointness queries; 48 full/selected overlap queries; and 48 composed collinear endpoint queries. They compare against independently authored exact points, check paired overlap endpoints, and feed returned parameters into evaluation and subdivision. Four additional setup intersections select and transport the composed cut.

The focused public tests and all four evidence/trim regressions pass. The final rebuilt independent generated-pair probe reports zero unresolved pairs (18 at ef0354f and 44 before retained rational dispatch), with eight certified empty region XORs against independently authored geometry before/after boundary-path readmission. This is coverage of the stated fixtures, not full-family closure or a performance claim. The final rebuilt probe and broad qualification are recorded separately in chord-support-intersection-qualification.json.

The original goal remains active. Selected-circle/parallel dispatch, exterior source domains, complete retraced and partial non-injective overlap relations, general curve/path topology, existing promotion failures and previously unqualified expensive cases remain open.
