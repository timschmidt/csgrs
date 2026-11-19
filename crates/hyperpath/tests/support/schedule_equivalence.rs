use hyperlimit::{Point2, PredicatePolicy};
use hyperpath::{
    CornerLookaheadJoinClass, FeedPathElement, LinePathSegment, LookaheadFeedPlanningLimits,
    LookaheadFeedScheduleReport, TangentAlignment, TangentJoinClass, TangentSpan,
    certify_lookahead_feed_schedule, plan_lookahead_feed_schedule,
};
use hyperreal::{Real, RealSign};
use hypersolve::{CandidateCertificationReport, CertifiedCandidateStatus, ConstraintKind};

struct Path {
    points: Vec<(i64, i64)>,
    lengths: Vec<i64>,
    directions: Vec<u8>,
    caps: Vec<i64>,
    radii: Vec<i64>,
    global: i64,
    acceleration: i64,
}

impl Path {
    fn decode(data: &[u8]) -> Option<Self> {
        let (&[flags, limits, entry], tail) = data.split_first_chunk()?;
        let mut p = Self {
            points: vec![(i64::from(entry), -i64::from(flags))],
            lengths: vec![],
            directions: vec![],
            caps: vec![i64::from(entry & 15)],
            radii: vec![],
            global: i64::from(limits & 15) + 1,
            acceleration: i64::from(limits >> 4) + 1,
        };
        let (u, v, scale) = if flags & 1 == 0 { (1, 0, 1) } else { (3, 4, 5) };
        let (edges, _) = tail.as_chunks::<2>();
        for edge in edges.iter().take(6) {
            let direction = edge[0] >> 6;
            let (dx, dy) = [(u, v), (-v, u), (-u, -v), (v, -u)][usize::from(direction)];
            let length = i64::from(edge[0] & 63) + 1;
            let (x, y) = *p.points.last().unwrap();
            p.points.push((x + length * dx, y + length * dy));
            p.lengths.push(length * scale);
            p.directions.push(direction);
            p.caps.push(i64::from(edge[1] & 15));
            p.radii.push(i64::from(edge[1] >> 4));
        }
        p.radii.pop()?; // The exit has no join radius.
        Some(p)
    }

    fn reverse(&mut self) {
        self.points.reverse();
        self.directions.reverse();
        for direction in &mut self.directions {
            *direction = (*direction + 2) % 4;
        }
        self.lengths.reverse();
        self.caps.reverse();
        self.radii.reverse();
    }

    fn join(&self, j: usize) -> CornerLookaheadJoinClass {
        match self.directions[j].abs_diff(self.directions[j + 1]) {
            0 => CornerLookaheadJoinClass::StraightThrough,
            2 => CornerLookaheadJoinClass::ReversalStop,
            _ => CornerLookaheadJoinClass::RadiusLimitedCorner,
        }
    }

    fn check(&self, edit: u8, policy: PredicatePolicy) -> Vec<i64> {
        let point = |(x, y): (i64, i64)| Point2::new(x.into(), y.into());
        let lines: Vec<_> = self
            .points
            .windows(2)
            .map(|ab| LinePathSegment::new(point(ab[0]), point(ab[1]), policy).unwrap())
            .collect();
        let spans: Vec<_> = lines.iter().map(TangentSpan::from_line_segment).collect();
        let route: Vec<_> = lines.into_iter().map(FeedPathElement::Line).collect();
        let n = route.len();
        let limits = LookaheadFeedPlanningLimits {
            maximum_entry_feed: self.caps[0].into(),
            maximum_corner_feeds: self.caps[1..n].iter().copied().map(Real::from).collect(),
            corner_radii: self.radii.iter().copied().map(Real::from).collect(),
            maximum_exit_feed: self.caps[n].into(),
        };
        let planned = plan_lookahead_feed_schedule(
            &route,
            &spans,
            &limits,
            self.global.into(),
            self.acceleration.into(),
            policy,
        )
        .unwrap();
        let mut positions = vec![0];
        for length in &self.lengths {
            positions.push(positions.last().unwrap() + length);
        }
        let mut ceilings: Vec<_> = self
            .caps
            .iter()
            .map(|v| (*v).min(self.global).pow(2))
            .collect();
        for (j, &radius) in self.radii.iter().enumerate() {
            match self.join(j) {
                CornerLookaheadJoinClass::RadiusLimitedCorner => {
                    ceilings[j + 1] = ceilings[j + 1].min(self.acceleration * radius)
                }
                CornerLookaheadJoinClass::ReversalStop => ceilings[j + 1] = 0,
                CornerLookaheadJoinClass::StraightThrough => {}
            }
        }
        // Every local ceiling constrains every node by its arclength distance.
        // Their minimum attains all adjacent squared-speed difference bounds.
        let bound = |i: usize, count: usize| {
            ceilings[..count]
                .iter()
                .enumerate()
                .map(|(j, &limit)| {
                    limit + 2 * self.acceleration * (positions[j] - positions[i]).abs()
                })
                .min()
                .unwrap()
        };
        let mut selected: Vec<_> = (0..=n).map(|i| bound(i, n + 1)).collect();
        assert_eq!(planned.effective_node_feed_limits.len(), n + 1);
        assert_eq!(planned.forward_node_feeds.len(), n + 1);
        assert_eq!(planned.caller_limit_certifications.len(), n + 1);
        assert!(planned.all_satisfied());
        for i in 0..=n {
            check_speed(&planned.effective_node_feed_limits[i], ceilings[i]);
            check_speed(&planned.forward_node_feeds[i], bound(i, i + 1));
            check_rows(
                &planned.caller_limit_certifications[i],
                &[cap_row(
                    "lookahead caller node feed cap",
                    selected[i],
                    self.caps[i],
                )],
            );
        }
        let mut schedule = planned.schedule;
        assert_eq!(schedule.corner_radii, limits.corner_radii);
        assert_eq!(schedule.corner_feeds.len(), n - 1);
        for (speed, &sq) in std::iter::once(&schedule.entry_feed)
            .chain(&schedule.corner_feeds)
            .chain(std::iter::once(&schedule.exit_feed))
            .zip(&selected)
        {
            check_speed(speed, sq);
        }
        self.check_report(&planned.certification, &selected);
        let certify = |schedule: &hyperpath::LookaheadFeedSchedule| {
            certify_lookahead_feed_schedule(
                &route,
                &spans,
                schedule,
                self.global.into(),
                self.acceleration.into(),
                policy,
            )
            .unwrap()
        };
        let replay = certify(&schedule);
        self.check_report(&replay, &selected);
        assert!(statuses(&planned.certification).eq(statuses(&replay)));

        let original = selected.clone();
        let i = usize::from(edit >> 4) % (n + 1);
        let replacement = i64::from(edit & 15);
        selected[i] = replacement.pow(2);
        *match i {
            0 => &mut schedule.entry_feed,
            _ if i == n => &mut schedule.exit_feed,
            _ => &mut schedule.corner_feeds[i - 1],
        } = replacement.into();
        let replay = certify(&schedule);
        self.check_report(&replay, &selected);
        original
    }

    fn check_report(&self, report: &LookaheadFeedScheduleReport, speeds: &[i64]) {
        assert_eq!(report.spans.len(), self.lengths.len());
        assert_eq!(report.corners.joins.len(), self.radii.len());
        let mut first_bad = None;
        let mut corners_ok = true;
        for (i, span) in report.spans.iter().enumerate() {
            let (a, b) = (speeds[i], speeds[i + 1]);
            let travel = 2 * self.acceleration * self.lengths[i];
            assert_eq!(span.index, i as u64);
            assert_eq!(span.path_length, Real::from(self.lengths[i]));
            assert_eq!(span.max_feed_rate, Real::from(self.global));
            assert_eq!(span.max_acceleration, Real::from(self.acceleration));
            check_speed(&span.start_feed, a);
            check_speed(&span.end_feed, b);
            if !check_rows(
                &span.certification,
                &[
                    cap_row("lookahead span start feed cap", a, self.global),
                    cap_row("lookahead span end feed cap", b, self.global),
                    upper_row("lookahead span acceleration distance", b - a - travel),
                    upper_row("lookahead span deceleration distance", a - b - travel),
                ],
            ) {
                first_bad.get_or_insert(i);
            }
        }
        for (j, join) in report.corners.joins.iter().enumerate() {
            let v = speeds[j + 1];
            assert_eq!(join.index, j as u64);
            assert_eq!(join.class, self.join(j));
            assert_eq!(join.corner_radius, Real::from(self.radii[j]));
            check_speed(&join.candidate_corner_feed, v);
            assert_eq!(join.max_feed_rate, Real::from(self.global));
            assert_eq!(join.max_acceleration, Real::from(self.acceleration));
            let (class, alignment) = match self.join(j) {
                CornerLookaheadJoinClass::StraightThrough => (
                    TangentJoinClass::G1Continuous,
                    TangentAlignment::SameDirection,
                ),
                CornerLookaheadJoinClass::ReversalStop => (
                    TangentJoinClass::ReversedTangent,
                    TangentAlignment::OppositeDirection,
                ),
                CornerLookaheadJoinClass::RadiusLimitedCorner => {
                    (TangentJoinClass::Corner, TangentAlignment::NotParallel)
                }
            };
            assert_eq!(join.tangent_join.endpoints_equal, Some(true));
            assert_eq!(join.tangent_join.class, class);
            assert_eq!(join.tangent_join.alignment, Some(alignment));
            let mut rows = vec![cap_row("lookahead local corner feed cap", v, self.global)];
            match self.join(j) {
                CornerLookaheadJoinClass::StraightThrough => {}
                CornerLookaheadJoinClass::RadiusLimitedCorner => rows.push(upper_row(
                    "lookahead local corner centripetal limit",
                    v - self.acceleration * self.radii[j],
                )),
                CornerLookaheadJoinClass::ReversalStop => rows.push((
                    "lookahead local reversal stop",
                    ConstraintKind::Equality,
                    Real::from(v).sqrt().unwrap(),
                    v == 0,
                )),
            }
            corners_ok &= check_rows(&join.certification, &rows);
        }
        assert_eq!(report.first_unsatisfied_span(), first_bad);
        assert_eq!(report.all_satisfied(), first_bad.is_none() && corners_ok);
    }
}

fn statuses(
    report: &LookaheadFeedScheduleReport,
) -> impl Iterator<Item = &CertifiedCandidateStatus> {
    report
        .spans
        .iter()
        .map(|span| &span.certification)
        .chain(report.corners.joins.iter().map(|join| &join.certification))
        .flat_map(|cert| cert.rows.iter().map(|row| &row.status))
}

fn check_speed(speed: &Real, sq: i64) {
    assert!(speed >= &Real::zero());
    assert_eq!(speed * speed, Real::from(sq));
}

type ExpectedRow = (&'static str, ConstraintKind, Real, bool);

fn upper_row(name: &'static str, excess: i64) -> ExpectedRow {
    (
        name,
        ConstraintKind::GreaterOrEqual,
        excess.into(),
        excess <= 0,
    )
}

fn cap_row(name: &'static str, squared_speed: i64, maximum: i64) -> ExpectedRow {
    (
        name,
        ConstraintKind::GreaterOrEqual,
        Real::from(squared_speed).sqrt().unwrap() - Real::from(maximum),
        squared_speed <= maximum.pow(2),
    )
}

fn check_rows(report: &CandidateCertificationReport, expected: &[ExpectedRow]) -> bool {
    let satisfied = expected.iter().filter(|row| row.3).count();
    assert_eq!(report.certified_satisfied_rows, satisfied);
    assert_eq!(report.certified_violation_rows, expected.len() - satisfied);
    assert_eq!(
        (
            report.bounded_unknown_rows,
            report.lossy_adapter_only_rows,
            report.domain_failure_rows
        ),
        (0, 0, 0)
    );
    assert_eq!(report.rows.len(), expected.len());
    for (i, (row, (name, kind, residual, ok))) in report.rows.iter().zip(expected).enumerate() {
        assert_eq!(row.constraint_index, i);
        assert_eq!(&row.name, name);
        assert_eq!(&row.kind, kind);
        // Equivalent radical values can have different expression trees.
        let difference = row.signed_residual.as_ref().unwrap() - residual;
        assert_eq!(
            difference.quadratic_tower_sign(),
            Some(RealSign::Zero),
            "{name}"
        );
        match row.status {
            CertifiedCandidateStatus::CertifiedZero { .. } => {
                assert!(*ok && *kind == ConstraintKind::Equality)
            }
            CertifiedCandidateStatus::CertifiedSatisfiedInequality { .. } => {
                assert!(*ok && *kind == ConstraintKind::GreaterOrEqual)
            }
            CertifiedCandidateStatus::CertifiedViolation { sign, .. } => {
                assert!(!ok);
                // Every violated inequality has positive excess. A nonzero
                // reversal speed is also positive on this input domain.
                assert_eq!(sign, RealSign::Positive);
            }
            ref other => panic!("unexpected replay status {other:?}"),
        }
    }
    satisfied == expected.len()
}

pub fn check(data: &[u8]) {
    let Some(mut path) = Path::decode(data) else {
        return;
    };
    let policy = if data[0] & 2 == 0 {
        PredicatePolicy::STRICT
    } else {
        PredicatePolicy::APPROXIMATE_512
    };
    let forward = path.check(data[0], policy);
    path.reverse();
    let mut backward = path.check(data[0], policy);
    backward.reverse();
    assert_eq!(forward, backward);
}
