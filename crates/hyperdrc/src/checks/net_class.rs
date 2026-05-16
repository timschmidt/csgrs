//! Net-class policy expansion helpers.
//!
//! The checks operate on flat `NetClassConfig` records, but rule decks benefit
//! from named base classes for repeated electrical and manufacturing policy.
//! Keeping inheritance resolution here follows Parnas' information-hiding
//! guidance from "On the Criteria To Be Used in Decomposing Systems into
//! Modules," Communications of the ACM, 1972, doi:10.1145/361598.361623: the
//! check implementations consume resolved policy without each one knowing how
//! inheritance is represented in JSON.

use std::collections::{BTreeMap, BTreeSet};

use crate::constraint_policy::NetClassConfig;
use crate::report::{Severity, Violation};

/// Resolved net-class policy plus non-fatal configuration diagnostics.
#[derive(Debug, Default)]
pub(super) struct NetClassResolution {
    /// Net classes after parent constraint fields have been applied.
    pub(super) classes: Vec<NetClassConfig>,
    /// Non-fatal rule-deck diagnostics found while resolving inheritance.
    pub(super) violations: Vec<Violation>,
}

/// Resolve `extends` relationships into flat net-class records.
///
/// A child inherits scalar constraint fields from listed parents only when the
/// child field is unset. Parent selectors are intentionally not inherited:
/// `nets`, `net_patterns`, and rectangular `regions` describe which copper a
/// concrete class owns, while parent classes can remain abstract policy bundles.
/// Missing parents, duplicate names, conflicting parent defaults, and
/// inheritance cycles are reported as warnings and skipped rather than failing
/// the whole readiness run.
pub(super) fn resolve_net_classes(net_classes: &[NetClassConfig]) -> NetClassResolution {
    let mut name_to_index = BTreeMap::<String, usize>::new();
    let mut violations = Vec::new();
    let mut duplicate_names = 0_usize;
    for (index, class) in net_classes.iter().enumerate() {
        let name = class.name.trim();
        if name.is_empty() {
            continue;
        }
        if name_to_index.insert(name.to_string(), index).is_some() {
            duplicate_names += 1;
            violations.push(net_class_violation(format!(
                "net class {name} is declared more than once; inheritance uses the last declaration"
            )));
        }
    }

    let mut context = ResolutionContext {
        net_classes,
        name_to_index,
        resolved: vec![None; net_classes.len()],
        stack: Vec::new(),
        reported_cycles: BTreeSet::new(),
        missing_parent_count: 0,
        cycle_count: 0,
        precedence_conflict_count: 0,
        violations,
    };

    let classes = (0..net_classes.len())
        .map(|index| context.resolve(index))
        .collect::<Vec<_>>();

    log::trace!(
        "net-class inheritance resolution: classes={} named={} extended={} duplicate_names={} missing_parents={} cycles={} precedence_conflicts={} violations={}",
        net_classes.len(),
        context.name_to_index.len(),
        net_classes
            .iter()
            .filter(|class| !class.extends.is_empty())
            .count(),
        duplicate_names,
        context.missing_parent_count,
        context.cycle_count,
        context.precedence_conflict_count,
        context.violations.len()
    );

    NetClassResolution {
        classes,
        violations: context.violations,
    }
}

struct ResolutionContext<'a> {
    net_classes: &'a [NetClassConfig],
    name_to_index: BTreeMap<String, usize>,
    resolved: Vec<Option<NetClassConfig>>,
    stack: Vec<usize>,
    reported_cycles: BTreeSet<String>,
    missing_parent_count: usize,
    cycle_count: usize,
    precedence_conflict_count: usize,
    violations: Vec<Violation>,
}

impl ResolutionContext<'_> {
    fn resolve(&mut self, index: usize) -> NetClassConfig {
        if let Some(resolved) = &self.resolved[index] {
            return resolved.clone();
        }

        if let Some(position) = self.stack.iter().position(|candidate| *candidate == index) {
            self.report_cycle(position, index);
            return self.net_classes[index].clone();
        }

        self.stack.push(index);
        let original = self.net_classes[index].clone();
        let mut resolved = self.net_classes[index].clone();
        for parent_name in &self.net_classes[index].extends {
            let parent_name = parent_name.trim();
            if parent_name.is_empty() {
                continue;
            }
            let Some(parent_index) = self.name_to_index.get(parent_name).copied() else {
                self.missing_parent_count += 1;
                self.violations.push(net_class_violation(format!(
                    "net class {} extends missing parent {parent_name}",
                    class_label(&self.net_classes[index])
                )));
                continue;
            };
            if self.stack.contains(&parent_index) {
                if let Some(position) = self
                    .stack
                    .iter()
                    .position(|candidate| *candidate == parent_index)
                {
                    self.report_cycle(position, parent_index);
                }
                continue;
            }
            let parent = self.resolve(parent_index);
            self.precedence_conflict_count +=
                report_precedence_conflicts(&original, &resolved, &parent, &mut self.violations);
            inherit_unset_constraints(&mut resolved, &parent);
        }
        self.stack.pop();
        self.resolved[index] = Some(resolved.clone());
        resolved
    }

    fn report_cycle(&mut self, position: usize, repeated_index: usize) {
        let mut names = self.stack[position..]
            .iter()
            .map(|index| class_label(&self.net_classes[*index]))
            .collect::<Vec<_>>();
        names.push(class_label(&self.net_classes[repeated_index]));
        let cycle = names.join(" -> ");
        if self.reported_cycles.insert(cycle.clone()) {
            self.cycle_count += 1;
            self.violations.push(net_class_violation(format!(
                "net class inheritance cycle detected: {cycle}"
            )));
        }
    }
}

fn report_precedence_conflicts(
    original_child: &NetClassConfig,
    current_child: &NetClassConfig,
    parent: &NetClassConfig,
    violations: &mut Vec<Violation>,
) -> usize {
    let mut count = 0_usize;
    let child_label = class_label(original_child);
    let parent_label = class_label(parent);

    macro_rules! conflict {
        ($field:ident) => {
            if original_child.$field.is_none() {
                let current_value = &current_child.$field;
                let parent_value = &parent.$field;
                if current_value.is_some()
                    && parent_value.is_some()
                    && current_value != parent_value
                {
                    count += 1;
                    violations.push(precedence_conflict_violation(
                        &child_label,
                        &parent_label,
                        stringify!($field),
                        &format!("{:?}", current_value),
                        &format!("{:?}", parent_value),
                    ));
                }
            }
        };
    }

    conflict!(min_width);
    conflict!(min_clearance);
    conflict!(max_layer_count);
    conflict!(min_via_count);
    conflict!(min_current_width);
    conflict!(min_voltage_clearance);
    conflict!(requires_reference_plane);
    conflict!(requires_impedance_control);
    conflict!(target_impedance_ohms);
    conflict!(impedance_tolerance_ohms);
    conflict!(differential_pair);
    conflict!(differential_role);
    conflict!(min_pair_spacing);
    conflict!(max_pair_spacing);
    conflict!(max_length);
    conflict!(max_pair_skew);
    conflict!(max_via_count);

    count
}

fn inherit_unset_constraints(child: &mut NetClassConfig, parent: &NetClassConfig) {
    child.min_width = child.min_width.or(parent.min_width);
    child.min_clearance = child.min_clearance.or(parent.min_clearance);
    child.max_layer_count = child.max_layer_count.or(parent.max_layer_count);
    child.min_via_count = child.min_via_count.or(parent.min_via_count);
    child.min_current_width = child.min_current_width.or(parent.min_current_width);
    child.min_voltage_clearance = child.min_voltage_clearance.or(parent.min_voltage_clearance);
    child.requires_reference_plane = child
        .requires_reference_plane
        .or(parent.requires_reference_plane);
    child.requires_impedance_control = child
        .requires_impedance_control
        .or(parent.requires_impedance_control);
    child.target_impedance_ohms = child.target_impedance_ohms.or(parent.target_impedance_ohms);
    child.impedance_tolerance_ohms = child
        .impedance_tolerance_ohms
        .or(parent.impedance_tolerance_ohms);
    child.differential_pair = child
        .differential_pair
        .clone()
        .or_else(|| parent.differential_pair.clone());
    child.differential_role = child.differential_role.or(parent.differential_role);
    child.min_pair_spacing = child.min_pair_spacing.or(parent.min_pair_spacing);
    child.max_pair_spacing = child.max_pair_spacing.or(parent.max_pair_spacing);
    child.max_length = child.max_length.or(parent.max_length);
    child.max_pair_skew = child.max_pair_skew.or(parent.max_pair_skew);
    child.max_via_count = child.max_via_count.or(parent.max_via_count);
}

fn precedence_conflict_violation(
    child: &str,
    parent: &str,
    field: &str,
    kept: &str,
    parent_value: &str,
) -> Violation {
    net_class_violation(format!(
        "net class {child} has conflicting inherited {field}; keeping earlier value {kept} instead of parent {parent} value {parent_value}"
    ))
}

fn net_class_violation(message: String) -> Violation {
    Violation::new(
        "net-constraint-readiness",
        Severity::Warning,
        vec!["net-class:config".to_string()],
        None,
        Vec::new(),
        Vec::new(),
        Some(message),
    )
}

fn class_label(class: &NetClassConfig) -> String {
    if class.name.trim().is_empty() {
        "unnamed".to_string()
    } else {
        class.name.trim().to_string()
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::constraint_policy::NetClassRegionConfig;

    #[test]
    fn child_inherits_unset_constraint_fields_without_parent_selectors() {
        let classes = vec![
            NetClassConfig {
                name: "base".to_string(),
                nets: vec!["SHOULD_NOT_MATCH".to_string()],
                regions: vec![NetClassRegionConfig {
                    name: "parent-region".to_string(),
                    min_x: Some(0.0),
                    min_y: Some(0.0),
                    max_x: Some(1.0),
                    max_y: Some(1.0),
                    ..NetClassRegionConfig::default()
                }],
                min_width: Some(0.25),
                min_clearance: Some(0.40),
                requires_reference_plane: Some(true),
                ..NetClassConfig::default()
            },
            NetClassConfig {
                name: "child".to_string(),
                extends: vec!["base".to_string()],
                nets: vec!["SIG".to_string()],
                min_width: Some(0.30),
                ..NetClassConfig::default()
            },
        ];

        let resolution = resolve_net_classes(&classes);

        assert!(resolution.violations.is_empty());
        assert_eq!(resolution.classes[1].nets, vec!["SIG"]);
        assert!(resolution.classes[1].regions.is_empty());
        assert_eq!(resolution.classes[1].min_width, Some(0.30));
        assert_eq!(resolution.classes[1].min_clearance, Some(0.40));
        assert_eq!(resolution.classes[1].requires_reference_plane, Some(true));
    }

    #[test]
    fn missing_parent_and_cycles_are_nonfatal_warnings() {
        let classes = vec![
            NetClassConfig {
                name: "a".to_string(),
                extends: vec!["b".to_string(), "missing".to_string()],
                min_width: Some(0.2),
                ..NetClassConfig::default()
            },
            NetClassConfig {
                name: "b".to_string(),
                extends: vec!["a".to_string()],
                min_clearance: Some(0.3),
                ..NetClassConfig::default()
            },
        ];

        let resolution = resolve_net_classes(&classes);
        let messages = resolution
            .violations
            .iter()
            .filter_map(|violation| violation.message.as_deref())
            .collect::<Vec<_>>();

        assert!(
            messages
                .iter()
                .any(|message| message.contains("missing parent"))
        );
        assert!(
            messages
                .iter()
                .any(|message| message.contains("inheritance cycle"))
        );
    }

    #[test]
    fn conflicting_parent_defaults_are_reported_with_first_parent_precedence() {
        let classes = vec![
            NetClassConfig {
                name: "slow".to_string(),
                min_width: Some(0.20),
                min_clearance: Some(0.30),
                ..NetClassConfig::default()
            },
            NetClassConfig {
                name: "fast".to_string(),
                min_width: Some(0.25),
                min_clearance: Some(0.30),
                ..NetClassConfig::default()
            },
            NetClassConfig {
                name: "child".to_string(),
                extends: vec!["slow".to_string(), "fast".to_string()],
                nets: vec!["SIG".to_string()],
                ..NetClassConfig::default()
            },
            NetClassConfig {
                name: "explicit-child".to_string(),
                extends: vec!["slow".to_string(), "fast".to_string()],
                nets: vec!["EXPLICIT".to_string()],
                min_width: Some(0.30),
                ..NetClassConfig::default()
            },
        ];

        let resolution = resolve_net_classes(&classes);
        let messages = resolution
            .violations
            .iter()
            .filter_map(|violation| violation.message.as_deref())
            .collect::<Vec<_>>();

        assert_eq!(resolution.classes[2].min_width, Some(0.20));
        assert_eq!(resolution.classes[3].min_width, Some(0.30));
        assert_eq!(
            messages
                .iter()
                .filter(|message| message.contains("conflicting inherited min_width"))
                .count(),
            1
        );
    }
}
