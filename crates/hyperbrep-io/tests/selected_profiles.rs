#[path = "support/selected_profiles.rs"]
mod acceptance;
#[path = "../../hyperbrep/tests/support/selected_profile_source.rs"]
mod source;

#[test]
fn selected_profile_step_vertices_remain_explicitly_unencodable() {
    acceptance::selected_profile_step_vertices_remain_explicitly_unencodable(
        source::profile,
        source::replay,
    );
}
