#[path = "support/supporting_line_archive.rs"]
mod support;

#[test]
fn supporting_line_archive_limits_and_forgery_recover() {
    support::supporting_line_archive_limits_and_forgery_recover();
}

#[test]
fn nested_supporting_line_archives_replay_with_record_limits() {
    support::nested_supporting_line_archives_replay_with_record_limits();
}
