# Separate compilation from the unchanged all-features test workload

Candidate validation v4's complete default curve suite passes. Its all-features
Cargo command exits 124 at the external 1,800-second limit, during the
arc-fragment property tests, without a recorded assertion failure. Compilation
alone takes 7 minutes 57 seconds in that command. This is an incomplete
qualification attempt, not a passing suite or evidence of faster geometry.
Its source, target, executables, output and resource records remain frozen.

Complete all remaining v4 checks first. Then create a new target for the same
frozen candidate-source-v4 and dependency lock. Run the complete all-features
test build with `--no-run`, followed by the complete original all-features test
command. Each command keeps the existing 1,800-second external timeout and
8 GiB address-space containment. Do not reopen a completed Cargo target.

This separates build work from test execution; it does not reduce tests, change
policies or generated-case configuration, accept a new Unknown, or remove a
fixture. The 120-second full-board worker CPU and 60-second copper-neck check
body limits are unchanged. Record the initial timeout alongside the replacement
complete test result. Source bytes are unchanged, so independently completed
v4 checks remain evidence for those exact inputs. No whole-v4-success claim is
permitted. Qualification remains incomplete until every affected command has
a complete successful observation under its declared contract.
