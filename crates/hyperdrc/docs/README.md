# hyperdrc Docs

This folder contains project-level documentation and visual assets for
`hyperdrc`.

## Contents

- [`design-readiness-plan.md`](design-readiness-plan.md) is the long-form
  roadmap. It tracks implemented checks, supported inputs, remaining data-model
  gaps, researched DFM/DFA backlog items, fixture ideas, reporting plans, IO
  source and sink ideas, and research notes.
- [`testing.md`](testing.md) explains what the current test suites look for and
  how they exercise `hyperdrc` parsers, geometry, checks, reporting, CLI
  behavior, waivers, and conversion paths.
- [`hyperdrc.png`](hyperdrc.png) is the project image used by the root
  [README](../README.md).

## How This Folder Is Used

The root README stays focused on using and navigating `hyperdrc`. Details that
are too long for that overview belong here. When a design-readiness feature is
implemented, update `design-readiness-plan.md`; when tests are added or a module
changes ownership, update `testing.md` so coverage expectations stay accurate.

Return to the [repository README](../README.md).
