# Documentation Index

This folder houses the in-repo wiki plus working notes. Use the structure below when adding or updating docs so contributors can find the right entry point quickly.

## Folder Layout
- `wiki/` – user-facing wiki content meant to mirror the future external knowledge base.
  - `overview/` – high-level context (mission, architecture, roadmap).
  - `setup/` – environment/bootstrap instructions for devs, CI, and simulators.
  - `migration/` – ROS 1→2 guides, checklists, and deprecation trackers.
  - `simulator/` – UAV simulator operations, scenarios, sample worlds.
  - `troubleshooting/` – FAQs, known issues, and debug recipes.
  - `reference/` – APIs, message schemas, parameter tables.
  - `templates/` – reusable doc templates (RFC, runbook, etc.).
- `plan/` – step-by-step porting or feature plans. Keep historical plans here.
- `porting/PORTING_PROGRESS.md` – canonical migration tracker; update whenever migration status changes.
- `codex/` – meta notes for Codex agents (conversation history, prompts).

## How To Contribute
1. Pick the right folder above; avoid dumping mixed content in the root.
2. Create a new Markdown file (lowercase-with-dashes) and link it from the nearest `README.md`.
3. When documenting porting progress, update both the relevant wiki page under `wiki/migration/` and `porting/PORTING_PROGRESS.md`.
4. Keep sections short with tables or lists where possible; favor ROS 2 terminology.
5. Mention verification commands (e.g., `colcon build --packages-select ...`) whenever applicable so others can reproduce steps.
