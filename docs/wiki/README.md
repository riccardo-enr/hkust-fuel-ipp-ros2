# Wiki Overview

This directory mirrors the structure we intend to publish to the external wiki. Each subfolder should contain:
- an `README.md` describing the scope for that section;
- focused topic pages (one topic per file) with practical steps, commands, and diagrams as needed;
- cross-links back to `docs/porting/PORTING_PROGRESS.md` or other source-of-truth files instead of duplicating data.

## Section Map
1. `overview/` – project mission, architecture diagrams, roadmap/status snapshots.
2. `setup/` – how-to guides for local dev, containers, CI tuning, and launch commands.
3. `migration/` – ROS 1→2 strategy, porting checklist, API change logs.
4. `simulator/` – instructions for running the UAV simulator, sample environments, and visualization tips.
5. `troubleshooting/` – FAQs, debugging flows, gotchas collected from Slack/GitHub issues.
6. `reference/` – parameters, message schemas, CLI tables.
7. `templates/` – doc templates so contributors can copy/paste the right structure.

> Tip: Keep Markdown titles short. Where diagrams are needed, store the asset under `files/` and reference it relatively.
