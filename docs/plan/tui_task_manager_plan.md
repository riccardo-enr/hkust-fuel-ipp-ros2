# Terminal Task Manager Planning Guide

This document outlines the design and implementation plan for a keyboard-first terminal task manager. It follows a phased approach from requirements through packaging to ensure the tool is discoverable, maintainable, and pleasant to use daily.

## 1. Requirements and User Flows
- **Core actions**: view tasks for today and the current week, navigate to arbitrary dates, add/edit/delete tasks, mark tasks done/undone, search tasks, and filter by status/priority/tag.
- **Navigation constraints**: keyboard-only operation with clear, discoverable shortcuts; arrow keys or `j/k` for movement; `Tab` to cycle panes; `Esc` to exit dialogs; command palette for fallback.
- **Display constraints**: support true-color and reduced-color terminals; responsive layout that adapts to narrow terminals by stacking panes; clear focus highlighting for accessibility.
- **Layout preferences**: persistent sidebar for date/filter context, main list for tasks, detail pane for contextual info, and a status bar for hints and sync state.

## 2. TUI Toolkit Choice
- **Selected library**: [`textual`](https://github.com/Textualize/textual) (Python).
- **Rationale**: provides composable panes, async event loop, timers for clocks/reminders, mouse support for optional interactions, reactive layout for resizing, built-in theming, and a mature widget ecosystem. Alternatives (`rich` + `textual` core or Rust `ratatui`) were considered, but `textual` gives the best balance between rapid iteration and accessibility.

## 3. Layout and State Model
- **Screens**:
  - **Agenda view** (default): split layout with sidebar calendar/filter chips, main task list, and detail pane.
  - **Detail view**: expands selected task with notes, tags, recurrence, and quick actions.
  - **Dialogs/popups**: new/edit task form, delete confirmation, shortcut/help overlay, search palette.
- **Data structures**:
  - `Task`: `{id: uuid, title: str, due: date | datetime, priority: enum(low/med/high/urgent), status: enum(open/done), tags: [str], notes: str, recurrence: optional rule, created_at, updated_at}`.
  - `ViewState`: `{selected_date, visible_range (day/week), focused_pane, selected_task_id, active_filter, search_query, sort_order}`.
  - `AppState`: caches tasks in memory, exposes derived views (today/week/overdue/tag filters), tracks pending mutations for optimistic UI.
- **Persistence**: SQLite for default durability; JSON import/export for portability. In-memory backend available for testing.

## 4. Routing and Components
- **Root app**: orchestrates screens and global keybindings; owns router for overlays.
- **Sidebar**: mini month calendar or week strip, quick filters (Today, Week, Overdue, Priority, Tag), and sync indicator.
- **Main list**: virtualized list of tasks for current filter; sortable by due date, priority, or title; inline markers for status and tags.
- **Detail pane**: shows full notes, recurrence info, and contextual actions (edit, toggle done, delete, schedule change).
- **Status bar**: shows focus hints, active filter, pending sync/last saved, and current time.

## 5. Input Handling
- **Navigation**: `j/k` or arrows to move list focus; `h/l` or `Tab/Shift+Tab` to change panes; `Ctrl+f/b` for page scroll.
- **Actions**: `Enter` to open/edit, `a` to add, `e` to edit, `d` to delete (with confirmation), `x` or `Space` to toggle done, `s` to cycle sort, `f` to cycle filters, `/` to search, `?` to open help.
- **Command palette**: `:` opens palette with fuzzy match on commands and shortcuts; supports command aliases (e.g., `today`, `week`, `overdue`).
- **Accessibility**: consistent focus ring, hints in status bar, and escape hatch (`Esc`) to close overlays.

## 6. CRUD Flows
- **Add/edit**: modal form with fields for title (required), due date (defaults to today), priority (defaults to medium), tags, recurrence, and notes; validation on required fields and date parsing; keyboard shortcuts for saving (`Ctrl+Enter`) or canceling (`Esc`).
- **Delete**: confirmation dialog showing task title and due date; destructive action requires explicit confirmation key (`y` or `Enter`).
- **Toggle done**: immediate optimistic update with status change in list and detail pane; auto-updates status bar counts.
- **Recurrence**: simple rules (daily/weekly/monthly or custom weekday sets); completion spawns next occurrence.

## 7. Persistence and Sync
- **Storage layer**: SQLite schema with `tasks` table (id, title, due, priority, status, tags as JSON/text, notes, recurrence rule, created_at, updated_at, deleted_at for soft deletes).
- **Migrations**: lightweight migration runner to evolve schema; startup checks and automatic migrations with backups.
- **Import/export**: JSON for backup/restore; ICS export optional for calendar interoperability.
- **Sync**: placeholder hook to integrate remote sync later; tracks dirty records and shows sync status in status bar.

## 8. Search and Filtering
- **Filters**: quick chips for Today, This Week, Overdue, Priority (high/urgent), Tag, Status (open/done), and text search.
- **Search**: fuzzy search on title and notes; scoped search within current filter by default with option to widen to all tasks.
- **Sorting**: toggle sort by due date (asc/desc), priority, or title; remember user preference.

## 9. UX Polish
- **Themes**: light and dark palettes with semantic colors for priority and status; degraded colors for low-color terminals.
- **Time display**: human-friendly due labels (e.g., "in 2h", "tomorrow", "overdue by 1d"); optional live clock in status bar.
- **Resilience**: responsive layout on resize, scroll position preservation per filter, undo snackbar after destructive actions.
- **Keyboard help**: contextual hints in status bar plus full cheat sheet overlay.

## 10. Testing Strategy
- **Unit tests**: storage layer CRUD and migrations, parser for dates/recurrence, search/filter logic, and optimistic update reconciliation.
- **Integration tests**: scripted keystroke flows for add/edit/delete/toggle/search; snapshot tests of key views if `textual` supports.
- **Reliability**: regression suite for recurrence generation and import/export round-trips.

## 11. Packaging and Documentation
- **CLI entrypoint**: `task-tui` console script that launches the app with optional path to database/config.
- **Configuration**: TOML/JSON config for keymap overrides, theme selection, data path, and sync toggles; sample config checked in.
- **Docs**: README section with install/run instructions, keymap cheat sheet, troubleshooting tips, and storage location notes.
- **Distribution**: packaged as a Python project with lockfile; release artifacts published to PyPI when ready.
