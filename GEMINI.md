## Gemini Added Memories

To ensure consistency with the development environment and version control standards:

### Container Execution

**Important**

All the code should be executed inside the container.

Identify the correct Docker container name by inspecting the `.zed/tasks.json` file in the workspace root. Prioritize executing general code and shell commands inside this container.

### Version Control
**Important:** Execute all `git` commands on the host machine (Ubuntu 24.04), not inside the container.

* **Commit Granularity:** Avoid large, monolithic commits. Group changes contextually into smaller, atomic commits that address specific tasks or fixes.
* **Format:** Strictly adhere to the **Conventional Commits** specification. Every commit message must include a scope modifier (e.g., `feat(mppi):`, `fix(ros2):`).
