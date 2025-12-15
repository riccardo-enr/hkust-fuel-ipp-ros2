## Gemini Added Memories

To ensure consistency with the development environment and version control standards:

### Container Execution

**Important**

All the code should be executed inside the container.

- If executing from outside the container: Use `docker compose exec fuel-dev <command>` to execute commands inside the development container.
- If executing from inside the container (like this agent): Run the command directly.

When building ROS 2 packages, use the following command to enable compile commands export and symlink installation:
`colcon build --symlink-install --cmake-args -DCMAKE_EXPORT_COMPILE_COMMANDS=ON`

### Version Control
**Important:** Execute all `git` commands on the host machine (Ubuntu 24.04), not inside the container.

* **Commit Granularity:** Avoid large, monolithic commits. Group changes contextually into smaller, atomic commits that address specific tasks or fixes.
* **Format:** Strictly adhere to the **Conventional Commits** specification. Every commit message must include a scope modifier (e.g., `feat(mppi):`, `fix(ros2):`).
