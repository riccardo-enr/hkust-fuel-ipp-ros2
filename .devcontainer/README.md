# FUEL ROS 2 Jazzy Development Container

This devcontainer provides a complete development environment for FUEL with ROS 2 Jazzy.

## Features

- **Base Image**: ROS 2 Jazzy Desktop Full (Ubuntu 24.04)
- **Pre-installed Dependencies**:
  - NLopt v2.7.1 (as specified in project README)
  - Armadillo (linear algebra library)
  - PCL (Point Cloud Library)
  - Eigen3
  - All required ROS 2 packages (tf2, visualization, sensor/nav messages, etc.)
  - colcon build tools
  - rosdep

- **VS Code Extensions**:
  - C/C++ Tools
  - CMake Tools
  - Python
  - ROS extension
  - XML and YAML support
  - Code Spell Checker

- **Convenient Aliases**:
  - `cb` - Build workspace with symlink install
  - `ct` - Run colcon tests
  - `cbp <package>` - Build specific package
  - `cbs <package>` - Build package and dependencies
  - `cclean` - Clean build artifacts

## Usage

### Opening in VS Code

1. Install the [Dev Containers extension](https://marketplace.visualstudio.com/items?itemName=ms-vscode-remote.remote-containers)
2. Open this repository in VS Code
3. Click on the green button in the bottom-left corner
4. Select "Reopen in Container"

The first time you open the container, it will:
- Pull the ROS 2 Jazzy Docker image
- Install all dependencies
- Build and install NLopt v2.7.1
- Set up your development environment

This may take 10-15 minutes on the first run.

### Building the Workspace

After the container is ready:

```bash
# Build all packages
colcon build --symlink-install

# Or use the alias
cb

# Build a specific package
colcon build --symlink-install --packages-select <package_name>

# Or use the alias
cbp <package_name>
```

### Running with GUI (RViz, etc.)

The devcontainer is configured to forward X11 for GUI applications:

**On Linux:**
```bash
# Allow X server connections (run on host before starting container)
xhost +local:docker
```

**On Windows/Mac:**
Install an X server like:
- Windows: [VcXsrv](https://sourceforge.net/projects/vcxsrv/)
- Mac: [XQuartz](https://www.xquartz.org/)

### Sourcing the Workspace

After building, source the workspace overlay:

```bash
source install/setup.bash
```

Or uncomment the line in `~/.bashrc` to source automatically:
```bash
# In the container's terminal
nano ~/.bashrc
# Uncomment: # source /workspace/install/setup.bash
```

## Project Structure

The workspace root (`/workspace`) maps to your local repository directory. All changes made in the container are reflected in your local filesystem.

## Troubleshooting

### GUI Applications Don't Display

1. **Linux**: Make sure you ran `xhost +local:docker` on the host
2. **Windows/Mac**: Ensure your X server is running and DISPLAY is set correctly

### Build Errors

1. Clean the workspace: `cclean` or `rm -rf build install log`
2. Source ROS 2: `source /opt/ros/jazzy/setup.bash`
3. Rebuild: `colcon build --symlink-install`

### Permission Issues

The devcontainer runs as user `vscode` (UID 1000). If you encounter permission issues, check file ownership:

```bash
sudo chown -R vscode:vscode /workspace
```

## Customization

You can customize the devcontainer by editing:
- `.devcontainer/devcontainer.json` - Container configuration
- `.devcontainer/setup.sh` - Post-create setup script

## Additional Resources

- [ROS 2 Jazzy Documentation](https://docs.ros.org/en/jazzy/)
- [colcon Documentation](https://colcon.readthedocs.io/)
- [VS Code Dev Containers](https://code.visualstudio.com/docs/devcontainers/containers)
