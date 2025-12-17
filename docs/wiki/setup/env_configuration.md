# Environment Configuration (.devcontainer/.env)

The `.env` file for Docker Compose configuration resides in the `.devcontainer/` directory.

## Setup

1.  Copy the example file:
    ```bash
    cp .devcontainer/.env.example .devcontainer/.env
    ```
2.  Edit `.devcontainer/.env` with your personal settings.

## Variables

| Variable | Description |
|----------|-------------|
| `GITHUB_USER` | Your GitHub username. |
| `GITHUB_NAME` | Your full name for Git commits. |
| `GH_TOKEN` | GitHub Personal Access Token (for cloning private repos or API limits). |
| `CUDA_CAPABILITIES` | Set to `ON` if you have an NVIDIA GPU, `OFF` otherwise. |

## Security

*   The `.devcontainer/.env` file is ignored by Git.
*   Do NOT commit it.
