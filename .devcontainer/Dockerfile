# Stage 0: base
# Contains core ROS 2 environment, basic build tools, and common libraries.
FROM osrf/ros:jazzy-desktop-full AS base

ARG CUDA_CAPABILITY
ENV DEBIAN_FRONTEND=noninteractive

# Install common build tools, libraries, and ROS 2 packages
RUN apt-get update && apt-get install -y \
  build-essential \
  cmake \
  git \
  # Required libraries for FUEL
  libarmadillo-dev \
  libpcl-dev \
  libeigen3-dev \
  # ROS 2 packages
  ros-dev-tools \
  ros-jazzy-tf2-geometry-msgs \
  ros-jazzy-tf2-eigen \
  ros-jazzy-visualization-msgs \
  ros-jazzy-sensor-msgs \
  ros-jazzy-nav-msgs \
  ros-jazzy-geometry-msgs \
  ros-jazzy-pcl-conversions \
  ros-jazzy-pcl-ros \
  ros-jazzy-rviz2 \
  ros-jazzy-rqt \
  ros-jazzy-rqt-common-plugins \
  ros-jazzy-plotjuggler-ros \
  python3-colcon-common-extensions \
  python3-colcon-mixin \
  python3-rosdep \
  python3-pip \
  # Other utilities
  sudo \
  curl \
  && rm -rf /var/lib/apt/lists/*

RUN if [ "${CUDA_CAPABILITY}" != "OFF" ]; then \
  apt-get update && apt-get install -y nvidia-cuda-toolkit && rm -rf /var/lib/apt/lists/*; \
  fi

# Install NLopt v2.7.1
WORKDIR /tmp
RUN git clone -b v2.7.1 https://github.com/stevengj/nlopt.git && \
  cd nlopt && \
  mkdir build && cd build && \
  cmake .. && \
  make -j$(nproc) && \
  make install && \
  ldconfig && \
  rm -rf /tmp/nlopt

# Create the user (ubuntu is default for this base image)
ARG USERNAME=ubuntu
ARG USER_UID=1000
ARG USER_GID=$USER_UID

RUN if ! getent group $USER_GID > /dev/null 2>&1; then \
  groupadd --gid $USER_GID $USERNAME; \
  fi && \
  if ! getent passwd $USER_UID > /dev/null 2>&1; then \
  useradd --uid $USER_UID --gid $USER_GID -m $USERNAME; \
  fi && \
  echo "$USERNAME ALL=(root) NOPASSWD:ALL" > /etc/sudoers.d/$USERNAME \
  && chmod 0440 /etc/sudoers.d/$USERNAME \
  && usermod -aG sudo $USERNAME

# Ensure home directory permissions for potential mounts
RUN mkdir -p /home/$USERNAME/.config/nvim \
  /home/$USERNAME/.local/share/nvim \
  /home/$USERNAME/.local/state/nvim \
  /home/$USERNAME/.ssh \
  && chown -R $USERNAME:$USER_GID /home/$USERNAME

# Source ROS 2 and the workspace for interactive shells
RUN for shell_rc in /root/.bashrc /home/$USERNAME/.bashrc; do \
  echo "source /opt/ros/jazzy/setup.bash" >> ${shell_rc}; \
  echo "if [ -f /workspace/install/setup.bash ]; then source /workspace/install/setup.bash; fi" >> ${shell_rc}; \
  done

# Also source ROS 2 in zshrc since we're using zsh
RUN for shell_rc in /root/.zshrc /home/$USERNAME/.zshrc; do \
  echo "source /opt/ros/jazzy/setup.zsh" >> ${shell_rc}; \
  echo "if [ -f /workspace/install/setup.zsh ]; then source /workspace/install/setup.zsh; fi" >> ${shell_rc}; \
  done

# RUN chsh -s $(which zsh) $USERNAME
# ENV SHELL=/usr/bin/zsh

# Initialize rosdep
RUN rosdep init || true && rosdep update

# ==============================================================================

# Stage 1: dev
FROM base AS dev

ARG USERNAME=ubuntu
ENV DEBIAN_FRONTEND=noninteractive

# Install developer-specific tools
RUN apt-get update && apt-get install -y \
  gdb \
  vim \
  nano \
  openssh-client \
  less \
  ripgrep \
  xclip \
  clangd \
  zsh \
  fzf \
  && rm -rf /var/lib/apt/lists/*

# Install Oh My Zsh for the user
USER $USERNAME
RUN sh -c "$(curl -fsSL https://raw.githubusercontent.com/ohmyzsh/ohmyzsh/master/tools/install.sh)" --unattended
RUN curl -sS https://raw.githubusercontent.com/ajeetdsouza/zoxide/main/install.sh | sh
# Install fzf (user-scoped) with shell key-bindings and completion, without modifying rc files
RUN git clone --depth 1 https://github.com/junegunn/fzf.git ~/.fzf && \
  ~/.fzf/install --all --no-update-rc
USER root

# FIX: Use usermod instead of chsh to avoid PAM/Password errors in Ubuntu 24.04
RUN usermod --shell /usr/bin/zsh $USERNAME
ENV SHELL=/usr/bin/zsh

# Install Node.js
RUN curl -fsSL https://deb.nodesource.com/setup_22.x | bash - && \
  apt-get install -y nodejs && \
  npm install -g @openai/codex && \
  npm install -g @google/gemini-cli && \
  npm install -g @anthropic-ai/claude-code && \
  npm install -g opencode-ai && \
  npm install -g markdownlint-cli2 && \
  rm -rf /var/lib/apt/lists/*

# Install Python-based documentation tools
RUN pip install --break-system-packages \
  mkdocs \
  mkdocs-material \
  pymdown-extensions

# Install Neovim (x86_64 for your workstation)
RUN curl -LO https://github.com/neovim/neovim/releases/latest/download/nvim-linux-x86_64.tar.gz \
  && tar xzvf nvim-linux-x86_64.tar.gz \
  && mv nvim-linux-x86_64 /opt/nvim \
  && ln -s /opt/nvim/bin/nvim /usr/local/bin/nvim \
  && rm nvim-linux-x86_64.tar.gz

# Install Lazygit (x86_64 for your workstation)
RUN LAZYGIT_VERSION=$(curl -s "https://api.github.com/repos/jesseduffield/lazygit/releases/latest" | grep -Po '"tag_name": "v\K[^" ]*') \
  && curl -Lo lazygit.tar.gz "https://github.com/jesseduffield/lazygit/releases/latest/download/lazygit_${LAZYGIT_VERSION}_Linux_x86_64.tar.gz" \
  && tar xf lazygit.tar.gz -C /usr/local/bin \
  && rm lazygit.tar.gz

# Install GitHub CLI
RUN mkdir -p -m 755 /etc/apt/keyrings && \
  curl -fsSL https://cli.github.com/packages/githubcli-archive-keyring.gpg | tee /etc/apt/keyrings/githubcli-archive-keyring.gpg > /dev/null && \
  chmod go+r /etc/apt/keyrings/githubcli-archive-keyring.gpg && \
  echo "deb [arch=$(dpkg --print-architecture) signed-by=/etc/apt/keyrings/githubcli-archive-keyring.gpg] https://cli.github.com/packages stable main" | tee /etc/apt/sources.list.d/github-cli.list > /dev/null && \
  apt-get update && \
  apt-get install -y gh && \
  rm -rf /var/lib/apt/lists/*

ARG GITHUB_USER_EMAIL
ARG GITHUB_USER_NAME
RUN git config --global user.email "$GITHUB_USER_EMAIL" && \
  git config --global user.name "$GITHUB_USER_NAME"

WORKDIR /workspace

ENV DEBIAN_FRONTEND=dialog

COPY entrypoint.sh /usr/local/bin/entrypoint.sh
ENTRYPOINT ["/usr/local/bin/entrypoint.sh"]

CMD ["/usr/bin/zsh"]

# ==============================================================================

# Stage 2: runtime
# Optimized for running the application, minimal dependencies.
FROM base AS runtime

WORKDIR /workspace

ENV DEBIAN_FRONTEND=dialog

COPY entrypoint.sh /usr/local/bin/entrypoint.sh
ENTRYPOINT ["/usr/local/bin/entrypoint.sh"]

CMD ["/bin/bash"]
