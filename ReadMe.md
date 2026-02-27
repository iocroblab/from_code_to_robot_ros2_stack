# From Code to Robot – ROS 2 Stack Setup

This repository provides a **reproducible environment** for the  
**From Code to Robot ROS 2 stack**, supporting both:

-  Docker-based workflow (recommended for teaching & reproducibility)
-  Native Ubuntu installation (for advanced users / development)

Both setups provide an **aligned ROS 2 Jazzy environment**.

---

## Overview

The installer supports four modes:

| Mode | Description |
|------|------------|
| `docker` | Build Docker image + create named container (stopped) |
| `fulldocker` | Install Docker Engine + docker mode |
| `native` | Workspace setup only (ROS already installed) |
| `fullnative` | Install ROS 2 Jazzy + dependencies + workspace |

---

## Docker Workflow (Recommended)

This workflow creates a **named container** that MATLAB can attach to.

### Install + Setup
```bash
bash install.sh fulldocker