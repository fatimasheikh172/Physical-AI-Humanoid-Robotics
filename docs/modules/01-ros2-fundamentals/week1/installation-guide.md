# Ubuntu Installation Guide

## Prerequisites

Before installing Ubuntu 22.04 LTS, ensure your system meets the following requirements:

- At least 8GB RAM
- At least 50GB of free disk space
- Multi-core processor
- Internet connection for package installation

## Installation Options

You have several options for setting up Ubuntu 22.04 LTS:

### Option 1: Dual Boot with Existing OS
1. Download Ubuntu 22.04 LTS ISO from https://ubuntu.com/download/desktop
2. Create a bootable USB drive using Rufus (Windows) or Etcher (Cross-platform)
3. Boot from the USB drive and follow installation prompts
4. When partitioning, allocate at least 50GB for Ubuntu
5. Complete the installation and set up your user account

### Option 2: Virtual Machine
1. Install virtualization software (VirtualBox, VMware, or Hyper-V)
2. Create a new VM with at least 4 cores, 8GB RAM, and 50GB disk space
3. Mount the Ubuntu 22.04 ISO and start the VM
4. Follow the installation process within the VM

### Option 3: Windows Subsystem for Linux (WSL2)
1. Ensure you're running Windows 10 version 2004 or later, or Windows 11
2. Open PowerShell as Administrator and run:
   ```bash
   wsl --install -d Ubuntu-22.04
   ```
3. Restart your computer when prompted
4. Launch Ubuntu from the Start menu and set up your user account

## Post-Installation Setup

After installation, run the following commands to update your system:

```bash
sudo apt update && sudo apt upgrade -y
```

Install basic development tools:

```bash
sudo apt install build-essential git curl wget vim
```