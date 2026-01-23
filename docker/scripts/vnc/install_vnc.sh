#!/bin/bash
##############################################################################
# VNC and Desktop Environment Installation Script
# This script installs TigerVNC, XFCE desktop, and necessary components
##############################################################################

set -e

echo "Installing VNC and desktop environment..."

export DEBIAN_FRONTEND=noninteractive

# Install VNC and XFCE Desktop
apt-get update && \
apt-get install -y \
    tigervnc-standalone-server \
    xterm \
    dbus-x11 \
    xfce4 \
    xfce4-terminal \
    x11-xserver-utils \
&& rm -rf /var/lib/apt/lists/*

echo "VNC installation complete!"
