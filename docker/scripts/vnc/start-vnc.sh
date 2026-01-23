#!/bin/bash
##############################################################################
# VNC Server Startup Script
# Starts VNC server with XFCE desktop environment
##############################################################################

set -e

echo "Starting VNC server..."

# Create .vnc directory if it doesn't exist
mkdir -p ~/.vnc

# Remove old .Xauthority to avoid conflicts
rm -f ~/.Xauthority ~/.Xauthority-*

# Set VNC password (default: ros)
VNC_PASSWORD=${VNC_PASSWORD:-ros}
echo "$VNC_PASSWORD" | vncpasswd -f > ~/.vnc/passwd
chmod 600 ~/.vnc/passwd

# Create xstartup script
cat > ~/.vnc/xstartup << 'EOF'
#!/bin/bash
unset SESSION_MANAGER
unset DBUS_SESSION_BUS_ADDRESS
export XDG_SESSION_TYPE=x11
export GDK_BACKEND=x11
exec dbus-launch startxfce4
EOF

chmod +x ~/.vnc/xstartup

# Kill any existing VNC server
vncserver -kill :2 2>/dev/null || true
sleep 1

# Clean up any leftover socket files (use sudo if needed)
sudo rm -rf /tmp/.X11-unix/X2 /tmp/.X2-lock 2>/dev/null || true
rm -rf ~/.vnc/*.pid 2>/dev/null || true
sleep 1

# Start VNC server
vncserver :2 -geometry 1920x1080 -depth 24 -localhost no

echo "VNC server started on :2 (port 5902)"
echo "Password: $VNC_PASSWORD"
echo "Connect with: <hostname>:5902"

# Keep container running
if [ $# -eq 0 ]; then
    echo "Tailing VNC logs..."
    tail -f ~/.vnc/*.log
else
    # Execute provided command
    exec "$@"
fi
