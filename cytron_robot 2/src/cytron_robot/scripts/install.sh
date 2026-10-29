#!/bin/bash
# ╔══════════════════════════════════════════════════════════════════════╗
# ║  CYTRON ROBOT — ONE-TIME INSTALL SCRIPT FOR RASPBERRY PI            ║
# ║  Run once after cloning the package:                                 ║
# ║    chmod +x scripts/install.sh && ./scripts/install.sh              ║
# ╚══════════════════════════════════════════════════════════════════════╝

set -e
echo ""
echo "════════════════════════════════════════"
echo "  Cytron Robot — RPi Install Script"
echo "════════════════════════════════════════"

# ── Detect ROS2 distro ─────────────────────────────────────────────────────
if [ -z "$ROS_DISTRO" ]; then
    echo "[ERROR] ROS2 is not sourced. Run:"
    echo "  source /opt/ros/humble/setup.bash"
    exit 1
fi
echo "[INFO] ROS2 distro: $ROS_DISTRO"

# ── System packages ────────────────────────────────────────────────────────
echo ""
echo "[STEP 1/5] Installing ROS2 Nav2, SLAM, and sensor packages..."
sudo apt-get update -qq
sudo apt-get install -y \
    ros-${ROS_DISTRO}-navigation2 \
    ros-${ROS_DISTRO}-nav2-bringup \
    ros-${ROS_DISTRO}-slam-toolbox \
    ros-${ROS_DISTRO}-rplidar-ros \
    ros-${ROS_DISTRO}-robot-state-publisher \
    ros-${ROS_DISTRO}-joint-state-publisher \
    ros-${ROS_DISTRO}-tf2-ros \
    ros-${ROS_DISTRO}-tf2-tools \
    ros-${ROS_DISTRO}-teleop-twist-keyboard \
    ros-${ROS_DISTRO}-nav2-map-server \
    ros-${ROS_DISTRO}-rviz2 \
    python3-pip \
    python3-colcon-common-extensions

echo "[OK] ROS2 packages installed"

# ── Python GPIO library ────────────────────────────────────────────────────
echo ""
echo "[STEP 2/5] Installing RPi.GPIO Python library..."
pip3 install RPi.GPIO --break-system-packages 2>/dev/null || \
pip3 install RPi.GPIO

echo "[OK] RPi.GPIO installed"

# ── Serial port permissions ────────────────────────────────────────────────
echo ""
echo "[STEP 3/5] Configuring serial port for RPLidar..."
sudo usermod -aG dialout $USER
echo "[OK] Added $USER to dialout group"
echo "     NOTE: Log out and back in (or reboot) for this to take effect"

# ── udev rule so RPLidar always appears as /dev/ttyUSB0 ───────────────────
echo ""
echo "[STEP 4/5] Creating udev rule for RPLidar A1..."
UDEV_RULE='KERNEL=="ttyUSB*", ATTRS{idVendor}=="10c4", ATTRS{idProduct}=="ea60", SYMLINK+="rplidar", MODE="0666"'
echo "$UDEV_RULE" | sudo tee /etc/udev/rules.d/99-rplidar.rules > /dev/null
sudo udevadm control --reload-rules
sudo udevadm trigger
echo "[OK] udev rule created — RPLidar will appear as /dev/rplidar"

# ── GPIO permission (no sudo required) ────────────────────────────────────
echo ""
echo "[STEP 5/5] Enabling GPIO access without sudo..."
sudo usermod -aG gpio $USER 2>/dev/null || true
# Also add mem group for hardware PWM
sudo usermod -aG kmem $USER 2>/dev/null || true
echo "[OK] GPIO group permissions set"

# ── Build workspace ────────────────────────────────────────────────────────
echo ""
echo "[BUILD] Building workspace..."
SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
WS_DIR="$(realpath "$SCRIPT_DIR/../../../../..")"
echo "       Workspace: $WS_DIR"

cd "$WS_DIR"
colcon build --packages-select cytron_robot --symlink-install

echo ""
echo "════════════════════════════════════════"
echo "  Install complete!"
echo ""
echo "  Add this to your ~/.bashrc:"
echo "    source /opt/ros/${ROS_DISTRO}/setup.bash"
echo "    source $WS_DIR/install/setup.bash"
echo ""
echo "  Then REBOOT or log out/in for GPIO/serial permissions."
echo "════════════════════════════════════════"
