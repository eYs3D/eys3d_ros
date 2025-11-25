#!/bin/bash
# Script to install NVIDIA Container Toolkit for Docker GPU support
# Run this script with: sudo ./install-nvidia-docker.sh

set -e

echo "=== Installing NVIDIA Container Toolkit ==="
echo ""

# Step 1: Add GPG key
echo "[1/5] Adding NVIDIA GPG key..."
curl -fsSL https://nvidia.github.io/libnvidia-container/gpgkey | gpg --dearmor -o /usr/share/keyrings/nvidia-container-toolkit-keyring.gpg

# Step 2: Add repository
echo "[2/5] Adding NVIDIA repository..."
curl -s -L https://nvidia.github.io/libnvidia-container/stable/deb/nvidia-container-toolkit.list | \
    sed 's#deb https://#deb [signed-by=/usr/share/keyrings/nvidia-container-toolkit-keyring.gpg] https://#g' | \
    tee /etc/apt/sources.list.d/nvidia-container-toolkit.list

# Step 3: Update and install
echo "[3/5] Installing nvidia-container-toolkit..."
apt-get update
apt-get install -y nvidia-container-toolkit

# Step 4: Configure Docker runtime
echo "[4/5] Configuring Docker runtime..."
nvidia-ctk runtime configure --runtime=docker

# Step 5: Restart Docker
echo "[5/5] Restarting Docker service..."
systemctl restart docker

echo ""
echo "=== Installation Complete ==="
echo ""
echo "Verify with: docker run --rm --gpus all nvidia/cuda:11.0.3-base-ubuntu20.04 nvidia-smi"
echo ""
echo "Then run the container with GPU support:"
echo "  ./docker-run.sh"
