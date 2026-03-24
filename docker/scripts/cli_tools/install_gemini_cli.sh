#!/bin/bash

set -e

echo "Start installing Gemini CLI..."

echo "[1/4] Installing curl, git, and certificates..."
sudo apt-get update
sudo apt-get install -y curl git ca-certificates

echo "[2/4] Setting up Node.js 20.x package source..."
curl -fsSL https://deb.nodesource.com/setup_20.x | sudo -E bash -

echo "[3/4] Installing Node.js..."
sudo apt-get install -y nodejs

echo "[4/4] Installing Gemini CLI..."
sudo npm install -g @google/gemini-cli

echo "Gemini CLI installation completed successfully!"