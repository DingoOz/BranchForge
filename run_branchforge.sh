#!/bin/bash

# BranchForge Launch Script
# Handles library path issues and launches the QML application

cd "$(dirname "$0")"

echo "Starting BranchForge..."
echo "Working directory: $(pwd)"

# Try different library path configurations
export LD_LIBRARY_PATH="/usr/lib/x86_64-linux-gnu:$LD_LIBRARY_PATH"

# Remove snap paths that might conflict
export PATH=$(echo $PATH | tr ':' '\n' | grep -v snap | tr '\n' ':')
export LD_LIBRARY_PATH=$(echo $LD_LIBRARY_PATH | tr ':' '\n' | grep -v snap | tr '\n' ':')

# Set Qt environment
export QT_QPA_PLATFORM=xcb
export QML_IMPORT_PATH=/usr/lib/x86_64-linux-gnu/qt6/qml

echo "Library path: $LD_LIBRARY_PATH"
echo "QML import path: $QML_IMPORT_PATH"

# Launch the application
./build/branchforge/branchforge_enhanced "$@"