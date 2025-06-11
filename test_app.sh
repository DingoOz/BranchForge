#!/bin/bash
export PATH=$(echo $PATH | tr ':' '\n' | grep -v snap | tr '\n' ':')
export LD_LIBRARY_PATH="/usr/lib/x86_64-linux-gnu"
export QML_IMPORT_PATH="/usr/lib/x86_64-linux-gnu/qt6/qml"

echo "Testing BranchForge QML application..."
echo "===================="

cd /home/dingo/Programming/BranchForge
timeout 10s ./build/branchforge/branchforge_enhanced 2>&1 | head -30