#!/bin/bash

# Create a temporary virtual environment
cd "$(dirname "$0")"
python3 -m venv .venv
source .venv/bin/activate

# Install blobconverter
pip install blobconverter

# Download model
wget -O ewasr_resnet18.onnx https://github.com/tersekmatija/eWaSR/releases/download/0.1.0/ewasr_resnet18.onnx

# Convert to blob
python3 convert_model.py

# Verify .blob file exists
if [ -f "../config/ewasr_resnet18.blob" ]; then
    echo "Model conversion successful: ../config/ewasr_resnet18.blob created."
else
    echo "Model conversion failed."
fi

# Clean up
deactivate
rm -rf .venv ewasr_resnet18.onnx
