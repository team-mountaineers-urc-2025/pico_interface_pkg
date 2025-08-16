#!/bin/bash
pip install mpremote

# Path to your new main.py file
NEW_MAIN_PY="pico_driver/main.py"

# Check if the new main.py file exists
if [ ! -f "$NEW_MAIN_PY" ]; then
    echo "Error: $NEW_MAIN_PY does not exist."
    exit 1
fi

# Upload the new main.py to the Pico
mpremote fs cp "$NEW_MAIN_PY" :main.py

# Optionally, you can reset the Pico after uploading
mpremote reset

echo "Successfully updated main.py on the Raspberry Pi Pico."
