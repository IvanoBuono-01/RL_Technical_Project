#!/bin/bash

# Script per caricare parametri controller solo quando controller_manager esiste

CONTROLLER_MANAGER="/iiwa/controller_manager"
YAML_FILE="$1"

echo "Waiting for $CONTROLLER_MANAGER to be available..."

# Aspetta fino a 30 secondi che il nodo esista
for i in {1..30}; do
    if ros2 node list | grep -q "$CONTROLLER_MANAGER"; then
        echo "Controller manager found! Loading parameters from $YAML_FILE"
        sleep 1  # Buffer di sicurezza
        ros2 param load "$CONTROLLER_MANAGER" "$YAML_FILE"
        exit 0
    fi
    echo "Attempt $i/30: Controller manager not ready yet..."
    sleep 1
done

echo "ERROR: Controller manager not found after 30 seconds"
exit 1
