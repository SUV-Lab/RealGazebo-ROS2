#!/bin/bash
#
# Stop RealGazebo Multi-Container Simulation
#
# Usage:
#   ./stop_simulation.sh [options]
#
# Options:
#   --volumes, -v   Also remove volumes (clears generated models)
#   --all, -a       Remove all containers, networks, and volumes
#

set -e

SCRIPT_DIR="$(cd "$(dirname "$0")" && pwd)"
PROJECT_DIR="$(dirname "$SCRIPT_DIR")"

REMOVE_VOLUMES=false
REMOVE_ALL=false

# Parse arguments
while [[ $# -gt 0 ]]; do
    case $1 in
        --volumes|-v)
            REMOVE_VOLUMES=true
            shift
            ;;
        --all|-a)
            REMOVE_ALL=true
            shift
            ;;
        --help|-h)
            head -15 "$0" | tail -12
            exit 0
            ;;
        *)
            echo "Unknown option: $1"
            exit 1
            ;;
    esac
done

cd "$PROJECT_DIR"

echo "Stopping RealGazebo simulation..."

if [[ "$REMOVE_ALL" == "true" ]]; then
    echo "Removing all containers, networks, and volumes..."
    docker compose down -v --remove-orphans
elif [[ "$REMOVE_VOLUMES" == "true" ]]; then
    echo "Removing containers and volumes..."
    docker compose down -v
else
    echo "Stopping containers..."
    docker compose down --remove-orphans
fi

echo ""
echo "Simulation stopped."
