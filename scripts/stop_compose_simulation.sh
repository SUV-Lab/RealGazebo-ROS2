#!/bin/bash
# Stop the RealGazebo multi-container simulation and clean up.
#
# `docker compose down` stops the gazebo/manager container (the manager's
# shutdown teardown despawns its vehicles first). Any vehicle containers
# left over from an unclean exit are removed as well.
set -uo pipefail
cd "$(dirname "$0")/.."

docker compose -f docker-compose.yml -f docker-compose.dev.yml down --timeout 20 2>/dev/null \
    || docker compose down --timeout 20

# Sweep stray vehicle containers (e.g. after a SIGKILLed manager)
STRAY=$(docker ps -a --format '{{.Names}}' | grep -E '^vehicle_[0-9]+$' || true)
if [[ -n "$STRAY" ]]; then
    echo "Removing stray vehicle containers: $STRAY"
    echo "$STRAY" | xargs docker rm -f >/dev/null
fi
echo "Simulation stopped."
