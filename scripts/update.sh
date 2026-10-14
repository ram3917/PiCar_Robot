#!/usr/bin/env bash
# Pulls the PiCar image and restarts the container. Run on the Pi.
#
#   ./scripts/update.sh                    # latest image
#   IMAGE_TAG=v0.1.0 ./scripts/update.sh   # roll back to a tagged release
#
# Safe to run again: with nothing new to pull, it changes nothing.
set -euo pipefail

die() {
    echo "update.sh: error: $*" >&2
    exit 1
}

command -v docker >/dev/null 2>&1 || die "docker not found; see docs/deployment.md"
docker compose version >/dev/null 2>&1 || die "the docker compose plugin is missing; see docs/deployment.md"

cd "$(dirname "$0")/../docker"
export IMAGE_TAG="${IMAGE_TAG:-latest}"

echo "Pulling ghcr.io/ram3917/picar_robot:${IMAGE_TAG}"
docker compose pull
docker compose up -d --remove-orphans

container="$(docker compose ps -q picar)"
[ -n "$container" ] || die "the picar container was not created; see: docker compose logs picar"

state="$(docker inspect --format '{{.State.Status}}' "$container")"
[ "$state" = "running" ] || die "the picar container is ${state}, not running; see: docker compose logs picar"

image="$(docker inspect --format '{{.Config.Image}}' "$container")"
echo "Running: ${image}"
