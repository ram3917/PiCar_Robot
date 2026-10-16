#!/usr/bin/env bash
# Manual smoke test, run before merging to main. It builds the base image and a wheel from this
# checkout, installs the wheel into a scratch volume with install_picar.sh (the same script the
# Pi uses), and starts the launch file on fake hardware. It checks the nodes come up and a sweep
# scan is published. Needs Docker. It touches nothing on the robot.
#
#   docker/smoke_test.sh
set -euo pipefail
# Git Bash rewrites /c/... paths in docker -v arguments unless this is set. Harmless elsewhere.
export MSYS_NO_PATHCONV=1

repo="$(cd "$(dirname "$0")/.." && pwd)"
version="0.0.0.dev1"
image="picar:smoke"
volume="picar-smoke-install"
container="picar-smoke"
out="${repo}/build/smoke"

cleanup() {
    docker rm -f "${container}" >/dev/null 2>&1 || true
}
trap cleanup EXIT

step() { echo; echo "== $*"; }

step "Building the base image"
(cd "${repo}" && docker build -f docker/Dockerfile -t "${image}" .)

step "Building the wheel ${version} from this checkout"
rm -rf "${out}" && mkdir -p "${out}"
docker run --rm -e PICAR_BUILD_VERSION="${version}" \
    -v "${repo}/picar:/src:ro" -v "${out}:/out" --entrypoint bash "${image}" \
    -c 'cp -r /src /tmp/pkg && uv build --wheel --out-dir /out /tmp/pkg'

step "Installing the wheel with install_picar.sh"
docker volume rm -f "${volume}" >/dev/null 2>&1 || true
docker run --rm -e PICAR_VERSION="${version}" \
    -e PICAR_WHEEL="/out/picar-${version}-py3-none-any.whl" \
    -v "${volume}:/opt/picar" -v "${out}:/out:ro" \
    --entrypoint /ws/install_picar.sh "${image}"

step "Starting the launch file on fake hardware"
docker run -d --name "${container}" \
    -e PICAR_VERSION="${version}" -e PICAR_HARDWARE=fake \
    -v "${volume}:/opt/picar" "${image}" >/dev/null
sleep 12

nodes="$(docker exec "${container}" bash -c 'source /ws/picar_env.sh; ros2 node list' || true)"
echo "${nodes}"
if ! grep -q '/controller' <<<"${nodes}" || ! grep -q '/planner' <<<"${nodes}"; then
    echo "FAIL: controller and planner did not both come up" >&2
    docker logs "${container}" | tail -40 >&2
    exit 1
fi

scan="$(docker exec "${container}" bash -c 'source /ws/picar_env.sh; timeout 10 ros2 topic echo --once /controller/scan --field angle_max | head -n 1' || true)"
if [ -z "${scan}" ]; then
    echo "FAIL: no sweep scan on /controller/scan" >&2
    docker logs "${container}" | tail -40 >&2
    exit 1
fi

echo
echo "PASS: nodes up, sweep scan published (angle_max ${scan} rad)."
