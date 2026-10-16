#!/bin/bash
# Installs the picar release PICAR_VERSION into the /opt/picar volume. Run it by hand, and only then:
#
#   docker compose run --rm --entrypoint /ws/install_picar.sh picar
#
# Nothing else calls it. The robot keeps running the installed release until you run this.
# It downloads the wheel and its .sha256 from GitHub Release v<PICAR_VERSION>, checks the checksum,
# and installs into a staging folder. Only after that succeeds does it replace the old install and
# delete it, so a failed download or checksum leaves the old release in place.
#
# PICAR_WHEEL=/path/to/picar.whl installs a local wheel instead, with no download and no checksum.
# That is for smoke tests of an unreleased build (docker/smoke_test.sh).
#
# Stop the robot first (Ctrl-C the launch, or docker compose stop picar) so nothing runs from the
# folder while it is replaced.
set -eo pipefail

source /ws/picar_env.sh
export UV_LINK_MODE=copy  # the scratch folder and the cache are on different filesystems

: "${PICAR_VERSION:?PICAR_VERSION is unset: set it in docker/.env, for example PICAR_VERSION=0.3.0}"
repo="${PICAR_REPO:-ram3917/PiCar_Robot}"
wheel="picar-${PICAR_VERSION}-py3-none-any.whl"
prefix=/opt/picar
work="$(mktemp -d "${prefix}/.install.XXXXXX")"
trap 'rm -rf "${work}"' EXIT

fetch() {
    curl --fail --silent --show-error --location --retry 5 --retry-delay 3 --retry-all-errors \
        -o "${work}/$1" "https://github.com/${repo}/releases/download/v${PICAR_VERSION}/$1"
}

if [ -n "${PICAR_WHEEL:-}" ]; then
    echo "Installing local wheel ${PICAR_WHEEL} (no checksum check)"
    cp "${PICAR_WHEEL}" "${work}/${wheel}"
else
    echo "Downloading picar ${PICAR_VERSION} from ${repo}"
    fetch "${wheel}"
    fetch "${wheel}.sha256"
    (cd "${work}" && sha256sum -c "${wheel}.sha256")
fi

# Install into the staging folder. The wheel's data files (launch, config, ament index) land in
# the prefix's share/, its scripts in bin/. ros2 finds executables under lib/picar, so link them.
uv pip install --no-deps --python /usr/bin/python3 --prefix "${work}/prefix" "${work}/${wheel}"
rm -f "${work}/prefix/.lock"
mkdir -p "${work}/prefix/lib/picar"
for exe in "${work}/prefix"/bin/*; do
    ln -sfn "../../bin/$(basename "${exe}")" "${work}/prefix/lib/picar/$(basename "${exe}")"
done
echo "${PICAR_VERSION}" > "${work}/prefix/PICAR_VERSION"

rm -rf "${prefix}/current"
mv "${work}/prefix" "${prefix}/current"
echo "Installed picar ${PICAR_VERSION} into ${prefix}/current"
