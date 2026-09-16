#!/usr/bin/env bash
set -euo pipefail

temporary_files=()

cleanup() {
    if ((${#temporary_files[@]})); then
        rm -f -- "${temporary_files[@]}"
    fi
}
trap cleanup EXIT

fail() {
    echo "NVIDIA setup failed: $*" >&2
    exit 1
}

install_toolkit() {
    local repository_url
    repository_url='https://nvidia.github.io/libnvidia-container/stable/rpm/nvidia-container-toolkit.repo'

    echo 'NVIDIA Container Toolkit is not installed. Installing it from the NVIDIA repository...'
    if command -v apt-get >/dev/null 2>&1; then
        local keyring_tmp repository_tmp
        keyring_tmp="$(mktemp)"
        repository_tmp="$(mktemp)"
        temporary_files+=("${keyring_tmp}" "${repository_tmp}")

        sudo apt-get update
        sudo apt-get install -y --no-install-recommends ca-certificates curl gnupg2
        curl -fsSL https://nvidia.github.io/libnvidia-container/gpgkey | \
            gpg --dearmor --batch --yes -o "${keyring_tmp}"
        curl -fsSL https://nvidia.github.io/libnvidia-container/stable/deb/nvidia-container-toolkit.list | \
            sed 's#deb https://#deb [signed-by=/usr/share/keyrings/nvidia-container-toolkit-keyring.gpg] https://#g' \
            > "${repository_tmp}"
        sudo install -m 0644 "${keyring_tmp}" \
            /usr/share/keyrings/nvidia-container-toolkit-keyring.gpg
        sudo install -m 0644 "${repository_tmp}" \
            /etc/apt/sources.list.d/nvidia-container-toolkit.list
        sudo apt-get update
        sudo apt-get install -y nvidia-container-toolkit
    elif command -v dnf >/dev/null 2>&1; then
        local repository_tmp
        repository_tmp="$(mktemp)"
        temporary_files+=("${repository_tmp}")

        sudo dnf install -y curl
        curl -fsSL "${repository_url}" -o "${repository_tmp}"
        sudo install -m 0644 "${repository_tmp}" \
            /etc/yum.repos.d/nvidia-container-toolkit.repo
        sudo dnf install -y nvidia-container-toolkit
    elif command -v zypper >/dev/null 2>&1; then
        if ! zypper repos --uri | grep -Fq "${repository_url}"; then
            sudo zypper addrepo "${repository_url}" nvidia-container-toolkit
        fi
        sudo zypper --gpg-auto-import-keys install -y nvidia-container-toolkit
    else
        fail 'automatic toolkit installation supports apt, dnf, and zypper only. Install nvidia-container-toolkit using https://docs.nvidia.com/datacenter/cloud-native/container-toolkit/latest/install-guide.html and rerun the script.'
    fi

    command -v nvidia-ctk >/dev/null 2>&1 || \
        fail 'nvidia-container-toolkit installation finished, but nvidia-ctk is still unavailable.'
}

command -v nvidia-smi >/dev/null 2>&1 || \
    fail 'nvidia-smi is missing; install the NVIDIA driver for the host first.'
command -v docker >/dev/null 2>&1 || \
    fail 'docker is missing; install Docker Engine first.'
command -v sudo >/dev/null 2>&1 || \
    fail 'sudo is required to configure and restart the system Docker daemon.'

echo 'Checking the NVIDIA host driver...'
nvidia-smi -L || fail 'the NVIDIA driver is installed but the GPU is unavailable.'

if ! command -v nvidia-ctk >/dev/null 2>&1; then
    install_toolkit
fi

echo 'Configuring NVIDIA Container Toolkit for Docker...'
sudo nvidia-ctk runtime configure --runtime=docker

echo 'Generating the NVIDIA CDI specification...'
sudo install -d -m 0755 /var/run/cdi
sudo nvidia-ctk cdi generate --output=/var/run/cdi/nvidia.yaml

echo 'Restarting Docker; currently running containers will be stopped...'
if [[ -d /run/systemd/system ]] && command -v systemctl >/dev/null 2>&1; then
    sudo systemctl restart docker
elif command -v sv >/dev/null 2>&1 && [[ -e /var/service/docker ]]; then
    sudo sv restart docker
elif command -v service >/dev/null 2>&1; then
    sudo service docker restart
else
    fail 'Docker was configured, but its service manager was not detected. Restart Docker manually, then rerun this script.'
fi

for _ in {1..15}; do
    if docker info >/dev/null 2>&1; then
        break
    fi
    sleep 1
done
docker info >/dev/null 2>&1 || \
    fail 'Docker did not become available after the restart.'

echo 'Checking NVIDIA GPU access from a container...'
docker run --rm --runtime=nvidia \
    -e NVIDIA_VISIBLE_DEVICES=all \
    ubuntu:22.04 nvidia-smi -L

echo 'NVIDIA support is ready. Open the Desktop (NVIDIA GPU) Dev Container profile.'
