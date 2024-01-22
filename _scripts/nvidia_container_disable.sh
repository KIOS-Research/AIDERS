#!/bin/bash

current_dir="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"

docker_compose_file="${current_dir}/../docker-compose.yml"
sim_docker_compose_file="${current_dir}/../_simulator/docker-compose.yml"


delete_nvidia_from_docker_compose() {
    local target_file="$1"
    sed -i '/# nvidia-runtime/d' $target_file
}


delete_nvidia_from_docker_compose $docker_compose_file
delete_nvidia_from_docker_compose $sim_docker_compose_file


echo -e "\nDone!\n"