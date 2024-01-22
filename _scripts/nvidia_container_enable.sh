#!/bin/bash

current_dir="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"

docker_compose_file="${current_dir}/../docker-compose.yml"
sim_docker_compose_file="${current_dir}/../_simulator/docker-compose.yml"


add_nvidia_to_docker_compose() {
    local target_file="$1"
    local search_string="$2"

    line_below_container_name=$(grep -A 1 "$search_string" "$target_file" | tail -n 1)

    if [ -n "$line_below_container_name" ]; then
        if [[ $line_below_container_name == *"runtime: nvidia"* ]]; then
            echo "$search_string appears to be ready to use Nvidia runtime environment."
            return
        fi
    else
        echo "String '$search_string' not found in the file."
        return
    fi

    echo "Adding Nvidia runtime instructions under '$search_string'"
    
    sed -i "/$search_string/ {
        a\        runtime: nvidia                       # nvidia-runtime
        a\        deploy:                               # nvidia-runtime
        a\          resources:                          # nvidia-runtime
        a\            reservations:                     # nvidia-runtime
        a\              devices:                        # nvidia-runtime
        a\                - driver: nvidia              # nvidia-runtime
        a\                  device_ids: ['0']           # nvidia-runtime
        a\                  capabilities: [gpu,video]   # nvidia-runtime
    }" "$target_file"


}


# add_nvidia_to_docker_compose $docker_compose_file "container_name: web"
add_nvidia_to_docker_compose $docker_compose_file "container_name: cv"
add_nvidia_to_docker_compose $sim_docker_compose_file "container_name: sim"
# SafeML
add_nvidia_to_docker_compose $sim_docker_compose_file "container_name: safeml"
