if [ "$(docker inspect -f '{{.State.Running}}' fast_livo2 2>/dev/null)" != "true" ]; then
    docker start fast_livo2 2>/dev/null || (echo "Container not found. Run run_container.sh first." && exit 1)
fi

docker exec -it fast_livo2 /bin/bash
