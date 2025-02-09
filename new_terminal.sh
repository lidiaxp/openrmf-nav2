#!/bin/bash

# Find the container running with the 'nav2' image
CONTAINER_ID=$(docker ps --filter "ancestor=nav2" --format "{{.ID}}")

# Check if a container was found
if [ -n "$CONTAINER_ID" ]; then
    echo "Attaching to container: $CONTAINER_ID"
    docker exec -it "$CONTAINER_ID" /bin/bash
else
    echo "No running container found for image 'nav2'."
fi
