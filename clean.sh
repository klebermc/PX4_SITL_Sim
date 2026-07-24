docker rmi -f ros-test
docker rmi -f $(docker images -f "dangling=true" -q)

