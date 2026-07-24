container=`docker ps | awk '{if (NR!=1) {print $1}}'` ; docker exec -it $container bash 
