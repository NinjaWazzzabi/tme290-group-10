docker build -f ./action-planner/Dockerfile  -t registry.git.chalmers.se/kalcic/tme290-group-10:action-planner  ./action-planner/
docker build -f ./aimpoint-driver/Dockerfile -t registry.git.chalmers.se/kalcic/tme290-group-10:aimpoint-driver ./aimpoint-driver/
docker build -f ./cone-detection/Dockerfile  -t registry.git.chalmers.se/kalcic/tme290-group-10:cone-detection  ./cone-detection/
docker build -f ./kiwi-detection/Dockerfile  -t registry.git.chalmers.se/kalcic/tme290-group-10:kiwi-detection  ./kiwi-detection/

docker push registry.git.chalmers.se/kalcic/tme290-group-10:action-planner
docker push registry.git.chalmers.se/kalcic/tme290-group-10:aimpoint-driver
docker push registry.git.chalmers.se/kalcic/tme290-group-10:cone-detection
docker push registry.git.chalmers.se/kalcic/tme290-group-10:kiwi-detection