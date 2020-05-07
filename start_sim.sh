# Compile all
docker build -f ./action-planner/Dockerfile  -t action-planner  ./action-planner/
docker build -f ./aimpoint-driver/Dockerfile -t aimpoint-driver ./aimpoint-driver/
docker build -f ./cone-detection/Dockerfile  -t cone-detection  ./cone-detection/
docker build -f ./kiwi-detection/Dockerfile  -t kiwi-detection  ./kiwi-detection/

# Start
docker-compose -f services.yml up