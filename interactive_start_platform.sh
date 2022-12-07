docker-compose -f docker-compose.dev.yml up -d
docker exec -it web bash  -c '${APP_DIR}/start_ros_services.sh; source ${APP_DIR}/django_api/logic/djiswarmcatkinws/devel/setup.bash; cd  ${APP_DIR}/django_api; bash;'
