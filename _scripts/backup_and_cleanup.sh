#!/bin/bash

# Determine the current directory of the script
CURRENT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"

# Load environment variables from the .env file
set -a
source "${CURRENT_DIR}/../.env" 2>/dev/null
set +a

# Variables
CONTAINER_NAME="db"
DATE=$(date +"%Y-%m-%d_%H-%M-%S")
BACKUP_DIR="./_backups/backup_$DATE"
BACKUP_FILE="$BACKUP_DIR/Database.sql"
COMPRESSED_BACKUP_FILE="./_backups/backup_$DATE.tar.gz"
DAYS_TO_KEEP=14 # Number of days to retain data
MEDIA_DIR="web/django_api/aiders/media"

# List of tables and columns to clean up (one per line, format: table_name,column_name)
TABLES_TO_CLEAN=(
	# general:
	"aiders_flyingreport,time"
	"aiders_systemmonitoring,time"
	"aiders_frontenduserinput,time"
	"aiders_terminal,time"
	# operation:
	"aiders_onlinesession,end_time"
	"aiders_manuallysetobject,created_at"
	"aiders_manuallysetobjectlocation,received_at"
	"aiders_manuallysetobjectdescription,updated_at"
	"aiders_weatherstation,time"
	# drone:
	## general
	"aiders_telemetry,time"
	"aiders_errormessage,time"
	"aiders_controldevice,time"
	"aiders_algorithm,time"
	"aiders_missionlog,time"
	"aiders_mission,time"
	"aiders_missionpoint,time"
	"aiders_mavlinklog,time"
	## detection
	"aiders_detectedobjectdescription,updated_at"
	"aiders_detecteddisaster,time"
	"aiders_detectioncrowdlocalizationresults,time"
	"aiders_detectedobject,time"
	## advance
	"aiders_watersampler,time"
	"aiders_ballistic,time"
	"aiders_safedroneresults,datetime"
	# device:
	"aiders_devicetelemetry,time"
	# lora:
	"aiders_baloratelemetry,time"
	"aiders_baloramonitor,time"
)

TABLES_TO_CLEAN_WITH_SESSIONS=(
	# drone:
	## general
	"aiders_buildmapsession,end_time,aiders_buildmapimage,session_id"
	# "aiders_buildmapadvancesession,time,aiders_buildmapadvanceimage,session_id"
	# ## live stream
	# "aiders_livestreamsession,end_time,aiders_rawframe,live_stream_session_id"
	# ## detection
	# "aiders_detectionsession,end_time,aiders_detectionframe,detection_session_id"
	# ## advance
	# "aiders_lidarpointsession,end_time,aiders_lidarpoint,lidar_point_session_id"
	# # device:
	# "aiders_devicesession,end_time,aiders_deviceimage,session_id"
)

# # Ensure backup directory exists
# mkdir -p $BACKUP_DIR

# # Backup database using mysqldump inside the container
# echo "Starting database backup..."
# docker exec $CONTAINER_NAME sh -c "mysqldump -u root -p$DB_PASSWORD $DB_DATABASE" >$BACKUP_FILE

# # Copy media files to the backup directory
# echo "Copying media files..."
# cp -r $MEDIA_DIR $BACKUP_DIR/media

# # Compress the backup directory
# echo "Compressing backup..."
# tar -czf $COMPRESSED_BACKUP_FILE -C $BACKUP_DIR .

# # Remove the uncompressed backup directory after compression
# rm -rf $BACKUP_DIR

# echo "Backup completed: $COMPRESSED_BACKUP_FILE"

# # Cleanup old data (older than the specified number of days) from specified tables
# echo "Starting cleanup of old data in database..."

# # Generate SQL statements for cleanup
SQL_CLEANUP_STATEMENTS=""
for ENTRY in "${TABLES_TO_CLEAN_WITH_SESSIONS[@]}"; do
	IFS=',' read -r SESSION_TABLE COLUMN DATA_TABLE SESSION_ID <<<"$ENTRY"
	SQL_CLEANUP_STATEMENTS+="DELETE FROM $DATA_TABLE JOIN $SESSION_TABLE ON $DATA_TABLE.$SESSION_ID = $SESSION_TABLE.id WHERE $SESSION_TABLE.$COLUMN < NOW() - INTERVAL $DAYS_TO_KEEP DAY;\n"
	SQL_CLEANUP_STATEMENTS+="DELETE FROM $SESSION_TABLE WHERE $COLUMN < NOW() - INTERVAL $DAYS_TO_KEEP DAY;\n"
done
# for ENTRY in "${TABLES_TO_CLEAN[@]}"; do
# 	IFS=',' read -r TABLE COLUMN <<<"$ENTRY"
# 	SQL_CLEANUP_STATEMENTS+="DELETE FROM $TABLE WHERE $COLUMN < NOW() - INTERVAL $DAYS_TO_KEEP DAY;\n"
# done

echo $SQL_CLEANUP_STATEMENTS
# Execute the cleanup statements
# docker exec db sh -c "mysql -u root -p\$DB_PASSWORD \$DB_DATABASE -e \"SET SQL_SAFE_UPDATES = 0; $SQL_CLEANUP_STATEMENTS SET SQL_SAFE_UPDATES = 1;\""

# echo "Database cleanup completed."

# # Cleanup old files in the media directory (older than the specified number of days)
# echo "Starting cleanup of old files in media directory..."
# sudo find $MEDIA_DIR -type f -mtime +$DAYS_TO_KEEP -exec rm {} \;
# sudo find $MEDIA_DIR -type d -empty -delete

# echo "Media directory cleanup completed."

# echo "Backup and cleanup process completed successfully."
