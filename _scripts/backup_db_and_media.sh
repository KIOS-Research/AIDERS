#!/bin/bash

# Load environment variables from .env file
dotenv_file="./.env"
if [ -f "$dotenv_file" ]; then
  source $dotenv_file
else
  echo "Error: .env file not found!"
  exit 1
fi

TIMESTAMP=$(date +"%Y%m%d%H%M%S")
BACKUP_DIR="./backup_${TIMESTAMP}"

# Create a new directory for this backup
mkdir $BACKUP_DIR

# Backup the database
docker exec db /usr/bin/mysqldump -u root --password=${SQL_PASSWORD} ${SQL_DATABASE} > $BACKUP_DIR/backup_${TIMESTAMP}.sql

# Copy the media folder
cp -r web/django_api/aiders/media $BACKUP_DIR/media

# Create a tar file with the backup
tar -cvzf $BACKUP_DIR.tar $BACKUP_DIR

# Remove the backup directory
rm -rf $BACKUP_DIR