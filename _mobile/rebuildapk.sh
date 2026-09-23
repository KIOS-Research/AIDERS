#!/bin/bash

# Define the split parts and the target TAR file
base_filename="aiders_apk_WS-V2.17.tar.part_"
tar_file="aiders_apk_WS-V2.17.tar"

# Reassemble the split parts into one TAR file
echo "Rebuilding the TAR archive..."
cat ${base_filename}aa ${base_filename}ab ${base_filename}ac > ${tar_file}

# Verify the TAR archive contents (optional)
echo "Listing contents of the TAR archive:"
tar -tvf ${tar_file}

# Extract aidersV4-WS-V2.17.apk
echo "Extracting aidersV4-WS-V2.17.apk..."
tar -xf ${tar_file} aidersV4-WS-V2.17.apk
if [ ! -f "aidersV4-WS-V2.17.apk" ]; then
    echo "Error: Failed to extract aidersV4-WS-V2.17.apk!"
    exit 1
fi
echo "aidersV4-WS-V2.17.apk extracted successfully."

# Extract aidersV5-WS-V2.17.apk
echo "Extracting aidersV5-WS-V2.17.apk..."
tar -xf ${tar_file} aidersV5-WS-V2.17.apk
if [ ! -f "aidersV5-WS-V2.17.apk" ]; then
    echo "Error: Failed to extract aidersV5-WS-V2.17.apk!"
    exit 1
fi
echo "aidersV5-WS-V2.17.apk extracted successfully."

# Delete the reassembled TAR file (keeping the part files intact)
echo "Cleaning up: Deleting ${tar_file}..."
rm -f ${tar_file}

echo "Both APKs have been successfully extracted, and the TAR file has been removed!"
