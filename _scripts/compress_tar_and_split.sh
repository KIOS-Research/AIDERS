#!/bin/bash

if [ "$#" -ne 1 ]; then
  echo "Usage: $0 <file_name>"
  exit 1
fi

file_name="$1"

if [ ! -f "$file_name" ]; then
  echo "File does not exist: $file_name"
  exit 1
fi

tar cvzf - "$file_name" | split --bytes=99MB - "$file_name.tar.gz."
