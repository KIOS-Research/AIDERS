#!/bin/bash

if [ "$#" -ne 1 ]; then
  echo "Usage: $0 <file_name>"
  exit 1
fi

file_name="$1"

if ! ls "$file_name.tar.gz."* 1> /dev/null 2>&1; then
  echo "Split archive files not found for: $file_name"
  exit 1
fi

cat "$file_name.tar.gz."* | tar xzvf -