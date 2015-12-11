#! /bin/bash
find . -name *.jpg -exec mv '{}' "$1/{}" \;
