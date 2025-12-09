#!/usr/bin/env bash
# Delete all .txt and .png files in the current directory (non-recursive)

shopt -s nullglob

files=( ./*.txt ./*.png )

if [ ${#files[@]} -eq 0 ]; then
    echo "No .txt or .png files found in current directory."
    exit 0
fi

echo "The following files will be deleted:"
printf '  %s\n' "${files[@]}"

read -p "Are you sure you want to delete these files? [y/N] " ans
case "$ans" in
    [Yy]* )
        rm -- "${files[@]}"
        echo "Files deleted."
        ;;
    * )
        echo "Aborted."
        exit 1
        ;;
esac
