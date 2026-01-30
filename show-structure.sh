#!/bin/bash

# Generiert eine Übersicht der Projektstruktur

echo "Beat Analyzer - Projektstruktur"
echo "================================"
echo ""

find /home/ra/beat-analyzer -type f \( \
    -name "*.h" -o \
    -name "*.cpp" -o \
    -name "*.cmake" -o \
    -name "*.yaml" -o \
    -name "*.md" -o \
    -name "*.sh" \
) | sort | while read file; do
    relative_path="${file#/home/ra/beat-analyzer/}"
    # Indentation based on directory depth
    depth=$(echo "$relative_path" | grep -o "/" | wc -l)
    indent=$(printf '%*s' $((depth * 2)) "")
    filename=$(basename "$file")
    
    # Show file type
    case "${file##*.}" in
        h)   type="[H] Header" ;;
        cpp) type="[C] Source" ;;
        md)  type="[D] Docs  " ;;
        yaml) type="[Y] Config" ;;
        sh)  type="[S] Script" ;;
        *)   type="[?] Other " ;;
    esac
    
    # Calculate file size
    size=$(stat -f%z "$file" 2>/dev/null || stat -c%s "$file" 2>/dev/null)
    
    printf "%s%s %s (%d bytes)\n" "$indent" "$type" "$filename" "$size"
done

echo ""
echo "Verzeichnis-Übersicht:"
echo ""
tree -L 3 -I 'build|.git' /home/ra/beat-analyzer 2>/dev/null || find /home/ra/beat-analyzer -type d | head -20
