#!/bin/bash

# =================================================================================
#           FYP DRONE - MESH EXPORT SCRIPT (v3 - Final Corrected Version)
# This script replicates the settings saved in the INI file for command-line use.
# It now correctly handles argument order for rtabmap-export.
# =================================================================================

# --- Configuration based on mesh_settings INI file ---
EXPORT_FLAGS="--mesh --texture --texture_size 8192 --mls --voxel 0.05"


# --- Script Execution Logic ---
set -e # Exit immediately if a command exits with a non-zero status.

# Default paths and names.
DEFAULT_DB_PATH="$HOME/.ros/rtabmap.db"
DEFAULT_OUTPUT_DIR="$HOME/dai_ws/src/my_drone/maps"
DEFAULT_OUTPUT_NAME="exported_mesh"

# Process command-line arguments.
# Usage: ./export_map.sh [path/to/database.db] [path/to/output_directory] [output_filename_without_extension]
DB_PATH=${1:-$DEFAULT_DB_PATH}
OUTPUT_DIR=${2:-$DEFAULT_OUTPUT_DIR}
OUTPUT_NAME=${3:-$DEFAULT_OUTPUT_NAME}

# Ensure the output directory exists.
mkdir -p "$OUTPUT_DIR"

# Check if the database file exists
if [ ! -f "$DB_PATH" ]; then
    echo "Error: Database file not found at '$DB_PATH'"
    exit 1
fi

echo "================================================="
echo "Starting RTAB-Map Mesh Export..."
echo "  Database:      $DB_PATH"
echo "  Output Dir:    $OUTPUT_DIR"
echo "  Output Name:   $OUTPUT_NAME"
echo "  Flags:         $EXPORT_FLAGS"
echo "================================================="

#
# === THE FIX IS HERE: Correct argument order ===
#
# The structure is: rtabmap-export [all flags] [input_db] [output_name_base]
# The --output_dir flag tells the command where to place the output_name_base files.
#
rtabmap-export --output_dir "$OUTPUT_DIR" $EXPORT_FLAGS "$DB_PATH"

echo "================================================="
echo "Export process completed successfully!"
echo "Check for ${OUTPUT_NAME}.obj in the directory '$OUTPUT_DIR'."
echo "================================================="
