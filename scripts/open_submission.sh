#!/usr/bin/env bash
# Submission Shortcut Launcher for Islam Qaj Mazarul (Group 2, Sakhir)
# Opens LMS submission page, GitHub repository, and local submission summary

WORKSPACE_DIR="/home/admin1/Documents/Final Project - Reactive Controller Competition in"$'\n'"F1Tenth"
LMS_URL="https://class.tu.ac.kr/courses/34717/assignments/803271"
GITHUB_URL="https://github.com/MAVERICK2205MAX/Final-Project--F1Tenth-Reactive-Controller-Sakhir"
SUBMISSION_DOC="${WORKSPACE_DIR}/SUBMISSION.md"

echo "=========================================================="
echo "  FINAL PROJECT SUBMISSION SHORTCUT"
echo "  Student: Islam Qaj Mazarul (Group 2, Sakhir)"
echo "=========================================================="

# Try opening in GUI editor / viewer or browser
if command -v xdg-open &>/dev/null; then
    xdg-open "${SUBMISSION_DOC}" &>/dev/null &
    xdg-open "${LMS_URL}" &>/dev/null &
    xdg-open "${GITHUB_URL}" &>/dev/null &
fi

echo "Submission details:"
if [ -f "${SUBMISSION_DOC}" ]; then
    head -n 45 "${SUBMISSION_DOC}"
else
    echo "Document located at ${SUBMISSION_DOC}"
fi
