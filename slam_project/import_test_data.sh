#!/bin/bash

# Specifying path for project repo
PROJECT_REPO=$( pwd )/slam_project

# Unzipping file and specifying to extract inside project repo
unzip "$PROJECT_REPO"/slam_handout.zip -d "$PROJECT_REPO"

# Copying the pickle file into test file
cp "$PROJECT_REPO"/slam_handout/tests/test_data.pickle "$PROJECT_REPO"/tests/

# Removing handout
rm -Rf "$PROJECT_REPO"/slam_handout

# Checks if test data is moved successfully
FILE="$PROJECT_REPO"/tests/test_data.pickle
if [ -f "$FILE" ]; then
    echo "$FILE exists."
else
    echo "$FILE does not exist."
fi


