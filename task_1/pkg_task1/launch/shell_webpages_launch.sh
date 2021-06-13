#!/bin/bash

# Store URL in a variable
URL1="http://www.hivemq.com/demos/websocket-client/"
URL2="https://docs.google.com/spreadsheets/d/1e5kdWTpRGpYrf_LlEzxKefqHUVVYlShIKOXmnn1_hMc/edit#gid=0"

# Print some message
echo "** Opening $URL1 in Firefox **"
echo "** Opening $URL2 in Firefox **"

# Use firefox to open the URL in a new window
firefox -new-window $URL1 $URL2

