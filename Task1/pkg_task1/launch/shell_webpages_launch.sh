#!/bin/bash

URL1="http://www.hivemq.com/demos/websocket-client/"

URL2="https://docs.google.com/spreadsheets/d/1RsBE_DMCctSoE6ylxZpzo_ysmX-gD8Hm56dHFmkPB5U/edit#gid=587969560"
# Use firefox to open the URL in a new window
echo "** Opening $URL1  $URL2 in Firefox **"
firefox --new-window $URL1 $URL2
