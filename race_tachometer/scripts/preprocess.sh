#!/bin/bash
# 2013-07-16 Sebastian Rockel
# check and create data file if necessary

DATADIR="`rospack find race_tachometer`/data"
DATAFILE="$DATADIR/data.dat"

# Check directory
[ -d $DATADIR ] || mkdir -p "$DATADIR"

# Check file
[ -f $DATAFILE ] || touch "$DATAFILE"

exit 0
