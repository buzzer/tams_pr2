#!/bin/bash
# This script updates to the latest revisions within the ROS_WORKSPACE
# directory. Then it collects the current revisions and builds all packages.
# Eventually all output is logged to a temporary file for observation.
#
# Author: Sebastian Rockel
#
ROSWSDIR="$ROS_WORKSPACE"
ROSBUILDDIR="$ROSWSDIR""/""race"
ROSSTACKSDIR="$ROSWSDIR""/""stacks"
ROSJAVADIR="$ROSSTACKSDIR""/""rosjava_core"
CALLER="`basename $0`"
PREFIX="$CALLER""_""$RANDOM"
LOGDIR="/tmp/""$PREFIX"
LOGFILE="$LOGDIR""/""$CALLER"".log"
SUCCESS=
mkdir -p "$LOGDIR" || exit -1
#echo "Logging to dir: $LOGDIR"
echo && echo "Logging to file: $LOGFILE" && touch "$LOGFILE" || exit -1

error ()
{
  echo && echo "Aborted, see the logfile for the reason: $LOGFILE"
  exit -1
}

success ()
{
  echo
  echo "built with following results:"
  tail -n 10 "$LOGFILE" | head -n 3
  exit 0
}

build ()
{
  echo "changing to $ROSWSDIR"
  cd "$ROSWSDIR" || exit -1
  echo "updating to the latest stack revisions.."
  # the "yes s" answers "skip" if the checked out repo doesn't match the one in
  # the rosinstall (e.g. read/write SVN instead of read-only, git-svn instead of
  # svn):
  #Failed to detect svn presence at [...]/race/race/race_cbr.
  #  (d)elete and replace, (a)bort, (b)ackup and replace, (s)kip:
  yes s | rosws update &>> "$LOGFILE" || error || exit -1
  echo "workaround: deleting catkin package.xml files.."
  find ./ -name "package.xml" | xargs rm -f &>> "$LOGFILE" || error || exit -1
  echo "collecting stack revisions info.."
  rosws &>> "$LOGFILE" || error || exit -1
  echo "changing to $ROSJAVADIR"
  cd "$ROSJAVADIR"
  echo "(re-)building rosjava.."
  # TODO check for symbolic link and create if necessary, export PATH
  ./gradlew &>> "$LOGFILE" || error || exit -1
  echo "changing to $ROSBUILDDIR"
  cd "$ROSBUILDDIR" || exit -1
  echo "updating the rosdep cache.."
  rosdep update &>> "$LOGFILE" #|| error || exit -1
  echo "installing dependencies.."
  yes Y | rosdep install * &>> "$LOGFILE" #|| error || exit -1
  echo "building all packages.."
  #rosmake * &>> "$LOGFILE" || error || exit -1
  # build all packages regardless of errors
  rosmake -a -r &>> "$LOGFILE" || error || exit -1
  echo && echo -n "Time statistics:"
}

{
  time build || exit -1
  success

} 2>&1 | tee -a "$LOGFILE" || error && exit -1
# Never reaches here
