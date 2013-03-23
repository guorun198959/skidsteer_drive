#!/usr/bin/env sh
# generated from catkin/cmake/templates/env.sh.in

if [ $# -eq 0 ] ; then
  /bin/echo "Entering environment at '/home/smp2/dry/skidsteer_drive/build/devel', type 'exit' to leave"
  . "/home/smp2/dry/skidsteer_drive/build/devel/setup.sh"
  "$SHELL" -i
  /bin/echo "Exiting environment at '/home/smp2/dry/skidsteer_drive/build/devel'"
else
  . "/home/smp2/dry/skidsteer_drive/build/devel/setup.sh"
  exec "$@"
fi
