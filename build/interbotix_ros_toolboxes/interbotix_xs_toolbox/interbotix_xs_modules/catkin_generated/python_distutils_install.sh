#!/bin/sh

if [ -n "$DESTDIR" ] ; then
    case $DESTDIR in
        /*) # ok
            ;;
        *)
            /bin/echo "DESTDIR argument must be absolute... "
            /bin/echo "otherwise python's distutils will bork things."
            exit 1
    esac
fi

echo_and_run() { echo "+ $@" ; "$@" ; }

echo_and_run cd "/home/saranya/interbotix_ws/src/interbotix_ros_toolboxes/interbotix_xs_toolbox/interbotix_xs_modules"

# ensure that Python install destination exists
echo_and_run mkdir -p "$DESTDIR/home/saranya/interbotix_ws/install/lib/python3/dist-packages"

# Note that PYTHONPATH is pulled from the environment to support installing
# into one location when some dependencies were installed in another
# location, #123.
echo_and_run /usr/bin/env \
    PYTHONPATH="/home/saranya/interbotix_ws/install/lib/python3/dist-packages:/home/saranya/interbotix_ws/build/lib/python3/dist-packages:$PYTHONPATH" \
    CATKIN_BINARY_DIR="/home/saranya/interbotix_ws/build" \
    "/usr/bin/python3" \
    "/home/saranya/interbotix_ws/src/interbotix_ros_toolboxes/interbotix_xs_toolbox/interbotix_xs_modules/setup.py" \
    egg_info --egg-base /home/saranya/interbotix_ws/build/interbotix_ros_toolboxes/interbotix_xs_toolbox/interbotix_xs_modules \
    build --build-base "/home/saranya/interbotix_ws/build/interbotix_ros_toolboxes/interbotix_xs_toolbox/interbotix_xs_modules" \
    install \
    --root="${DESTDIR-/}" \
    --install-layout=deb --prefix="/home/saranya/interbotix_ws/install" --install-scripts="/home/saranya/interbotix_ws/install/bin"
