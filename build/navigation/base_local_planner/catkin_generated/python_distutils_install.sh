#!/bin/sh -x

if [ -n "$DESTDIR" ] ; then
    case $DESTDIR in
        /*) # ok
            ;;
        *)
            /bin/echo "DESTDIR argument must be absolute... "
            /bin/echo "otherwise python's distutils will bork things."
            exit 1
    esac
    DESTDIR_ARG="--root=$DESTDIR"
fi

cd "/home/alfred/quan_ws/src/navigation/base_local_planner"

# Note that PYTHONPATH is pulled from the environment to support installing
# into one location when some dependencies were installed in another
# location, #123.
/usr/bin/env \
    PYTHONPATH="/home/alfred/quan_ws/install/lib/python2.7/dist-packages:/home/alfred/quan_ws/build/lib/python2.7/dist-packages:$PYTHONPATH" \
    CATKIN_BINARY_DIR="/home/alfred/quan_ws/build" \
    "/usr/bin/python" \
    "/home/alfred/quan_ws/src/navigation/base_local_planner/setup.py" \
    build --build-base "/home/alfred/quan_ws/build/navigation/base_local_planner" \
    install \
    $DESTDIR_ARG \
    --install-layout=deb --prefix="/home/alfred/quan_ws/install" --install-scripts="/home/alfred/quan_ws/install/bin"
