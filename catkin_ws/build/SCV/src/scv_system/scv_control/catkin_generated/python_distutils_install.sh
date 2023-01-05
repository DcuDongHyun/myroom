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

echo_and_run cd "/home/dong/catkin_ws/src/SCV/src/scv_system/scv_control"

# ensure that Python install destination exists
echo_and_run mkdir -p "$DESTDIR/home/dong/catkin_ws/install/lib/python3/dist-packages"

# Note that PYTHONPATH is pulled from the environment to support installing
# into one location when some dependencies were installed in another
# location, #123.
echo_and_run /usr/bin/env \
    PYTHONPATH="/home/dong/catkin_ws/install/lib/python3/dist-packages:/home/dong/catkin_ws/build/lib/python3/dist-packages:$PYTHONPATH" \
    CATKIN_BINARY_DIR="/home/dong/catkin_ws/build" \
    "/usr/bin/python3" \
    "/home/dong/catkin_ws/src/SCV/src/scv_system/scv_control/setup.py" \
     \
    build --build-base "/home/dong/catkin_ws/build/SCV/src/scv_system/scv_control" \
    install \
    --root="${DESTDIR-/}" \
    --install-layout=deb --prefix="/home/dong/catkin_ws/install" --install-scripts="/home/dong/catkin_ws/install/bin"