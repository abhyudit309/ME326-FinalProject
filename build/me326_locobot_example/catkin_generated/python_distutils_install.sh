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

echo_and_run cd "/home/connor/ME326-FinalProject/src/me326_locobot_example"

# ensure that Python install destination exists
echo_and_run mkdir -p "$DESTDIR/home/connor/ME326-FinalProject/install/lib/python3/dist-packages"

# Note that PYTHONPATH is pulled from the environment to support installing
# into one location when some dependencies were installed in another
# location, #123.
echo_and_run /usr/bin/env \
    PYTHONPATH="/home/connor/ME326-FinalProject/install/lib/python3/dist-packages:/home/connor/ME326-FinalProject/build/me326_locobot_example/lib/python3/dist-packages:$PYTHONPATH" \
    CATKIN_BINARY_DIR="/home/connor/ME326-FinalProject/build/me326_locobot_example" \
    "/usr/bin/python3" \
    "/home/connor/ME326-FinalProject/src/me326_locobot_example/setup.py" \
     \
    build --build-base "/home/connor/ME326-FinalProject/build/me326_locobot_example" \
    install \
    --root="${DESTDIR-/}" \
    --install-layout=deb --prefix="/home/connor/ME326-FinalProject/install" --install-scripts="/home/connor/ME326-FinalProject/install/bin"
