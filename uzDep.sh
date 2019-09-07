DIR="$( cd "$( dirname "${BASH_SOURCE[0]}" )" >/dev/null 2>&1 && pwd )"
if [ ! -d "$DIR/MiGIO-2" ]; then
    unzip "$DIR/MiGIO-2.zip" -d $DIR
fi
if [ ! -d "$DIR/MiGIO-3" ]; then
    unzip "$DIR/MiGIO-3.zip" -d $DIR
fi
if [ ! -d "$DIR/OpenCVDemo" ]; then
    unzip "$DIR/OpenCVDemo.zip" -d $DIR
fi
if [ ! -d "$DIR/cvblobslib_OpenCV_v6" ]; then
    unzip "$DIR/cvblobslib_OpenCV_v6.zip" -d $DIR
fi
