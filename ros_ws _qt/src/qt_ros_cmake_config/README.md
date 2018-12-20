# qt_ros
A standalone bare-bones repository for creating ros applications with qt that doesn't rely on rqt, using cmake.

ros: http://www.ros.org/

qt:  https://www.qt.io/

The general structure is:

```
  qt_ros/
  |- src/            (*.cpp goes here)
       |- main.cpp   (the main entry point into a QApplication, contains ros::init())
  |- include/        (*.h goes here)
  |- resources/      (*.ui goes here)
       |- qrc/       (*.qrc goes here)
  |- CMakeLists.txt
  |- package.xml
```

This repository is designed to be cloned into the src directory of a catkin
workspace. That is, a workspace created by running the command `catkin_init_workspace`.
see here: http://wiki.ros.org/catkin/Tutorials/create_a_workspace

The CmakeLists.txt file is configured to automatically compile all files
into a shared library (lib_qt_ros.so) which is linked to the executable created
from main.cpp, so long as you put the appropriate files in their respective directory.
Don't move main.cpp unless you want to edit CMakelists.txt yourself
(you may want to edit if you plan on renaming the library, which you should).

For those new to qt don't trip if when examining your headers you see `#include "ui_<my_file>.h"`
and it doesn't exist yet. Just run `catkin_make` from the root of your catkin workspace and
the qt build system will create them for you in the auto generated `build` directory.

Oh and it contains a MainWindow.[ui | h | cpp] example complete with ros node initialization
and publishing. Just `rosrun turtlesim turtlesim_node` along with this program.
