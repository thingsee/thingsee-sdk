The NuttX build sequence will install google-test unit test applications into this folder.

The executables are self-contained and link statically against google-test libraries. The
build environment has to have the libgtest.a and libgtest_main.a available. The path is
configurable and by default is set to /usr/lib. This is the location, where they are installed,
from libgtest-dev package.
