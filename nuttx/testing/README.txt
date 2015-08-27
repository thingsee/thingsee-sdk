README
^^^^^^

This folder hosts NuttX unit test framework source files.

INTRODUCTION
^^^^^^^^^^^^

NuttX unit test framework is based on Unity (https://github.com/ThrowTheSwitch/Unity).
The source code bears minimal changes and most of the additional functionality is brought
through the extension.

NUTTX-SPECIFIC ISSUES
^^^^^^^^^^^^^^^^^^^^^

Unity relies on setjmp/longjmp to be available, so a testcase can be aborted. While the
functionality exists in simulator builds, the hardware environment does not have it. To
provide the same baseline we run test bodies and other abortable parts in their own threads.
The main thread blocks until a spawn exits. Thus we create the same context as offered by
setjmp/longjmp.

INTEGRATING WITH CONTINUOUS INTEGRATION
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

Bare Unity does not offer filtered test running or test listing. These are essential for our
test automation realization. The "fixture" extension offers grouping and filtering, but it
is not at the level we want it to be.

There is custom test runner generator derived from generate_test_runner.rb. Its implementation is
in the nuttx/tools/generate_test_project.rb and instructions are in the nuttx/tools/README.txt.

CREATING TESTS
^^^^^^^^^^^^^^

Only test modules are supported. Preferred way is to generate the module using
nuttx/tools/generate_module.rb. More documentation is in docs folder. The project pattern is
the same as e.g. apps/examples/hello.

RUNNING TESTS
^^^^^^^^^^^^^

Tests can be executed both in simulator and in the target. The test application can be built-in
into NSH or provide application entry point.

Test application supports command line interface:

    application [--list] [test [test...]]

        --list - Lists all available testcases from all included modules

Test is identified by its module name and function name.
