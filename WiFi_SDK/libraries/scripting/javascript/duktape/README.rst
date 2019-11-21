=======
Duktape
=======

.. contents:: :backlinks: top

Overview
========

Duktape is a embeddable JavaScript (JS), or more formerly as Ecmacscript
E5/E5.1, engine with a tiny footprint. The code is meant to be portable and
require as little platform-specific code as possible. As such, though, Duktape
in it's initial form is very self-contained, i.e., no ability to control GPIOs,
WiFi, etc. However, additional functionality can be easily added via modules
written either in JS or C code.

One important thing to note is that Duktape does not handle DOM (Document
Object Model) like the JS engines do in web browsers. As such, it is not
possible to use Duktape to run JS that alters HTML5 objects. It is possible to
support DOM handling via JS modules like jsdom (which requires Node.js support)
or custom-written modules/objects.

More information can be found on the official website (http://duktape.org/).

Source Code
===========

The Duktape source code is located in the ``src`` directory, under a
``verX.Y.Z`` directory, where ``X.Y.Z`` corresponds to the Duktape release
version. The ``DUKTAPE_VER`` variable in ``duktape.mk`` controls which version
gets built. The source code comes from the official release tarballs found on
the official website.

Effort has been made to minimize changes to the source code. In order to
highlight the additions/modifications, all changes are highlighted with the
keyword ``WICED``.

Configuration
-------------

Duktape provides a nifty configuration tool for tuning functionality, code size,
RAM usage, platform-specific code, and more. The tool is written in python and
is located under the ``tools`` directory.

The default configuration that comes with the release tarball is produced as
follows:::

    python tools/configure.py --line-directives \
        --emit-legacy-feature-check --emit-config-sanity-check \
        --output-directory src

The resulting build of Duktape is located in the ``src`` directory at the root
of the release source code. The code is not optimized for low memory, nor small
code size.

Building for WICED requires some modifications to the resulting ``duk_config.h``
of the build. By using the "fixup" feature of the configuration tool, the
needed modifications for WICED can be kept the separate file located at
``src/wiced_duk_config_fixup.h``. The command to configure the build is:::

    python tools/configure.py --line-directives \
        --emit-legacy-feature-check --emit-config-sanity-check \
        --fixup-file=../wiced_duk_config_fixup.h \
        --output-directory=src-WICED

Note the resulting build of Duktape is located in the ``src-WICED`` directory
at the root of the release source code.

Duktape has an example build for low-memory platforms. It sacrifices some
features for smaller code size (~52 KiB less)  and memory usage. A build of
this is done with the following command:::

    python tools/configure.py --line-directives \
        --emit-legacy-feature-check --emit-config-sanity-check \
        --option-file=config/examples/low_memory.yaml \
        --fixup-file=../wiced_duk_config_fixup.h \
        --output-directory=src-WICED-low-memory

The resuling build of Duktape is located in the ``src-WICED-low-memory``
directory. It can be selected in the ``duktape.mk`` via the
``WICED_DUKTAPE_OPT_LOW_MEMORY_BUILD`` flag.


v2.0.0 Notes
------------

- The ``duk__utf8_emit_repl`` and ``duk__utf8_encode_char`` were defined but
  not used when ``DUK_USE_ENCODING_BUILTINS`` is false, resulting in a
  compilation error (``-Werror=unused-function``). Conditional compilation
  preproccesors were added to avoid the error.

WICED Integration
=================

The Duktape engine is integrated into WICED as a library. The library provides
the following functionality:

- Duktape heap management
- Duktape/C modules and objects to extend functionality
- Module resolving and loading
- Filesystem support
- WICED resources support for embeddeding modules/scripts into firmware image

Objects and Modules
-------------------

On its own, Duktape does not allow for much system interaction, save for some
simple console logging. However, with custom written objects and modules, it is
possible to expose system functionality to scripts.

Some objects and modules are included as part of this library. They can be
optionally compiled in or out via the ``WICED_DUKTAPE_OPT_MODULE_*`` or
``WICED_DUKTAPE_OPT_OBJECT_*`` flags in ``duktape.mk``. The included objects
and modules are written in C, and thus can call WICED APIs.

This library's module resolving and loading mechanism also supports the loading
of JS modules in addition to the aforementioned Duktape/C modules. Node.js style
modules are supported, as long as all dependencies are met, i.e., some modules
require the libuv platform support library, which is not yet available.

Programming Guidelines
----------------------

When writing new Duktape/C modules and objects, it is important to be wary of
a few facts:

- Each Duktape heap (the library only currently supports one) can only be
  accessed by one native C thread at any time; there are no internal Duktape
  locks that save states
- Duktape requires a LOT of stack space to run. It is recommended to use the
  DUKTAPE_WORKER_THREAD for any out-of-band Duktape/C code
- When trying to callback into Duktape from an event handler, schedule a worker
  function to run via the ``wiced_duktape_schedule_work()`` API
- If that's not possible (due to memory restrictions or whatnot), always use the
  ``wiced_duktape_get_control()`` API to gain control of the Duktape heap
  before doing any modifications. Then use the ``wiced_duktape_put_control()``
  API to release control of the Duktape heap. WARNING: those APIs may make a
  blocking call to obtain the semaphore
- It is a good idea to use comments to keep track of the Duktape call stack;
  the notation is ``/* -> [...] */``, where the stack values ares named inside
  the brackets

Usage
=====

The best way to illustrate the use of this library is through an example. The
Duktape test app (``apps/test/duktape``) is a simple console app that
initializes this library, and allows the users to use commands from the Duktape
console commands (``libraries/utilities/command_console/duktape``). The console
commands are simple wrappers around this libraries eval API. The app also
illustrates how to add modules and scripts to the firmware.

Tests
=====

Duktape API
-----------

Duktape has a set of tests that test the Duktape API. The tests are individual
C programs, each testing a particular set of features. The tests are designed
to run on a PC with node.js support. Each test is compiled individually into an
executable, and run. The results from ``stdout`` are then compared to the
expected ``stdout``, which is stored as comments inside the C source code files.
Differences will highlight any bugs/issues in the Duktape engine's.

For the WICED platforms, all the tests are compiled into the firmware, and run
one-after-another, each with a fresh Duktape heap. The results of the tests are
printed to the console. However, since the expected results are over 200 KiB in
size, it does not make sense to include it into the firmware. As such, comparing
the results would require copying the output from the WICED platform's console
and comparing them to the expected results on a PC.

The Duktape API tests live under ``tests/api`` subdirectory of the Duktape
source code directory. The following bash commands are run on the tests to make
them compatible with the WICED build environment:::

    export TESTS_HEADER="wiced_duktape_tests_api_internal.h"; \
    export TESTS_LIST_H="wiced_duktape_tests_api_list.h"; \
    export TESTS_LIST_C="wiced_duktape_tests_api_list.c"; \
    export TESTS_RESULTS="wiced_duktape_tests_api_expected_results_official.txt"; \
    rm ${TESTS_RESULTS} ${TESTS_LIST_H} ${TESTS_LIST_C}; \
    echo -e "#include \"duktape.h\"\n" >> ${TESTS_LIST_H}; \
    echo -e "typedef struct\n{\n    const char* name;\n    void (*func)( duk_context* );\n} wiced_duktape_tests_api_entry;\n" >> ${TESTS_LIST_H}; \
    echo -e "extern wiced_duktape_tests_api_entry wiced_duktape_tests_api_list[];\n" >> ${TESTS_LIST_H}; \
    echo -e "#include \"${TESTS_LIST_H}\"\n" >> ${TESTS_LIST_C}; \
    echo -e "wiced_duktape_tests_api_entry wiced_duktape_tests_api_list[] =\n{">> ${TESTS_LIST_C}; \
    for FILE in `ls test-*.c | sort -V`; \
    do \
        TEST_NAME=`basename ${FILE} .c`; \
        FUNCTION="${TEST_NAME#test-}"; \
        FUNCTION="wiced_duktape_tests_api_${FUNCTION//-/_}"; \
        sed -i -e '1 i #if defined(WICED)\n#include "'${TESTS_HEADER}'"\n#endif\n' \
               -e 's/\(^void [^test|wiced].*\)/#if defined(WICED)\nstatic \1\n#else\n\1\n#endif/g' \
               -e 's/^void test\(.*\)/#if defined(WICED)\nvoid '${FUNCTION}'\1\n#else\nvoid test\1\n#endif/g' ${FILE}; \
        echo "+++ ${TEST_NAME}" >> ${TESTS_RESULTS}; \
        echo "    { \"${TEST_NAME}\", ${FUNCTION} }," >> ${TESTS_LIST_C}; \
        echo "extern void ${FUNCTION}(duk_context *ctx);" >> ${TESTS_LIST_H}; \
        sed -n -e '/\/\*===/,/===\*\//{/\/\*===\|===\*\//!p}' ${FILE} >> ${TESTS_RESULTS}; \
    done; \
    echo -e "    { NULL, NULL }\n};" >> ${TESTS_LIST_C};

Details of the commands:

- For each test file:

  - Include the WICED API test header ``wiced_duktape_tests_api_internal.h``
  - Rename the main test function (``void test(duk_context *ctx)``) with its
    filename to give it a unique name
  - Make sure all functions other than the main test function is declared with
    the ``static`` keyword

- Generate an expected results file
  ``wiced_duktape_tests_api_expected_results_official.txt`` from the expected
  results comments in each of the test files
- Generate a struct array of tests to ``wiced_duktape_tests_api_list.h`` and
  ``wiced_duktape_tests_api_list.c``


v2.0.0 Notes
************

A number of tests are omitted for WICED. Some tests are officially skipped by
the author of Duktape, and others are not compatible with WICED. The tests that
are skipped are suffixed with ``.skipped`` and not included in the WICED build.

The following is a table of all the omitted tests and the reason:

======================================  ========================================
Test Case                               Reason for Omitting
======================================  ========================================
test-all-public-symbols                 Duktape assumes ``va_list`` is ``typedef
                                        char*``, but WICED is ``typedef
                                        __builtin_va_list``
test-bug-peval-pcompile-no-file         Officially skipped (``"skip": true``)
test-compile-file                       Officially skipped (``"skip": true``)
test-def-prop
test-dev-cmodule-guide                  Officially skipped (``"skip": true``)
test-dev-error-fileline-blame-gh455
test-dev-rom-builtins-1                 Officially skipped (``"skip": true``)
test-dump-load-basic
test-dump-load-fastint                  Depends on disabled ``FASTINT`` feature
test-eval-file                          Officially skipped (``"skip": true``)
test-external-buffer
test-fatal-return
test-fatal
test-get-prop-desc
test-inspect-value
test-logging                            Officially skipped (``"skip": true``)
test-push-buffer-object-disabled        Officially skipped (``"skip": true``)
test-push-buffer-object
test-push-sprintf
test-push-vsprintf
test-put-prop
test-suspend-resume-pthread             Requires unavailable pthread
======================================  ========================================

Of the tests that do run, some output is different than the official expected
results. Also, some tests are known to fail. For each case that is different,
an analysis was done to find the cause of the difference:

=================================================== ============================
Test Case                                           Reason for Difference
=================================================== ============================
test-debugger-notify:test_notify_invalid_count1     Not enough memory for alloc
test-dev-string-intern-side-effect:test_side_effect Not enough memory for alloc
test-error:(various)                                Modifications to Duktape
                                                    source code caused line
                                                    numbers to shift
test-get-now                                        WICED platform lacks RTC
test-get-pointer                                    WICED prints ``0x0`` instead
                                                    of ``(nil)``
test-put-func-num-list:test_1                       Same issue when run on
                                                    Linux
test-require-pointer:test_1                         WICED prints ``0x0`` instead
                                                    of ``(nil)``
test-to-number:test_1                               Officially known issue
test-to-pointer:test_1                              WICED prints ``0x0`` instead
                                                    of ``(nil)``
test-types                                          WICED prints ``0x0`` instead
                                                    of ``(nil)``
test-xcopy-xmove:various                            Not enough memory for alloc
=================================================== ============================

After analysis, the expected results on a WICED platform are saved to the
``wiced_duktape_tests_api_expected_results_wiced.txt`` file, which can be used
for testing.

TODOs
=====

Here's a list of items that would be nice to add to the WICED Duktape library
at some point in the future:

- Add some sort of debug macro to every Duktape/C function to make sure that the
  Duktape call stack value (from ``duktape_get_top()``) is expected at the end
  of the function call
- Add new API to do timed scheduled work, e.g.,
  ``wiced_duktape_schedule_timed_work()``
- Add new API to generate an unique reference for an object, which includes the
  Duktape context and ``'this'`` pointer; the reference can be used when
  scheduling work
- Support multiple Duktape heaps and/or threads
- Modify ``wiced_duktape_schedule_work`` to take in a variable number of
  arguments

