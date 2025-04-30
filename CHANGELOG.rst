^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
Changelog for package tf_service
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

0.8.0 (2025-04-30)
------------------
* Add maintainer

0.7.0 (2022-07-14)
------------------
* Log the name of the service that we wait for.
* Test keepalive in Python rostest.
* Remove type hints for Melodic compatibility (for now...)
* Implement optional 'keepalive'.
* Loh message when reconnecting asynchronously.
* Add async reconnection method to client.
  Can be called by application code to reconnect without blocking if the
  service connection was lost.

0.6.3 (2021-08-11)
------------------
* Remove travis config and update build badge.
* Update main.yml
* Configure GitHub actions CI.

0.6.2 (2021-05-14)
------------------
* Fix leaking C++->Python cast of TransformStamped.header.stamp (`#17 <https://github.com/magazino/tf_service/issues/17>`_)
  The manually call to our custom type_caster is missing an ownership
  transfer, so the refcount is wrong. Stealing would be correct in this case:
  ```c++
  tf.attr("header").attr("stamp") =
  ::pybind11::reinterpret_steal<object>(
  type_caster<ros::Time>().cast(src.header.stamp, policy, parent));
  ```
  Overall, this is not necessary because `pybind11::cast` automatically
  takes care of this and we can just use that instead.
  Closes: `#16 <https://github.com/magazino/tf_service/issues/16>`_
* Mention legacy mode in README.

0.6.1 (2020-10-01)
------------------
* Default --num_threads to number of cores.
  Also, log the proper final number instead of 0 when starting up.
* Update README.md

0.6.0 (2020-09-07)
------------------
* Set and log legacy server namespace in server_main.cc
* Parameterize gtest to test also the legacy mode.
* Allow running a legacy tf2::BufferServer in the same node.
  Useful to avoid two separate nodes when both are used.

0.5.0 (2020-08-11)
------------------
* Drop intermediate directory when installing headers.
  `#include "tf_service/client.h"`
  in client code instead of
  `#include "tf_service/tf_service/client.h"`

0.4.0 (2020-07-29)
------------------
* Rename --debug to --frames_service

0.3.0 (2020-06-24)
------------------
* Show supported distros in README.md
* Use explicit relative import for Python 3 support.
* Add Dockerfile for Noetic and run it in Travis CI.

0.2.1 (2020-01-09)
------------------
* Kick out pybind11 submodule.
* Compatibility with old pybind11 from Ubuntu.
  Luckily this only affects two smaller things (different macro signature
  and lambda support). It's also forward-compatible with the current
  upstream of pybind11, although with a deprecation warning.
  Note: documentation for this old version can be fetched by installing
  pybind11-doc (then in /usr/share/pybind11-doc).

0.2.0 (2020-01-08)
------------------
* Remove unnecessary smart pointers.
* Preserve original docstrings of decorated Python methods.
  Without `functools.wraps` the signature of the decorator would be shown
  in help functions.
* Add configurable server options.
* Clean shutdown of Python wrapper.
  Removes the hacky signal handler in C++ glue code, it makes way more
  sense to do this in Python.
* Add Travis badge to README.md
* Add .travis.yml
* Add Dockerfile for melodic.
* Add missing tf2_geometry_msgs dependency.
* Use https for pybind11 submodule.
* Add Apache license terms.
* Replace Magazino magic by catkin commands.
* Extend package documentation.
* Catch CLI exceptions.
* Further simplify names.
* Add Python client rostest.
* Fix wrong include.
* Rename package to "tf_service".
  Less chars and easier to find togeter with other tf* packages.
* Clean up module names and shutdown.
* Cleanup CMakeLists.txt
* Add rostest for client.
* Fix missing return value in decorator.
* Disable SIGINT handler in binding-internal node.
* Add waitForServer to C++ client.
* Update README.md
* Add diagram to README.
* Cleanup C++ namespaces.
* Add info about max timeout to README.
* Add 'advanced' flag to service requests.
* Add simple Python benchmark launch file.
* Translate exceptions to Python tf2_ros.*Exceptions.
* Use -1 timeout by default in Python wait_for_server.
* Add Python wrapper with proper tf2_ros.BufferInterface
* Improve Python module name and docstrings.
* First working Python bindings with pybind11
* Pin pybind11 to v2.4.3
* Add pybind11 submodule.
* Simplify client constructor.
* Use existing tf2_msgs::TF2Error
* Update README.md
* Add comments regarding mutex.
* Reconnection method and consistent camel case.
* Remove wrong num_threads variable.
* Add README.md
* Cleanup namespaces and add --num_threads option.
* Mutable service client members for client.
* Use exception type annotations in status responses.
* First prototype.
