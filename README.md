# simple_tf_buffer_server

TF buffer server / client implementation based on ROS services.

Implemented in C++ and Python bindings.

---

![diagram](diagram.png)

---

### Installation

```
magclone simple_tf_buffer_server
cd simple_tf_buffer_server
git submodule init
git submodule update
ci .
```

---
## server

```
rosrun simple_tf_buffer_server server --num_threads 10
```

Starts the TF server node. The number of threads limits the number of request queues the server can handle in parallel.

---
## client

The client implements the normal TF2 buffer interface.

### Python

The Python bindings are wrapped in a standard `tf2_ros.BufferInterface`:

```python

import rospy                                                         
from simple_tf_buffer_client import SimpleBufferClient

buffer = SimpleBufferClient("/simple_tf_buffer_server")

# Use it like any other TF2 buffer.
if buffer.can_transform("map", "odom", rospy.Time(0), rospy.Duration(5)):
    buffer.lookup_transform("map", "odom", rospy.Time(0), rospy.Duration(1))
```

### C++

Implements a standard `tf2_ros::BufferInterface`:

```cpp
#include "simple_tf_buffer_server/buffer_client.h"

namespace sbs = simple_tf_buffer_server;

// ...

sbs::SimpleBufferClient buffer("/simple_tf_buffer_server");

// Use it like any other TF2 buffer.
std::string errstr;
if (buffer.canTransform("map", "odom", ros::Time(0), ros::Duration(1), &errstr)) {
  buffer.lookupTransform("map", "odom", ros::Time(0), ros::Duration(1));
}
```

## Benchmarks

The Python client implementation is much more efficient than the old `tf2_ros.BufferClient`.

The test scenario are 5 clients doing requests at 10Hz. Compare the resource consumption of:
```
roslaunch simple_tf_buffer_server benchmark_py.launch use_old_version:=true
```
with the new implementation from this package:
```
roslaunch simple_tf_buffer_server benchmark_py.launch use_old_version:=false
```

## Limitations

Service calls are blocking and should be "short".
To reduce the risk that clients can brick the server, timeouts greater than 10 seconds are not allowed.

If you need very long timeouts, you might be better off with the old action-client based implementation from TF2.
