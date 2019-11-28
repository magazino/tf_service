# tf_service

TF buffer server / client implementation based on ROS services.

Implemented in C++ and Python bindings.

---

![diagram](diagram.png)

---

### Installation

#### Package

*todo*

#### Testing

```
magclone tf_service
git -C tf_service/ submodule update --init
ci
```

---
## server

```
rosrun tf_service server --num_threads 10
```

Starts the TF server node. The number of threads limits the number of request queues the server can handle in parallel.

---
## client

The client implements the normal TF2 buffer interface.

### Python

The Python bindings are wrapped in a standard `tf2_ros.BufferInterface`:

```python

import rospy                                                         
import tf_service

buffer = tf_service.BufferClient("/tf_service")

# Use it like any other TF2 buffer.
if buffer.can_transform("map", "odom", rospy.Time(0), rospy.Duration(5)):
    buffer.lookup_transform("map", "odom", rospy.Time(0), rospy.Duration(1))
```

### C++

Implements a standard `tf2_ros::BufferInterface`:

```cpp
#include "tf_service/buffer_client.h"

// ...

tf_service::BufferClient buffer("/tf_service");

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
roslaunch tf_service benchmark_py.launch use_old_version:=true
```
with the new implementation from this package:
```
roslaunch tf_service benchmark_py.launch use_old_version:=false
```

## Limitations

Service calls are blocking and should be "short".
To reduce the risk that clients can brick the server, timeouts greater than 10 seconds are not allowed.

If you need very long timeouts, you might be better off with the old action-client based implementation from TF2.
