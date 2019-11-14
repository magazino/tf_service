# simple_tf_buffer_server

Prototype RPC TF buffer server / client implementation based on ROS services.

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

*TODO*

### C++

```cpp
#include "simple_tf_buffer_server/buffer_client.h"

...

tf2_ros::SimpleBufferClient buffer("/simple_tf_buffer_server", node_handle);

// Use it like any other TF2 buffer.
std::string errstr;
if (buffer.canTransform("map", "odom", ros::Time(0), ros::Duration(1), &errstr)) {
  buffer.lookupTransform("map", "odom", ros::Time(0), ros::Duration(1));
}

```

---
## TODO

* recovery for persistent service loss
* handling of all INTERNAL/OTHER error responses
* Python client (binding)
