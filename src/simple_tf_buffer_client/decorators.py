import tf2_ros

import simple_tf_buffer_server.client


# Decorator for class methods that translates internal exception types to
# corresponding tf2_ros.*Exceptions.
def translate_exceptions(method):
    def translate(obj, *args, **kwargs):
        try:
            method(obj, *args, **kwargs)
        except simple_tf_buffer_server.client.ConnectivityException as e:
            raise tf2_ros.ConnectivityException(str(e))
        except simple_tf_buffer_server.client.ExtrapolationException as e:
            raise tf2_ros.ExtrapolationException(str(e))
        except simple_tf_buffer_server.client.LookupException as e:
            raise tf2_ros.LookupException(str(e))
        except simple_tf_buffer_server.client.ExtrapolationException as e:
            raise tf2_ros.ExtrapolationException(str(e))
        except simple_tf_buffer_server.client.InvalidArgumentException as e:
            raise tf2_ros.InvalidArgumentException(str(e))
        except simple_tf_buffer_server.client.TimeoutException as e:
            raise tf2_ros.TimeoutException(str(e))
        except simple_tf_buffer_server.client.TransformException as e:
            raise tf2_ros.TransformException(str(e))

    return translate
