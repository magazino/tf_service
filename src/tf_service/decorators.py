# Copyright 2019 Magazino GmbH
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#     https://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.

import functools

import tf2_ros

from tf_service import client_binding


# Decorator for class methods that translates internal exception types to
# corresponding tf2_ros.*Exceptions.
def translate_exceptions(method):
    @functools.wraps(method)
    def translate(obj, *args, **kwargs):
        try:
            return_value = method(obj, *args, **kwargs)
            return return_value
        except client_binding.ConnectivityException as e:
            raise tf2_ros.ConnectivityException(str(e))
        except client_binding.ExtrapolationException as e:
            raise tf2_ros.ExtrapolationException(str(e))
        except client_binding.LookupException as e:
            raise tf2_ros.LookupException(str(e))
        except client_binding.ExtrapolationException as e:
            raise tf2_ros.ExtrapolationException(str(e))
        except client_binding.InvalidArgumentException as e:
            raise tf2_ros.InvalidArgumentException(str(e))
        except client_binding.TimeoutException as e:
            raise tf2_ros.TimeoutException(str(e))
        except client_binding.TransformException as e:
            raise tf2_ros.TransformException(str(e))

    return translate
