# Copyright 2025 Magazino GmbH
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

import tf2_ros
from tf2_msgs.msg import TF2Error


def throw_on_error(status: TF2Error) -> None:
    if status.error == TF2Error.CONNECTIVITY_ERROR:
        raise tf2_ros.ConnectivityException(status.error_string)
    if status.error == TF2Error.EXTRAPOLATION_ERROR:
        raise tf2_ros.ExtrapolationException(status.error_string)
    if status.error == TF2Error.INVALID_ARGUMENT_ERROR:
        raise tf2_ros.InvalidArgumentException(status.error_string)
    if status.error == TF2Error.LOOKUP_ERROR:
        raise tf2_ros.LookupException(status.error_string)
    if status.error == TF2Error.TIMEOUT_ERROR:
        raise tf2_ros.TimeoutException(status.error_string)
    if status.error == TF2Error.TRANSFORM_ERROR:
        raise tf2_ros.TransformException(status.error_string)
