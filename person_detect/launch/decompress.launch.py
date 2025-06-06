# Software License Agreement (BSD)
#
# @author    Roni Kreinin <rkreinin@clearpathrobotics.com>
# @copyright (c) 2023, Clearpath Robotics, Inc., All rights reserved.
#
# Redistribution and use in source and binary forms, with or without
# modification, are permitted provided that the following conditions are met:
# * Redistributions of source code must retain the above copyright notice,
#   this list of conditions and the following disclaimer.
# * Redistributions in binary form must reproduce the above copyright notice,
#   this list of conditions and the following disclaimer in the documentation
#   and/or other materials provided with the distribution.
# * Neither the name of Clearpath Robotics nor the names of its contributors
#   may be used to endorse or promote products derived from this software
#   without specific prior written permission.
#
# THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
# AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
# IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
# ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE
# LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
# CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
# SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
# INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
# CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
# ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
# POSSIBILITY OF SUCH DAMAGE.
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    namespace = LaunchConfiguration('namespace')
    out_raw = LaunchConfiguration('out_raw')
    in_compressed = LaunchConfiguration('in_compressed')
    # compress = LaunchConfiguration('compress')

    arg_namespace = DeclareLaunchArgument(
        'namespace',
        default_value=''
    )

    arg_in_compressed = DeclareLaunchArgument(
        'in_compressed',
        default_value='/oak/rgb/image_raw/dynamic/compressed'
    )

    arg_out_raw = DeclareLaunchArgument(
        'out_raw',
        default_value='/oak/rgb/image_raw/decompressed'
    )

    compressed_transport_node = Node(
        name='compressed_to_image_raw',
        namespace=namespace,
        package='image_transport',
        executable='republish',
        remappings=[
            ('in/compressed', in_compressed),
            ('out', out_raw),
        ],
        # parameters=[{'out.jpeg_quality': compress}],
        arguments=['compressed', 'raw'],
    )

    ld = LaunchDescription()
    ld.add_action(arg_namespace)
    ld.add_action(arg_in_compressed)
    ld.add_action(arg_out_raw)
    ld.add_action(compressed_transport_node)
    return ld