#天地图API
#http://t0.tianditu.gov.cn/img_w/wmts?SERVICE=WMTS&REQUEST=GetTile&VERSION=1.0.0&LAYER=img&STYLE=default&TILEMATRIXSET=w&FORMAT=tiles&TILEMATRIX={level}&TILEROW={y}&TILECOL={x}&tk=375f936acc7c22013f7b7b96559094a7
#google api
#http://localhost:8080/wmts/gm_layer/gm_grid/{level}/{x}/{y}.png
#容器
#sudo docker run -d --restart=always \
#  -p 8080:8080 \
#  --add-host=host.docker.internal:host-gateway \
#  -e http_proxy=http://host.docker.internal:7890 \
#  -e https_proxy=http://host.docker.internal:7890 \
#  -v /home/ga/mapproxy:/mapproxy \
#  danielsnider/mapproxy

import launch
import launch.actions
import launch.substitutions
import launch_ros.actions


def generate_launch_description():
    return launch.LaunchDescription([
        launch_ros.actions.Node(
            package="mapviz",
            executable="mapviz",
            name="mapviz",
        ),
        launch_ros.actions.Node(
            package="swri_transform_util",
            executable="initialize_origin.py",
            name="initialize_origin",
            parameters=[
                {"local_xy_frame": "map"},
                {"local_xy_origin": "swri"},
                {"local_xy_origins": """[
                    {"name": "swri",
                        "latitude": 28.2260675,
                        "longitude": 112.895094,
                        "altitude": 56.0,
                        "heading": 0.0}
                ]"""}
            ]
        ),
        launch_ros.actions.Node(
            package="tf2_ros",
            executable="static_transform_publisher",
            name="swri_transform",
            arguments=["0", "0", "0", "0", "0", "0", "map", "origin"]
        ),
        launch_ros.actions.Node(
            package="tf2_ros",
            executable="static_transform_publisher",
            name="gps_transform",
            arguments=["0", "0", "0", "0", "0", "0", "map", "gps"]
        ),
        launch_ros.actions.Node(
            package="tf2_ros",
            executable="static_transform_publisher",
            name="path_transform",
            arguments=["0", "0", "0", "0", "0", "0", "map", "path"]
        )
    ])
