from realgazebo.vehicle_extras import (
    get_sensor_bridges, build_image_receiver_argv, build_sensor_bridge_argv)


def test_build_image_receiver_argv_node_name_contract():
    argv = build_image_receiver_argv('x500_lidar_2d', 4, 'front', '10.0.0.5')
    # the node name is how image_viewer finds the receiver
    assert '__node:=image_receiver_x500_lidar_2d_4_front' in argv
    assert 'vehicle_type:=x500_lidar_2d' in argv
    assert 'vehicle_id:=4' in argv
    assert 'unreal_ip:=10.0.0.5' in argv
    assert 'rtsp_port:=8554' in argv
    assert 'camera_type:=front' in argv


def test_build_sensor_bridge_argv():
    argv = build_sensor_bridge_argv('/tmp/bridges/a.yaml', 'sensor_bridge_a')
    assert argv[:4] == ['ros2', 'run', 'ros_gz_bridge', 'parameter_bridge']
    assert 'config_file:=/tmp/bridges/a.yaml' in argv


def test_get_sensor_bridges_direct_and_included(tmp_path):
    # included lidar model (like lidar_2d_v2 under the PX4 models dir)
    inc_dir = tmp_path / 'models' / 'lidar_2d_v2'
    inc_dir.mkdir(parents=True)
    (inc_dir / 'model.sdf').write_text(
        '<sdf><model name="lidar_2d_v2"><link name="link">'
        '<sensor name="lidar_2d_v2" type="gpu_lidar"/></link></model></sdf>')

    sdf = tmp_path / 'x500_lidar_2d.sdf'
    sdf.write_text(
        '<sdf><model name="x500_lidar_2d">'
        '<include><uri>model://lidar_2d_v2</uri></include>'
        '<link name="base"><sensor name="cam" type="camera"/></link>'
        '</model></sdf>')

    bridges = get_sensor_bridges(
        'x500_lidar_2d', 4, 'urban', str(sdf), [str(tmp_path / 'models')])

    # camera type is not bridgeable; only the included gpu_lidar maps
    assert len(bridges) == 2  # scan + scan/points
    scan = next(b for b in bridges if b['ros_topic_name'] == '/vehicle5/scan')
    assert scan['gz_topic_name'] == ('/world/urban/model/x500_lidar_2d_4'
                                     '/link/link/sensor/lidar_2d_v2/scan')
    assert scan['direction'] == 'GZ_TO_ROS'


def test_get_sensor_bridges_missing_sdf(tmp_path):
    assert get_sensor_bridges('x500', 0, 'c-track',
                              str(tmp_path / 'nope.sdf')) == []
