import types
import signal
import subprocess

import pytest

from realgazebo import backends
from realgazebo.backends import SubprocessBackend, make_backend
from realgazebo.entity import Entity
from realgazebo.yaml_config import VehicleSpec


class FakeProc:
    def __init__(self, pid=4242):
        self.pid = pid
        self.waited = False
        self.extras = []   # composite-handle duck-typing for plain procs

    def wait(self, timeout=None):
        self.waited = True


def test_launch_renders_creates_and_boots_px4(monkeypatch, tmp_path):
    calls = {'popen': []}
    monkeypatch.setattr(backends, 'render_sdf',
                        lambda t, ip, port: f'/tmp/models/{t}.sdf')
    monkeypatch.setattr(backends, 'get_sensor_bridges',
                        lambda *a, **kw: [])
    monkeypatch.setattr(backends.subprocess, 'run',
                        lambda argv, **kw: calls.setdefault('create', argv))
    fake = FakeProc()

    def fake_popen(argv, **kwargs):
        calls['popen'].append((argv, kwargs))
        return fake

    monkeypatch.setattr(backends.subprocess, 'Popen', fake_popen)
    monkeypatch.setattr(
        backends.threading, 'Thread',
        lambda **kw: types.SimpleNamespace(start=lambda: None))

    airframes = tmp_path / 'ROMFS/px4fmu_common/init.d-posix/airframes'
    airframes.mkdir(parents=True)
    (airframes / '4001_gz_x500').write_text('')
    spec = VehicleSpec(2, 'x500', str(tmp_path), (1.0, 2.0, 0.5, 0.0),
                       entity=Entity('x500', 2, 0))

    handle = SubprocessBackend().launch(
        spec, 'c-track', (1.0, 2.0, 0.5), (0.0, 0.0, 0.0), '10.0.0.5', 5005)

    assert handle.px4 is fake
    assert handle.pid == fake.pid
    assert calls['create'][:4] == ['ros2', 'run', 'ros_gz_sim', 'create']

    px4_argv, px4_kwargs = calls['popen'][0]
    assert px4_argv[-2:] == ['-i', '2']
    assert px4_kwargs['cwd'].endswith('build/px4_sitl_default')
    assert px4_kwargs['start_new_session'] is True

    # parity extras: one camera receiver per VEHICLE_CAMERAS entry, each in
    # its own session (cannot join PX4's group across sessions)
    receivers = [(a, k) for a, k in calls['popen'][1:]
                 if 'image_receiver_node' in a]
    assert len(receivers) == 2  # x500: front + bottom
    names = {a[a.index('-r') + 1] for a, _ in receivers}
    assert names == {'__node:=image_receiver_x500_2_front',
                     '__node:=image_receiver_x500_2_bottom'}
    assert all(k.get('start_new_session') is True for _, k in receivers)
    assert len(handle.extras) == 2


def test_launch_starts_sensor_bridge_for_lidar(monkeypatch, tmp_path):
    calls = []
    monkeypatch.setattr(backends, 'render_sdf',
                        lambda t, ip, port: f'/tmp/models/{t}.sdf')
    monkeypatch.setattr(
        backends, 'get_sensor_bridges',
        lambda *a, **kw: [{'ros_topic_name': '/vehicle3/scan'}])
    monkeypatch.setattr(backends.subprocess, 'run', lambda argv, **kw: None)
    monkeypatch.setattr(backends.subprocess, 'Popen',
                        lambda argv, **kw: calls.append(argv) or FakeProc())
    monkeypatch.setattr(
        backends.threading, 'Thread',
        lambda **kw: types.SimpleNamespace(start=lambda: None))

    airframes = tmp_path / 'ROMFS/px4fmu_common/init.d-posix/airframes'
    airframes.mkdir(parents=True)
    (airframes / '4013_gz_x500_lidar_2d').write_text('')
    spec = VehicleSpec(2, 'x500_lidar_2d', str(tmp_path), (0.0, 0.0, 0.0, 0.0),
                       entity=Entity('x500_lidar_2d', 2, 5))

    SubprocessBackend().launch(
        spec, 'urban', (0, 0, 0), (0, 0, 0), 'h', 5005)

    bridge_argvs = [a for a in calls if 'parameter_bridge' in a]
    assert len(bridge_argvs) == 1
    assert '__node:=sensor_bridge_x500_lidar_2d_2' in bridge_argvs[0]


def test_kill_terminates_process_group(monkeypatch):
    events = []
    monkeypatch.setattr(backends.os, 'getpgid', lambda pid: 999)
    monkeypatch.setattr(backends.os, 'killpg',
                        lambda pgid, sig: events.append((pgid, sig)))
    proc = FakeProc()
    SubprocessBackend().kill(proc)
    assert events == [(999, signal.SIGTERM)]
    assert proc.waited


def test_kill_composite_handle_reaps_extras_too(monkeypatch):
    import types as _types
    events = []
    monkeypatch.setattr(backends.os, 'getpgid', lambda pid: pid)  # pgid = pid
    monkeypatch.setattr(backends.os, 'killpg',
                        lambda pgid, sig: events.append(pgid))
    px4, recv1, recv2 = FakeProc(pid=10), FakeProc(pid=20), FakeProc(pid=30)
    handle = _types.SimpleNamespace(pid=10, px4=px4, extras=[recv1, recv2])
    SubprocessBackend().kill(handle)
    assert events == [10, 20, 30]


def test_alive_uses_px4_of_composite_handle():
    import types as _types
    px4 = FakeProc()
    px4.poll = lambda: None
    handle = _types.SimpleNamespace(pid=px4.pid, px4=px4, extras=[])
    assert SubprocessBackend().alive(handle) is True
    px4.poll = lambda: 0
    assert SubprocessBackend().alive(handle) is False


def test_kill_escalates_to_sigkill_on_timeout(monkeypatch):
    events = []
    monkeypatch.setattr(backends.os, 'getpgid', lambda pid: 999)
    monkeypatch.setattr(backends.os, 'killpg',
                        lambda pgid, sig: events.append(sig))
    proc = FakeProc()

    def timeout_wait(timeout=None):
        raise subprocess.TimeoutExpired('px4', timeout)

    proc.wait = timeout_wait
    SubprocessBackend().kill(proc)
    assert events == [signal.SIGTERM, signal.SIGKILL]


def test_kill_none_is_noop():
    SubprocessBackend().kill(None)  # must not raise


def test_kill_already_gone_process(monkeypatch):
    def raise_lookup(pid):
        raise ProcessLookupError

    monkeypatch.setattr(backends.os, 'getpgid', raise_lookup)
    SubprocessBackend().kill(FakeProc())  # must not raise


def test_make_backend():
    assert isinstance(make_backend('subprocess'), SubprocessBackend)
    with pytest.raises(ValueError):
        make_backend('lxc')


class FakeDockerClient:
    def __init__(self):
        self.calls = []
        self.created = {}
        self.running = {}     # container_id -> bool (for inspect)
        self.listing = []     # entries for list_containers

    def create_container(self, name, config):
        self.calls.append(('create', name))
        self.created[name] = config
        return 'cid-' + name

    def connect_network(self, network, container_id, ipv4=None):
        self.calls.append(('connect', network, container_id, ipv4))

    def start_container(self, container_id):
        self.calls.append(('start', container_id))

    def stop_container(self, container_id, timeout=10):
        self.calls.append(('stop', container_id))

    def remove_container(self, container_id, force=True):
        self.calls.append(('remove', container_id))

    def inspect_container(self, container_id):
        if container_id not in self.running:
            raise RuntimeError('no such container')
        return {'State': {'Running': self.running[container_id]}}

    def list_containers(self, all_states=True):
        return self.listing


def _docker_backend(client):
    from realgazebo.backends import DockerBackend
    return DockerBackend(
        image='aware4docker/realgazebo:1.2',
        px4_path='/home/user/realgazebo/RealGazebo-PX4',
        gazebo_network='gazebo-network',
        vehicle_network='vehicle-network',
        mavlink_gcs_ip='172.17.0.1',
        roster_fn=lambda: ['x500_0'],
        client=client,
    )


def test_docker_launch_assembles_compose_equivalent_container():
    client = FakeDockerClient()
    spec = VehicleSpec(2, 'x500', '/home/user/realgazebo/RealGazebo-PX4',
                       (1.0, 2.0, 0.5, 0.3))
    handle = _docker_backend(client).launch(
        spec, 'c-track', (1.0, 2.0, 0.5), (0.0, 0.0, 0.3),
        'host.docker.internal', 5005)

    assert handle == 'cid-vehicle_2'
    # order: create -> connect isolated DDS net -> start
    assert [c[0] for c in client.calls] == ['create', 'connect', 'start']
    assert client.calls[1] == (
        'connect', 'vehicle-network', 'cid-vehicle_2', '172.30.0.12')

    config = client.created['vehicle_2']
    env = dict(e.split('=', 1) for e in config['Env'])
    assert env['GZ_IP'] == '172.20.0.12'
    assert env['GZ_PARTITION'] == 'realgazebo'
    assert env['FASTRTPS_DEFAULT_PROFILES_FILE'] == \
        '/tmp/dds_profiles/px4_participant_2.xml'
    assert config['HostConfig']['PortBindings'] == {
        '18572/udp': [{'HostPort': '18572'}]}
    assert config['HostConfig']['Privileged'] is True
    assert config['NetworkingConfig']['EndpointsConfig'][
        'gazebo-network']['IPAMConfig']['IPv4Address'] == '172.20.0.12'
    cmd = config['Cmd'][2]
    assert 'vehicle.launch.py' in cmd
    assert 'instance_id:=2' in cmd
    assert 'vehicle_type:=x500' in cmd
    assert 'spawnpoint:=1.0,2.0,0.5,0.3' in cmd
    assert 'unreal_ip:=host.docker.internal' in cmd
    # roster = existing actives + the new vehicle itself
    assert 'vehicle_models:=x500_0,x500_2' in cmd


def test_docker_launch_cleans_up_on_start_failure():
    client = FakeDockerClient()

    def boom(container_id):
        raise RuntimeError('start failed')

    client.start_container = boom
    spec = VehicleSpec(2, 'x500', '/px4', (0.0, 0.0, 0.0, 0.0))
    with pytest.raises(RuntimeError):
        _docker_backend(client).launch(
            spec, 'c-track', (0, 0, 0), (0, 0, 0), 'h', 5005)
    assert ('remove', 'cid-vehicle_2') in client.calls


def test_docker_kill_force_removes():
    client = FakeDockerClient()
    _docker_backend(client).kill('cid-vehicle_2')
    assert client.calls == [('remove', 'cid-vehicle_2')]


def test_subprocess_alive():
    backend = SubprocessBackend()
    proc = FakeProc()
    proc.poll = lambda: None
    assert backend.alive(proc) is True
    proc.poll = lambda: 0       # exited
    assert backend.alive(proc) is False
    assert backend.alive(None) is False


def test_docker_alive():
    client = FakeDockerClient()
    backend = _docker_backend(client)
    client.running['cid-a'] = True
    assert backend.alive('cid-a') is True
    client.running['cid-a'] = False
    assert backend.alive('cid-a') is False
    assert backend.alive('cid-gone') is False   # inspect raises -> dead
    assert backend.alive(None) is False


def test_docker_find_existing_parses_running_vehicles():
    client = FakeDockerClient()
    client.listing = [
        {'Id': 'cid-7', 'Names': ['/vehicle_7'],
         'Command': 'bash -c "... vehicle.launch.py instance_id:=7 '
                    'vehicle_type:=x500 spawnpoint:=1,0,0.5,0 ..."'},
        {'Id': 'cid-x', 'Names': ['/gazebo'], 'Command': 'irrelevant'},
        {'Id': 'cid-9', 'Names': ['/vehicle_9'],
         'Command': 'ros2 launch realgazebo vehicle.launch.py '
                    'instance_id:=9 vehicle_type:=rover_ackermann'},
    ]
    found = _docker_backend(client).find_existing()
    assert found == [('x500', 7, 'cid-7'), ('rover_ackermann', 9, 'cid-9')]


def test_docker_launch_explicit_roster_overrides_actives():
    """Boot-time fleets pass the full roster so every container gets it."""
    client = FakeDockerClient()
    spec = VehicleSpec(2, 'x500', '/px4', (0.0, 0.0, 0.0, 0.0))
    _docker_backend(client).launch(
        spec, 'c-track', (0, 0, 0), (0, 0, 0), 'h', 5005,
        roster=['boat_9', 'x500_2', 'x500_3'])
    cmd = client.created['vehicle_2']['Cmd'][2]
    assert 'vehicle_models:=boat_9,x500_2,x500_3' in cmd
