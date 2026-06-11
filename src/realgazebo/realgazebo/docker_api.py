import json
import socket
import http.client

# The realgazebo image ships neither the docker CLI nor the python docker
# SDK, so the manager speaks the raw Docker Engine API over the mounted
# unix socket (stdlib only). API version pinned to the engine's
# MinAPIVersion for forward compatibility.
DOCKER_SOCKET = '/var/run/docker.sock'
API_PREFIX = '/v1.44'


class DockerAPIError(RuntimeError):
    def __init__(self, status, message):
        super().__init__(f"docker API {status}: {message}")
        self.status = status


class _UnixHTTPConnection(http.client.HTTPConnection):
    def __init__(self, socket_path, timeout=60):
        super().__init__('localhost', timeout=timeout)
        self._socket_path = socket_path

    def connect(self):
        sock = socket.socket(socket.AF_UNIX, socket.SOCK_STREAM)
        sock.settimeout(self.timeout)
        sock.connect(self._socket_path)
        self.sock = sock


class DockerClient:
    """Minimal Docker Engine API client (create/connect/start/stop/rm/inspect)."""

    def __init__(self, socket_path=DOCKER_SOCKET):
        self._socket_path = socket_path

    def _request(self, method, path, body=None):
        conn = _UnixHTTPConnection(self._socket_path)
        try:
            payload = json.dumps(body) if body is not None else None
            headers = {'Content-Type': 'application/json'} if payload else {}
            conn.request(method, API_PREFIX + path, body=payload, headers=headers)
            resp = conn.getresponse()
            data = resp.read()
            if resp.status >= 400:
                try:
                    message = json.loads(data).get('message', '')
                except Exception:
                    message = data.decode(errors='replace')
                raise DockerAPIError(resp.status, message)
            return json.loads(data) if data else None
        finally:
            conn.close()

    def create_container(self, name, config):
        """POST /containers/create; returns the new container id."""
        return self._request('POST', f'/containers/create?name={name}', config)['Id']

    def connect_network(self, network, container_id, ipv4=None):
        endpoint = {'Container': container_id}
        if ipv4:
            endpoint['EndpointConfig'] = {'IPAMConfig': {'IPv4Address': ipv4}}
        self._request('POST', f'/networks/{network}/connect', endpoint)

    def start_container(self, container_id):
        self._request('POST', f'/containers/{container_id}/start')

    def stop_container(self, container_id, timeout=10):
        self._request('POST', f'/containers/{container_id}/stop?t={timeout}')

    def remove_container(self, container_id, force=True):
        force_str = 'true' if force else 'false'
        self._request('DELETE', f'/containers/{container_id}?force={force_str}')

    def inspect_container(self, container_id):
        return self._request('GET', f'/containers/{container_id}/json')
