import http.client
import json
import secrets
import threading
from types import SimpleNamespace
from http.server import ThreadingHTTPServer

import pytest
from carkit_webui.access import AccessControl, load_access_key
from carkit_webui.server import ApiHandler


@pytest.fixture
def server():
    server = ThreadingHTTPServer(('127.0.0.1', 0), ApiHandler)
    server.access = AccessControl(secrets.token_urlsafe(32))
    server.allowed_origins = set()
    server.manager = SimpleNamespace(terminal_list=lambda: [])
    worker = threading.Thread(target=server.serve_forever, daemon=True)
    worker.start()
    yield server
    server.shutdown(); server.server_close(); worker.join()


def request(server, method, path, body=None, headers=None):
    conn = http.client.HTTPConnection(*server.server_address)
    conn.request(method, path, json.dumps(body) if body is not None else None, headers or {})
    response = conn.getresponse()
    result = response.status, dict(response.getheaders()), json.loads(response.read())
    conn.close()
    return result


def test_private_apis_require_login_and_cross_origin_mutations_are_rejected(server):
    for method, path in [('GET', '/api/terminals'), ('POST', '/api/terminal/create'),
                         ('POST', '/api/launch'), ('POST', '/api/editor/save')]:
        assert request(server, method, path, {})[0] == 401
    assert request(server, 'POST', '/api/login', {'key': 'invalid'})[0] == 401
    status, headers, body = request(server, 'POST', '/api/login', {'key': server.access.key})
    assert status == 200 and body == {'ok': True}
    assert 'HttpOnly' in headers['Set-Cookie'] and 'SameSite=Strict' in headers['Set-Cookie']
    cookie = headers['Set-Cookie'].split(';')[0]
    assert request(server, 'GET', '/api/terminals', headers={'Cookie':cookie})[0] == 200
    assert request(server, 'POST', '/api/stop', {}, {
        'Cookie':cookie, 'Origin':'https://untrusted.example'})[0] == 403
    response = request(server, 'GET', '/api/session', headers={'Cookie':cookie})[2]
    assert response['authenticated'] and response['bridge_token'] != server.access.key
    assert request(server, 'GET', '/api/session')[2]['bridge_token'] is None


def test_access_key_file_is_private_and_reused(tmp_path, monkeypatch):
    monkeypatch.delenv('CARKIT_ACCESS_KEY', raising=False)
    key = load_access_key(tmp_path)
    assert len(key) >= 32 and load_access_key(tmp_path) == key
    assert (tmp_path/'.carkit/access.key').stat().st_mode & 0o777 == 0o600


def test_keyless_access_preserves_origin_checks_and_bridge_session(server):
    server.access = AccessControl(secrets.token_urlsafe(32), require_key=False)
    assert request(server, 'GET', '/api/terminals')[0] == 200
    status, _, session = request(server, 'GET', '/api/session')
    assert status == 200 and session['authenticated']
    assert session['bridge_token'] == server.access.bridge_token
    assert request(server, 'POST', '/api/stop', {}, {
        'Origin': 'https://untrusted.example'})[0] == 403
