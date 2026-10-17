"""Local key authentication and short-lived browser sessions for the WebUI."""
import os
from pathlib import Path
import secrets
import threading
import time
from http.cookies import SimpleCookie


class AccessControl:
    def __init__(self, key, bridge_token=None, require_key=True):
        if len(key) < 32:
            raise ValueError('The access key must contain at least 32 characters')
        self.require_key = require_key
        self.key = key
        self.bridge_token = bridge_token or secrets.token_urlsafe(32)
        self.sessions = {}
        self.lock = threading.Lock()

    def login(self, key):
        if not isinstance(key, str) or not secrets.compare_digest(key, self.key):
            return None
        session = secrets.token_urlsafe(32)
        with self.lock:
            now = time.monotonic()
            self.sessions = {k: v for k, v in self.sessions.items() if v > now}
            if len(self.sessions) >= 128:
                return None
            self.sessions[session] = now + 12 * 3600
        return session

    def authenticated(self, cookie):
        if not self.require_key:
            return True
        try:
            cookies = SimpleCookie(cookie or '')
            item = cookies.get('carkit_session')
            session = item.value if item else ''
        except Exception:
            return False
        with self.lock:
            return self.sessions.get(session, 0) > time.monotonic()


def load_access_key(workspace):
    configured = os.environ.get('CARKIT_ACCESS_KEY')
    if configured:
        return configured
    path = Path(workspace) / '.carkit' / 'access.key'
    path.parent.mkdir(parents=True, exist_ok=True)
    try:
        fd = os.open(path, os.O_WRONLY | os.O_CREAT | os.O_EXCL, 0o600)
    except FileExistsError:
        if path.stat().st_mode & 0o077:
            raise RuntimeError(f'Access key must be private: chmod 600 {path}')
        return path.read_text().strip()
    key = secrets.token_urlsafe(32)
    with os.fdopen(fd, 'w') as target:
        target.write(key + '\n')
    return key
