import importlib.util
import pathlib
from unittest import mock


MODULE_PATH = pathlib.Path(__file__).parents[1] / "vehicle" / "report_ip.py"
SPEC = importlib.util.spec_from_file_location("report_ip", MODULE_PATH)
REPORT_IP = importlib.util.module_from_spec(SPEC)
SPEC.loader.exec_module(REPORT_IP)


def test_current_ip_uses_default_route_source_address():
    fake_socket = mock.MagicMock()
    fake_socket.__enter__.return_value = fake_socket
    fake_socket.getsockname.return_value = ("128.175.213.241", 49152)
    with mock.patch.object(REPORT_IP.socket, "socket", return_value=fake_socket):
        assert REPORT_IP.current_ip() == "128.175.213.241"
    fake_socket.connect.assert_called_once_with(("1.1.1.1", 443))


def test_send_check_in_uses_authenticated_json_request():
    reply = mock.MagicMock(status=200)
    reply.__enter__.return_value = reply
    with mock.patch.object(REPORT_IP.urllib.request, "urlopen", return_value=reply) as urlopen:
        REPORT_IP.send_check_in(
            "https://example.test/api/check-in",
            "secret-token",
            "ADA5",
            8080,
            "128.175.213.241",
        )
    request = urlopen.call_args.args[0]
    assert request.method == "POST"
    assert request.get_header("Authorization") == "Bearer secret-token"
    assert b'"vehicle_id": "ADA5"' in request.data
    assert b'"ip_address": "128.175.213.241"' in request.data
