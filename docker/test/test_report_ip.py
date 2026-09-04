import importlib.util
import pathlib
import urllib.error
from unittest import mock


MODULE_PATH = pathlib.Path(__file__).parents[1] / "webmonitor" / "report_ip.py"
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


def test_offline_reporter_stops_after_twenty_five_minute_attempts():
    environment = {
        "CARKIT_MONITOR_ENDPOINT": "https://example.test/api/check-in",
        "CARKIT_REPORTER_TOKEN": "x" * 64,
        "CARKIT_VEHICLE_ID": "ADA5",
    }
    with (
        mock.patch.dict(REPORT_IP.os.environ, environment, clear=True),
        mock.patch.object(REPORT_IP, "current_ip", return_value="192.168.1.25"),
        mock.patch.object(
            REPORT_IP,
            "send_check_in",
            side_effect=urllib.error.URLError("offline"),
        ) as send_check_in,
        mock.patch.object(REPORT_IP.time, "sleep") as sleep,
    ):
        assert REPORT_IP.main() == 0

    assert send_check_in.call_count == 20
    assert sleep.call_count == 19
    assert all(call.args == (300,) for call in sleep.call_args_list)
