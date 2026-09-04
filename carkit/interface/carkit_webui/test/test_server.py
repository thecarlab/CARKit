from html.parser import HTMLParser
import base64
from pathlib import Path
import re
import tempfile
import time
import unittest
from unittest.mock import Mock, patch

from carkit_webui.server import (
    BUILD_TARGETS,
    CHASSIS,
    EDITOR_COMPONENT_FILES,
    EDITOR_FILES,
    EDITOR_ROOTS,
    IMPLEMENTATIONS,
    IMPLEMENTATION_BUILD_TARGETS,
    PERCEPTION_MODELS,
    PROFILE_IMPLEMENTATIONS,
    ProcessManager,
    RevisionConflict,
    SAFE_PATH,
    estimate_lipo_percentage,
    resolve_build_packages,
)


class _IdCollector(HTMLParser):
    def __init__(self):
        super().__init__()
        self.ids = []

    def handle_starttag(self, _tag, attrs):
        element_id = dict(attrs).get("id")
        if element_id:
            self.ids.append(element_id)


class TestServer(unittest.TestCase):
    def test_lipo_estimate_handles_common_three_and_four_cell_packs(self):
        self.assertEqual(estimate_lipo_percentage(10.8), 0.0)
        self.assertAlmostEqual(estimate_lipo_percentage(12.6), 1.0)
        self.assertEqual(estimate_lipo_percentage(14.4), 0.0)
        self.assertAlmostEqual(estimate_lipo_percentage(16.8), 1.0)
        self.assertIsNone(estimate_lipo_percentage(float("nan")))

    @staticmethod
    def _editor_manager(directory):
        manager = ProcessManager(Path(directory))
        for files in EDITOR_FILES.values():
            for relative_path in files.values():
                path = Path(directory) / relative_path
                path.parent.mkdir(parents=True, exist_ok=True)
                if not path.exists():
                    path.write_text("value = 1\n", encoding="utf-8")
        return manager

    def test_supported_chassis_are_explicit(self):
        self.assertEqual(CHASSIS, {"osracer", "f1tenth"})

    def test_compile_targets_are_explicit_package_groups(self):
        self.assertEqual(
            set(BUILD_TARGETS),
            {"perception", "localization", "control", "planning"},
        )
        self.assertIn("carkit_perception_msgs", BUILD_TARGETS["perception"])
        self.assertIn("carkit_amcl", BUILD_TARGETS["localization"])
        self.assertIn("carkit_control_center", BUILD_TARGETS["control"])
        self.assertNotIn("carkit_behavior", BUILD_TARGETS["control"])
        self.assertIn("carkit_behavior", BUILD_TARGETS["planning"])
        self.assertIn(
            "carkit_navigation",
            IMPLEMENTATION_BUILD_TARGETS["planning"]["reference"],
        )

    def test_compile_resolves_only_the_selected_course_implementation(self):
        cpp_perception = resolve_build_packages(
            "perception", {"perception": "intro2av_cpp"}
        )
        self.assertEqual(
            cpp_perception,
            ["carkit_perception_msgs", "carkit_intro2av_cpp"],
        )
        self.assertNotIn("carkit_perception", cpp_perception)
        self.assertNotIn("carkit_intro2av", cpp_perception)
        self.assertNotIn("carkit_ada_academy", cpp_perception)

        python_planning = resolve_build_packages(
            "planning", {"planning": "intro2av_python"}
        )
        self.assertEqual(
            python_planning,
            ["carkit_behavior", "carkit_intro2av"],
        )

    def test_compile_includes_required_reference_and_ada_support(self):
        self.assertEqual(
            resolve_build_packages(
                "perception", {"perception": "ada_academy"}
            ),
            [
                "carkit_perception_msgs",
                "carkit_perception",
                "carkit_ada_academy",
            ],
        )
        reference_control = resolve_build_packages(
            "control", {"control": "reference"}
        )
        self.assertIn("carkit_control_center", reference_control)
        self.assertIn("carkit_amcl", reference_control)
        self.assertFalse(any("intro2av" in name for name in reference_control))

    def test_localization_compile_is_implementation_independent(self):
        self.assertEqual(
            resolve_build_packages(
                "localization", {"localization": "invalid"}
            ),
            ["carkit_amcl", "carkit_slam"],
        )

    def test_invalid_compile_implementation_is_rejected(self):
        with self.assertRaisesRegex(ValueError, "invalid control implementation"):
            resolve_build_packages("control", {"control": "shell-command"})

    def test_compile_tab_sends_setup_implementation_selection(self):
        static_dir = Path(__file__).parents[1] / "static"
        javascript = (static_dir / "app.js").read_text(encoding="utf-8")
        self.assertIn("implementations: setup.implementations", javascript)
        self.assertIn("result.packages.join", javascript)

    def test_camera_stream_uses_native_binary_cbor_with_json_fallback(self):
        static_dir = Path(__file__).parents[1] / "static"
        html = (static_dir / "index.html").read_text(encoding="utf-8")
        javascript = (static_dir / "app.js").read_text(encoding="utf-8")
        self.assertIn('id="perception-overlay"', html)
        self.assertIn('socket.binaryType = "arraybuffer"', javascript)
        self.assertIn("native CARKit C++ bridge", javascript)
        self.assertIn('{compression: "cbor"}', javascript)
        self.assertIn("decodeCbor(event.data)", javascript)
        self.assertIn('typeof bytes === "string"', javascript)
        self.assertIn("drawPerceptionOverlay()", javascript)
        self.assertNotIn(
            '["/yolo/inference_image/compressed",', javascript
        )

    def test_invalid_compile_target_is_rejected_before_spawning(self):
        with tempfile.TemporaryDirectory() as directory:
            manager = ProcessManager(Path(directory))
            with self.assertRaisesRegex(ValueError, "invalid compile target"):
                manager.compile("shell; command")

    def test_compile_command_uses_selected_packages_and_builds_dependencies(self):
        """Verify that compile command uses selected packages and builds
        dependencies."""
        with tempfile.TemporaryDirectory() as directory:
            manager = ProcessManager(Path(directory))
            process = Mock()
            process.poll.return_value = None
            manager._spawn = Mock(return_value=process)
            result = manager.compile(
                "perception", {"perception": "intro2av_cpp"}
            )
            command = manager._spawn.call_args.args[0][2]

        self.assertEqual(
            result["packages"],
            ["carkit_perception_msgs", "carkit_intro2av_cpp"],
        )
        self.assertIn("--packages-up-to", command)
        self.assertNotIn("--packages-select", command)
        self.assertIn("-DCMAKE_BUILD_TYPE=Release", command)
        self.assertIn(
            "carkit_perception_msgs carkit_intro2av_cpp", command
        )
        self.assertNotIn("carkit_ada_academy", command)

    def test_supported_perception_models_are_explicit(self):
        self.assertEqual(
            PERCEPTION_MODELS,
            {"generic_coco", "traffic_signs", "combined", "custom"},
        )

    def test_course_profiles_select_all_three_explicit_implementations(self):
        self.assertNotIn("profile", IMPLEMENTATIONS)
        self.assertEqual(
            set(PROFILE_IMPLEMENTATIONS["ada_high_school"].values()),
            {"ada_academy"},
        )
        self.assertEqual(
            set(PROFILE_IMPLEMENTATIONS["intro2av"].values()),
            {"intro2av_python"},
        )

    def test_map_paths_cannot_inject_shell_commands(self):
        self.assertIsNotNone(
            SAFE_PATH.fullmatch("/workspaces/CARKit/map/map_3f.yaml")
        )
        self.assertIsNone(
            SAFE_PATH.fullmatch("map.yaml; touch /tmp/untrusted")
        )

    def test_map_dropdown_lists_only_yaml_files_from_map_folder(self):
        with tempfile.TemporaryDirectory() as directory:
            map_directory = Path(directory) / "map"
            map_directory.mkdir()
            (map_directory / "campus.yaml").write_text("image: campus.pgm\n")
            (map_directory / "track.yaml").write_text("image: track.pgm\n")
            (map_directory / "track.pgm").write_bytes(b"P5\n")
            manager = ProcessManager(Path(directory))

            files = manager.map_files()

        self.assertEqual(
            [item["name"] for item in files],
            ["campus.yaml", "track.yaml"],
        )
        self.assertTrue(
            all(Path(item["path"]).suffix == ".yaml" for item in files)
        )

    def test_fresh_workspace_is_not_installed(self):
        with tempfile.TemporaryDirectory() as directory:
            manager = ProcessManager(Path(directory))
            status = manager.status()
        self.assertFalse(status["installed"])
        self.assertFalse(status["running"])
        self.assertIsNone(status["launch_config"])
        self.assertIsNone(status["chassis_telemetry"])
        self.assertEqual(
            set(status["system"]),
            {
                "cpu_count", "cpu_capacity_percent",
                "cpu_percent", "workload_cpu_percent", "memory",
                "workload_memory",
                "cpu_temperature_c",
                "load_average", "uptime_seconds",
            },
        )

    def test_cpu_metrics_use_aggregate_per_core_capacity(self):
        with tempfile.TemporaryDirectory() as directory:
            manager = ProcessManager(Path(directory))
            manager.cpu_sample = (1000, 400)
            manager.workload_cpu_sample = (10.0, 100.0)
            manager._cpu_totals = Mock(return_value=(1600, 500))
            manager._workload_cpu_usage_seconds = Mock(return_value=13.0)
            manager._workload_memory_metrics = Mock(return_value={
                "used_bytes": 1024,
                "total_bytes": 4096,
                "percent": 25.0,
                "limited": False,
            })
            with (
                patch("carkit_webui.server.os.cpu_count", return_value=6),
                patch("carkit_webui.server.time.monotonic", return_value=101.0),
            ):
                metrics = manager.system_metrics()

        self.assertEqual(metrics["cpu_count"], 6)
        self.assertEqual(metrics["cpu_capacity_percent"], 600.0)
        self.assertEqual(metrics["cpu_percent"], 500.0)
        self.assertEqual(metrics["workload_cpu_percent"], 300.0)
        self.assertEqual(metrics["workload_memory"]["percent"], 25.0)

    def test_workload_memory_uses_host_total_for_unlimited_cgroup(self):
        with patch("carkit_webui.server.Path.read_text") as read_text:
            read_text.side_effect = ["1073741824\n", "max\n"]
            metrics = ProcessManager._workload_memory_metrics(8 * 1024 ** 3)

        self.assertEqual(metrics["used_bytes"], 1024 ** 3)
        self.assertEqual(metrics["total_bytes"], 8 * 1024 ** 3)
        self.assertEqual(metrics["percent"], 12.5)
        self.assertFalse(metrics["limited"])

    def test_stop_clears_stale_launch_configuration(self):
        with tempfile.TemporaryDirectory() as directory:
            manager = ProcessManager(Path(directory))
            manager.current_config = {"profile": "reference"}
            manager.stop()
            self.assertIsNone(manager.status()["launch_config"])

    def test_status_can_return_only_new_log_lines(self):
        with tempfile.TemporaryDirectory() as directory:
            manager = ProcessManager(Path(directory))
            manager._append_log_locked("first")
            cursor = manager.status()["log_cursor"]
            manager._append_log_locked("second")
            status = manager.status(cursor)

        self.assertEqual(status["logs"], ["second"])
        self.assertEqual(status["log_start"], cursor)
        self.assertEqual(status["log_cursor"], cursor + 1)

    def test_dashboard_javascript_references_existing_elements(self):
        static_dir = Path(__file__).parents[1] / "static"
        parser = _IdCollector()
        parser.feed((static_dir / "index.html").read_text(encoding="utf-8"))
        javascript = (static_dir / "app.js").read_text(encoding="utf-8")
        referenced_ids = set(re.findall(r'\$\("([^"]+)"\)', javascript))
        dynamic_ids = {
            "start-camera",
            "start-lidar",
            *(f"component-{name}" for name in (
                "chassis", "sensors", "planning", "control",
                "perception", "behavior",
            )),
        }
        self.assertTrue(referenced_ids)
        self.assertEqual(len(parser.ids), len(set(parser.ids)))
        self.assertEqual(referenced_ids - set(parser.ids) - dynamic_ids, set())

    def test_system_log_is_only_available_from_overview(self):
        static_dir = Path(__file__).parents[1] / "static"
        html = (static_dir / "index.html").read_text(encoding="utf-8")
        javascript = (static_dir / "app.js").read_text(encoding="utf-8")

        self.assertIn('id="system-console"', html)
        self.assertNotIn('<span>System log</span>', html)
        self.assertNotIn('"#system-console":', javascript)

    def test_header_uses_official_carlab_mark(self):
        static_dir = Path(__file__).parents[1] / "static"
        html = (static_dir / "index.html").read_text(encoding="utf-8")
        stylesheet = (static_dir / "style.css").read_text(encoding="utf-8")
        self.assertEqual(html.count('class="brand-mark"'), 2)
        self.assertNotIn("<span>C</span><i></i>", html)
        self.assertIn("Official CAR Lab mark", stylesheet)
        encoded = re.search(
            r'data:image/webp;base64,([^"]+)', stylesheet
        ).group(1)
        logo = base64.b64decode(encoded)
        self.assertEqual(logo[:4], b"RIFF")
        self.assertEqual(logo[8:12], b"WEBP")

    def test_terminal_tab_exposes_shared_session_controls(self):
        static_dir = Path(__file__).parents[1] / "static"
        html = (static_dir / "index.html").read_text(encoding="utf-8")
        javascript = (static_dir / "app.js").read_text(encoding="utf-8")
        stylesheet = (static_dir / "style.css").read_text(encoding="utf-8")

        self.assertIn('href="#terminal"', html)
        self.assertIn('id="terminal-page"', html)
        self.assertIn('id="terminal-list"', html)
        self.assertIn('id="terminal-interrupt"', html)
        self.assertIn('id="terminal-eof"', html)
        self.assertIn('id="terminal-screen" tabindex="0"', html)
        self.assertNotIn('id="terminal-command"', html)
        self.assertNotIn('id="terminal-command-form"', html)
        self.assertIn('"/api/terminal/create"', javascript)
        self.assertIn('"/api/terminal/input"', javascript)
        self.assertIn("function selectTerminal(terminalId)", javascript)
        self.assertIn("function terminalSelection()", javascript)
        self.assertIn("function updateTerminalCursor()", javascript)
        self.assertIn('addEventListener("keydown", terminalKey)', javascript)
        self.assertIn('Tab: "\\t"', javascript)
        self.assertIn('shortcut === "v"', javascript)
        self.assertIn("this.escape.length === 2", javascript)
        self.assertIn('queueTerminalInput("\\u0003")', javascript)
        self.assertIn("terminal-cursor-blink", stylesheet)

    def test_shared_terminal_runs_in_workspace_and_replays_output(self):
        with tempfile.TemporaryDirectory() as directory:
            manager = ProcessManager(Path(directory))
            terminal = manager.create_terminal("Lab shell", "Ada")
            manager.terminal_input(
                terminal["id"], "printf 'shared-terminal-marker:%s\\n' \"$PWD\"\r"
            )
            output = ""
            expected = f"shared-terminal-marker:{directory}"
            deadline = time.monotonic() + 3.0
            while time.monotonic() < deadline:
                snapshot = manager.terminal_output(terminal["id"], 0)
                output = base64.b64decode(snapshot["output"]).decode(
                    "utf-8", errors="replace"
                )
                if expected in output:
                    break
                time.sleep(0.02)
            listed = manager.terminal_list()
            manager.close()

        self.assertIn(expected, output)
        self.assertEqual(listed[0]["title"], "Lab shell")
        self.assertEqual(listed[0]["owner"], "Ada")

    def test_terminal_ids_and_input_are_validated(self):
        with tempfile.TemporaryDirectory() as directory:
            manager = ProcessManager(Path(directory))
            with self.assertRaisesRegex(ValueError, "invalid terminal id"):
                manager.terminal_output("../../not-a-terminal", 0)
            terminal = manager.create_terminal("\x00\n", "\x1bStudent")
            with self.assertRaisesRegex(ValueError, "8 KiB"):
                manager.terminal_input(terminal["id"], "x" * 9000)
            manager.close()

        self.assertTrue(terminal["title"].startswith("Terminal"))
        self.assertEqual(terminal["owner"], "Student")

    def test_map_can_be_replayed_from_the_native_bridge(self):
        static_dir = Path(__file__).parents[1] / "static"
        html = (static_dir / "index.html").read_text(encoding="utf-8")
        javascript = (static_dir / "app.js").read_text(encoding="utf-8")

        self.assertIn('id="republish-map"', html)
        self.assertIn('id="active-map-file"', html)
        self.assertIn('<select id="map">', html)
        self.assertNotIn('<input id="map"', html)
        self.assertIn("function republishMap()", javascript)
        self.assertIn(
            "function updateMapCoordinate(clientX, clientY)", javascript
        )
        self.assertIn("state.mapCursor = point", javascript)
        odom_handler = re.search(
            r'topic === "/odom"\)(.*?)topic === "/control_center/main_state"',
            javascript,
            re.DOTALL,
        )
        self.assertIsNotNone(odom_handler)
        self.assertNotIn("map-coordinate", odom_handler.group(1))
        self.assertIn('topic: "/map"', javascript)
        self.assertIn('$("republish-map").onclick = republishMap', javascript)

    def test_initial_pose_drag_sets_heading_and_tracks_amcl_pose(self):
        static_dir = Path(__file__).parents[1] / "static"
        javascript = (static_dir / "app.js").read_text(encoding="utf-8")

        self.assertIn("function startPoseDrag(event)", javascript)
        self.assertIn("function stopPoseDrag(event, cancelled = false)", javascript)
        self.assertIn("Math.atan2(drag.end.y - drag.start.y", javascript)
        self.assertIn("z: Math.sin(halfHeading)", javascript)
        self.assertIn("w: Math.cos(halfHeading)", javascript)
        self.assertIn('["/amcl_pose", "geometry_msgs/msg/PoseWithCovarianceStamped"', javascript)
        self.assertIn('topic === "/amcl_pose"', javascript)
        self.assertIn("if (sent) setMapPose(pose, null)", javascript)

    def test_control_authority_switch_uses_confirmed_control_center_mode(self):
        static_dir = Path(__file__).parents[1] / "static"
        html = (static_dir / "index.html").read_text(encoding="utf-8")
        javascript = (static_dir / "app.js").read_text(encoding="utf-8")

        self.assertIn('id="mode-human"', html)
        self.assertIn('id="mode-autonomous"', html)
        self.assertIn(
            '"/enable_autonomous_control", "std_msgs/msg/Int8"',
            javascript,
        )
        self.assertIn('topic === "/control_center/main_state"', javascript)
        self.assertIn('renderControlMode(message.data)', javascript)

    def test_runtime_health_tracks_lidar_separately_from_camera(self):
        static_dir = Path(__file__).parents[1] / "static"
        html = (static_dir / "index.html").read_text(encoding="utf-8")
        javascript = (static_dir / "app.js").read_text(encoding="utf-8")

        self.assertIn('id="health-sensors"', html)
        self.assertIn("Camera sensor", html)
        self.assertIn('id="health-lidar"', html)
        self.assertIn("LiDAR sensor", html)
        self.assertIn('<small>/scan</small>', html)
        self.assertIn('markHealth("lidar",', javascript)
        self.assertIn('markHealth("sensors")', javascript)

    def test_taskbar_supports_both_chassis_battery_topics(self):
        static_dir = Path(__file__).parents[1] / "static"
        html = (static_dir / "index.html").read_text(encoding="utf-8")
        javascript = (static_dir / "app.js").read_text(encoding="utf-8")
        stylesheet = (static_dir / "style.css").read_text(encoding="utf-8")

        for element_id in (
            "battery-voltage", "cpu-usage", "memory-usage",
            "carkit-cpu-usage", "carkit-memory-usage",
            "cpu-temperature", "chassis-status",
        ):
            self.assertIn(f'id="{element_id}"', html)
        self.assertIn('"/battery_state", "sensor_msgs/msg/BatteryState"', javascript)
        self.assertIn('"/sensors/core", "vesc_msgs/msg/VescStateStamped"', javascript)
        self.assertIn("CPU usage", html)
        self.assertIn("RAM usage", html)
        self.assertIn("CARKit", html)
        self.assertIn("busyCores = cpu === null ? null : cpu / 100", javascript)
        self.assertIn("system.workload_cpu_percent", javascript)
        self.assertIn("system.workload_memory", javascript)
        self.assertIn(".resource-chip small { margin-top: 3px", stylesheet)
        self.assertIn(".resource-chip dd b, .resource-chip small b { font-size: 12px", stylesheet)
        self.assertIn("system.cpu_capacity_percent", javascript)
        self.assertIn("status.chassis_telemetry", javascript)

    def test_editor_exposes_reference_as_read_only(self):
        self.assertEqual(
            set(EDITOR_FILES),
            {"reference", "ada_academy", "intro2av_python", "intro2av_cpp"},
        )
        self.assertTrue(all(
            set(files) == {"perception", "planning", "control"}
            for files in EDITOR_FILES.values()
        ))
        manifest = ProcessManager(Path(".")).editor_manifest()
        self.assertEqual(
            {item["id"] for item in manifest["implementations"]},
            set(EDITOR_FILES),
        )
        access = {
            item["id"]: item["read_only"]
            for item in manifest["implementations"]
        }
        self.assertTrue(access["reference"])
        self.assertFalse(access["ada_academy"])
        self.assertFalse(access["intro2av_python"])
        self.assertFalse(access["intro2av_cpp"])

    def test_reference_editor_is_browseable_but_cannot_be_saved(self):
        with tempfile.TemporaryDirectory() as directory:
            manager = self._editor_manager(directory)
            tree = manager.editor_tree("reference", "planning")
            snapshot = manager.collaborative_snapshot(
                "reference", "planning", "student_one", "Ada",
            )
            reference_path = Path(directory) / EDITOR_FILES["reference"]["planning"]
            original = reference_path.read_text(encoding="utf-8")

            with self.assertRaisesRegex(ValueError, "read-only"):
                manager.save_editor_file(
                    "reference", "planning", "changed\n", snapshot["revision"]
                )
            with self.assertRaisesRegex(ValueError, "read-only"):
                manager.synchronize_editor(
                    "reference", "planning", "student_one", "Ada",
                    snapshot["version"], "changed\n",
                )
            unchanged = reference_path.read_text(encoding="utf-8")

        self.assertTrue(tree["read_only"])
        self.assertTrue(snapshot["read_only"])
        self.assertEqual(unchanged, original)

    def test_editor_requests_the_selected_implementation_and_component(self):
        static_dir = Path(__file__).parents[1] / "static"
        html = (static_dir / "index.html").read_text(encoding="utf-8")
        javascript = (static_dir / "app.js").read_text(encoding="utf-8")

        self.assertIn('<option value="ada_academy">', html)
        self.assertIn('<option value="reference">Reference · Read only</option>', html)
        self.assertIn('api("/api/editor")', javascript)
        self.assertIn("editorConfig.implementations || editorConfig.profiles", javascript)
        self.assertIn("implementation=${encodeURIComponent(implementation)}", javascript)
        self.assertIn("component=${encodeURIComponent(component)}", javascript)
        self.assertIn("return tree.default || tree.defaults[component]", javascript)
        self.assertIn("function scopeLegacyEditorTree", javascript)
        self.assertIn("allowed.has(file.path)", javascript)
        self.assertIn("function setEditorReadOnly(readOnly)", javascript)
        self.assertIn('$("code-editor").readOnly = state.editorReadOnly', javascript)
        self.assertIn('state.editorReadOnly ? "Read only" : "Sync now"', javascript)

    def test_editor_save_uses_optimistic_revision_conflicts(self):
        with tempfile.TemporaryDirectory() as directory:
            manager = self._editor_manager(directory)
            opened = manager.read_editor_file("intro2av_python", "planning")
            saved = manager.save_editor_file(
                "intro2av_python",
                "planning",
                "value = 2\n",
                opened["revision"],
            )
            with self.assertRaises(RevisionConflict):
                manager.save_editor_file(
                    "intro2av_python",
                    "planning",
                    "value = 3\n",
                    opened["revision"],
                )

        self.assertNotEqual(saved["revision"], opened["revision"])
        self.assertIsNone(saved["syntax_error"])

    def test_editor_reports_python_syntax_without_losing_saved_work(self):
        with tempfile.TemporaryDirectory() as directory:
            manager = self._editor_manager(directory)
            opened = manager.read_editor_file("ada_academy", "control")
            saved = manager.save_editor_file(
                "ada_academy",
                "control",
                "def incomplete(:\n",
                opened["revision"],
            )
            reopened = manager.read_editor_file(
                "ada_academy",
                "control",
            )

        self.assertEqual(reopened["content"], "def incomplete(:\n")
        self.assertEqual(saved["syntax_error"]["line"], 1)

    def test_editor_accepts_intro2av_cpp_source(self):
        with tempfile.TemporaryDirectory() as directory:
            manager = self._editor_manager(directory)
            opened = manager.read_editor_file("intro2av_cpp", "control")
            saved = manager.save_editor_file(
                "intro2av_cpp",
                "control",
                "// student C++\nint main() { return 0; }\n",
                opened["revision"],
            )

        self.assertEqual(opened["language"], "cpp")
        self.assertEqual(saved["language"], "cpp")
        self.assertIsNone(saved["syntax_error"])

    def test_collaboration_rebases_concurrent_edits_without_losing_either(self):
        with tempfile.TemporaryDirectory() as directory:
            manager = self._editor_manager(directory)
            first = manager.collaborative_snapshot(
                "intro2av_python", "planning", "student_one", "Ada"
            )
            second = manager.collaborative_snapshot(
                "intro2av_python", "planning", "student_two", "Grace"
            )
            manager.synchronize_editor(
                "intro2av_python", "planning", "student_one", "Ada",
                first["version"], "# first\n" + first["content"],
            )
            merged = manager.synchronize_editor(
                "intro2av_python", "planning", "student_two", "Grace",
                second["version"], second["content"] + "# second\n",
            )

        self.assertIn("# first", merged["content"])
        self.assertIn("# second", merged["content"])
        self.assertEqual({user["name"] for user in merged["users"]}, {"Ada", "Grace"})

    def test_collaboration_is_document_scoped_and_reports_diagnostics(self):
        with tempfile.TemporaryDirectory() as directory:
            manager = self._editor_manager(directory)
            planning = manager.collaborative_snapshot(
                "intro2av_python", "planning", "student_one", "Ada"
            )
            invalid = manager.synchronize_editor(
                "intro2av_python", "planning", "student_one", "Ada",
                planning["version"], "def broken(:\n",
            )
            control = manager.collaborative_snapshot(
                "intro2av_python", "control", "student_two", "Grace"
            )
            cpp = manager.collaborative_snapshot(
                "intro2av_cpp", "control", "student_three", "Linus"
            )
            invalid_cpp = manager.synchronize_editor(
                "intro2av_cpp", "control", "student_three", "Linus",
                cpp["version"], "int main() {\n",
            )

        self.assertTrue(invalid["diagnostics"])
        self.assertEqual(control["content"], "value = 1\n")
        self.assertIn("unclosed", invalid_cpp["diagnostics"][0]["message"])

    def test_editor_tree_is_scoped_to_implementation_and_component(self):
        with tempfile.TemporaryDirectory() as directory:
            manager = self._editor_manager(directory)
            root = Path(directory) / EDITOR_ROOTS["intro2av_python"]
            for relative in EDITOR_COMPONENT_FILES["intro2av_python"]["planning"]:
                path = root / relative
                path.parent.mkdir(parents=True, exist_ok=True)
                if not path.exists():
                    path.write_text("value = 1\n", encoding="utf-8")
            (root / "README.md").write_text(
                "# Student notes\n", encoding="utf-8"
            )
            tree = manager.editor_tree("intro2av_python", "planning")
            notes = manager.collaborative_snapshot(
                "intro2av_python", "planning", "student_one", "Ada",
                file_path="README.md",
            )
            changed_notes = manager.synchronize_editor(
                "intro2av_python", "planning", "student_one", "Ada",
                notes["version"], "# Shared student notes\n",
                file_path="README.md",
            )
            planning = manager.collaborative_snapshot(
                "intro2av_python", "planning", "student_two", "Grace",
                file_path=tree["defaults"]["planning"],
            )

        self.assertEqual(tree["root"], str(EDITOR_ROOTS["intro2av_python"]))
        self.assertEqual(tree["implementation"], "intro2av_python")
        self.assertEqual(tree["component"], "planning")
        self.assertIn("README.md", {item["path"] for item in tree["files"]})
        self.assertNotIn(
            "carkit_intro2av/control_algorithm.py",
            {item["path"] for item in tree["files"]},
        )
        self.assertEqual(
            tree["defaults"]["planning"],
            "carkit_intro2av/planning_algorithm.py",
        )
        self.assertEqual(tree["default"], tree["defaults"]["planning"])
        self.assertEqual(notes["content"], "# Student notes\n")
        self.assertEqual(notes["language"], "md")
        self.assertEqual(changed_notes["version"], 1)
        self.assertEqual(planning["version"], 0)
        self.assertEqual(planning["content"], "value = 1\n")

    def test_editor_rejects_a_file_from_another_component(self):
        with tempfile.TemporaryDirectory() as directory:
            manager = self._editor_manager(directory)
            with self.assertRaisesRegex(ValueError, "selected.*component"):
                manager.collaborative_snapshot(
                    "intro2av_python", "planning", "student_one", "Ada",
                    file_path="carkit_intro2av/control_algorithm.py",
                )

    def test_editor_tree_rejects_paths_outside_student_package(self):
        with tempfile.TemporaryDirectory() as directory:
            manager = self._editor_manager(directory)
            with self.assertRaisesRegex(ValueError, "escaped"):
                manager.collaborative_snapshot(
                    "intro2av_python", "planning", "student_one", "Ada",
                    file_path="../../protected.py",
                )
