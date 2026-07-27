#!/usr/bin/env python3
"""Headless tests for system_metrics_plotter.py."""

from __future__ import annotations

from dataclasses import replace
from pathlib import Path
import tempfile
import unittest

import numpy as np

import system_metrics_plotter as plotter


CSV_HEADER = (
    "timestamp_iso,elapsed_s,sample,cpu_total_percent,"
    "cpu0_freq_hz,cpu1_freq_hz,temp_cpu_thermal_c,"
    "vdd_in_voltage_v,vdd_cpu_gpu_cv_power_w,custom_metric\n"
)
CSV_ROWS = (
    "2026-07-22T21:00:00-04:00,0,1,80,1200000000,1000000000,"
    "40,5.0,4.0,5\n"
    "2026-07-22T21:01:00-04:00,60,2,90,,1400000000,"
    "offline,4.9,4.5,6\n"
    "2026-07-22T21:02:00-04:00,120,3,100,1600000000,1400000000,"
    "45,4.8,5.0,7\n"
)
EVENT_CSV = (
    "timestamp,elapsed_s,event_id,event,source_topic,"
    "auto_session,route_session,details\n"
    "2026-07-22T21:00:30-04:00,30,1,goal_accepted,"
    "/foxglove/waypoints/status,1,1,accepted\n"
    "2026-07-22T21:01:30-04:00,90,2,vehicle_stopped,"
    "/odom,1,1,stopped\n"
)
TOPIC_RATE_CSV = (
    "timestamp,elapsed_s,sample,signal,source_topic,interval_s,"
    "message_count,hz,total_count,last_message_age_s\n"
    "2026-07-22T21:00:00-04:00,0,1,cmd_vel,/cmd_vel,1,20,20,20,0.01\n"
    "2026-07-22T21:00:00-04:00,0,1,odom,/odom,1,50,50,50,0.01\n"
    "2026-07-22T21:01:00-04:00,60,2,cmd_vel,/cmd_vel,1,19,19,39,0.02\n"
    "2026-07-22T21:01:00-04:00,60,2,odom,/odom,1,49,49,99,0.02\n"
    "2026-07-22T21:02:00-04:00,120,3,cmd_vel,/cmd_vel,1,21,21,60,0.01\n"
    "2026-07-22T21:02:00-04:00,120,3,odom,/odom,1,51,51,150,0.01\n"
)


class SystemMetricsPlotterTest(unittest.TestCase):
    def setUp(self) -> None:
        self.temporary_directory = tempfile.TemporaryDirectory()
        self.directory = Path(self.temporary_directory.name)
        self.csv_path = self.directory / "run_a.csv"
        self.csv_path.write_text(CSV_HEADER + CSV_ROWS, encoding="utf-8")
        self.event_csv_path = self.directory / "pipeline_events.csv"
        self.event_csv_path.write_text(EVENT_CSV, encoding="utf-8")
        self.topic_csv_path = self.directory / "pipeline_rates.csv"
        self.topic_csv_path.write_text(TOPIC_RATE_CSV, encoding="utf-8")

    def tearDown(self) -> None:
        self.temporary_directory.cleanup()

    def test_load_converts_frequency_and_keeps_unknown_numeric_column(
        self,
    ) -> None:
        run = plotter.load_csv(self.csv_path)

        np.testing.assert_allclose(
            run.metrics["cpu0_freq_hz"][[0, 2]],
            [1200.0, 1600.0],
        )
        np.testing.assert_allclose(
            run.metrics["cpu_mean_freq_mhz"],
            [1100.0, 1400.0, 1500.0],
        )
        self.assertTrue(np.isnan(run.metrics["temp_cpu_thermal_c"][1]))
        self.assertEqual(run.specs["custom_metric"].group, "Other")
        np.testing.assert_allclose(run.metrics["custom_metric"], [5, 6, 7])

    def test_missing_elapsed_and_non_monotonic_elapsed_are_rejected(
        self,
    ) -> None:
        missing = self.directory / "missing.csv"
        missing.write_text("timestamp_iso,value\n2026-01-01T00:00:00Z,1\n")
        with self.assertRaisesRegex(plotter.PlotterError, "elapsed_s"):
            plotter.load_csv(missing)

        backwards = self.directory / "backwards.csv"
        backwards.write_text(
            "elapsed_s,value\n0,1\n2,2\n1,3\n",
            encoding="utf-8",
        )
        with self.assertRaisesRegex(plotter.PlotterError, "moved backwards"):
            plotter.load_csv(backwards)

    def test_pipeline_events_load_as_markers_without_metrics(self) -> None:
        run = plotter.load_csv(self.event_csv_path)

        self.assertEqual(run.kind, "pipeline_event")
        self.assertEqual(run.metrics, {})
        self.assertEqual(
            [event.event_type for event in run.events],
            ["goal_accepted", "vehicle_stopped"],
        )
        self.assertEqual(
            plotter.available_event_types([run]),
            ("goal_accepted", "vehicle_stopped"),
        )

    def test_selected_pipeline_events_overlay_existing_subplots(self) -> None:
        metric_run = plotter.load_csv(self.csv_path)
        event_run = plotter.load_csv(self.event_csv_path)
        result = plotter.create_plot_figure(
            [metric_run, event_run],
            ["vdd_in_voltage_v", "vdd_cpu_gpu_cv_power_w"],
            plotter.PlotConfig(
                time_unit="seconds",
                event_types=("goal_accepted",),
            ),
        )

        self.assertEqual(len(result.data_axes), 2)
        for axis in result.data_axes:
            event_lines = [
                line
                for line in axis.lines
                if line.get_gid()
                == "pipeline-event-line:goal_accepted"
            ]
            self.assertEqual(len(event_lines), 1)
            self.assertEqual(event_lines[0].get_linestyle(), "--")
            np.testing.assert_allclose(event_lines[0].get_xdata(), [30, 30])
        event_labels = [
            text
            for text in result.data_axes[0].texts
            if text.get_gid()
            == "pipeline-event-label:goal_accepted"
        ]
        self.assertEqual(
            [text.get_text() for text in event_labels],
            ["Goal OK"],
        )
        self.assertGreaterEqual(event_labels[0].get_fontsize(), 8)
        self.assertIsNotNone(event_labels[0].get_bbox_patch())
        self.assertFalse(
            any(
                str(text.get_gid()).startswith("pipeline-event-label:")
                for text in result.data_axes[1].texts
            )
        )

        all_events = plotter.create_plot_figure(
            [metric_run, event_run],
            ["vdd_in_voltage_v"],
            plotter.PlotConfig(time_unit="seconds"),
        )
        self.assertEqual(
            len(
                [
                    line
                    for line in all_events.data_axes[0].lines
                    if str(line.get_gid()).startswith(
                        "pipeline-event-line:"
                    )
                ]
            ),
            2,
        )
        no_events = plotter.create_plot_figure(
            [metric_run, event_run],
            ["vdd_in_voltage_v"],
            plotter.PlotConfig(
                time_unit="seconds",
                event_types=(),
            ),
        )
        self.assertFalse(
            any(
                str(line.get_gid()).startswith("pipeline-event-line:")
                for line in no_events.data_axes[0].lines
            )
        )

    def test_pipeline_event_labels_are_short_and_state_specific(self) -> None:
        state_event = plotter.PipelineEvent(
            event_id="1",
            event_type="behavior_state_changed",
            source_topic="/behavior/state",
            auto_session="1",
            route_session="1",
            details="previous=NORMAL_NAV2 current=TRAFFIC_LIGHT",
        )
        self.assertEqual(
            plotter.pipeline_event_label(state_event, "en"),
            "LIGHT",
        )
        self.assertEqual(
            plotter.pipeline_event_label(state_event, "zh"),
            "状态:红绿灯",
        )

    def test_time_filter_and_statistics_ignore_missing_values(self) -> None:
        run = plotter.load_csv(self.csv_path)
        config = plotter.PlotConfig(
            time_mode="elapsed",
            time_unit="minutes",
            start=1.0,
            end=2.0,
        )
        records = plotter.compute_statistics(
            [run],
            ["vdd_in_voltage_v", "temp_cpu_thermal_c"],
            config,
        )
        by_key = {record.metric_key: record for record in records}

        self.assertEqual(by_key["vdd_in_voltage_v"].count, 2)
        self.assertAlmostEqual(by_key["vdd_in_voltage_v"].mean, 4.85)
        self.assertEqual(by_key["temp_cpu_thermal_c"].count, 1)
        self.assertAlmostEqual(by_key["temp_cpu_thermal_c"].mean, 45.0)

    def test_timestamp_statistics_keep_input_timezone(self) -> None:
        run = plotter.load_csv(self.csv_path)
        base = plotter.PlotConfig(time_mode="timestamp")
        config = replace(
            base,
            start=plotter.parse_boundary(
                "2026-07-22T21:00:00-04:00",
                base,
            ),
            end=plotter.parse_boundary(
                "2026-07-22T21:02:00-04:00",
                base,
            ),
        )
        record = plotter.compute_statistics(
            [run],
            ["vdd_in_voltage_v"],
            config,
        )[0]

        self.assertTrue(record.selection_start.endswith("-04:00"))
        self.assertTrue(record.selection_end.endswith("-04:00"))

    def test_multiple_runs_align_by_elapsed_time_and_use_distinct_styles(
        self,
    ) -> None:
        run_a = plotter.load_csv(self.csv_path, name="1200 MHz")
        run_b = plotter.load_csv(self.csv_path, name="1728 MHz")
        config = plotter.PlotConfig(
            time_mode="elapsed",
            time_unit="minutes",
        )
        result = plotter.create_plot_figure(
            [run_a, run_b],
            ["vdd_in_voltage_v"],
            config,
        )
        lines = result.data_axes[0].lines

        self.assertEqual(len(lines), 2)
        np.testing.assert_allclose(lines[0].get_xdata(), [0, 1, 2])
        np.testing.assert_allclose(lines[1].get_xdata(), [0, 1, 2])
        self.assertNotEqual(lines[0].get_linestyle(), lines[1].get_linestyle())

    def test_selected_topic_hz_metrics_can_share_one_axis(self) -> None:
        run = plotter.load_csv(self.topic_csv_path)
        topic_hz_keys = tuple(
            key
            for key, spec in run.specs.items()
            if spec.group == "Topic Hz"
        )
        self.assertEqual(
            set(topic_hz_keys),
            {"topic_cmd_vel_hz", "topic_odom_hz"},
        )

        separate = plotter.create_plot_figure(
            [run],
            topic_hz_keys,
            plotter.PlotConfig(time_unit="seconds"),
        )
        self.assertEqual(len(separate.data_axes), 2)

        combined = plotter.create_plot_figure(
            [run],
            topic_hz_keys,
            plotter.PlotConfig(
                time_unit="seconds",
                combine_topic_hz=True,
            ),
        )
        self.assertEqual(len(combined.data_axes), 1)
        lines = combined.data_axes[0].lines
        self.assertEqual(len(lines), 2)
        self.assertEqual(
            {line.get_gid() for line in lines},
            {
                "metric-series:topic_cmd_vel_hz",
                "metric-series:topic_odom_hz",
            },
        )
        self.assertEqual(
            {line.get_label() for line in lines},
            {"cmd_vel (/cmd_vel)", "odom (/odom)"},
        )
        self.assertEqual(
            combined.data_axes[0].get_ylabel(),
            "Topic Rate (Hz)",
        )
        self.assertEqual(len(combined.figure.legends), 1)

    def test_topic_hz_overlay_coexists_with_normal_metric_panels(self) -> None:
        system_run = plotter.load_csv(self.csv_path)
        topic_run = plotter.load_csv(self.topic_csv_path)
        result = plotter.create_plot_figure(
            [system_run, topic_run],
            [
                "cpu_total_percent",
                "topic_cmd_vel_hz",
                "topic_odom_hz",
            ],
            plotter.PlotConfig(
                time_unit="seconds",
                combine_topic_hz=True,
            ),
        )

        self.assertEqual(len(result.data_axes), 2)
        self.assertEqual(
            result.data_axes[0].get_ylabel(),
            "CPU Utilization (%)",
        )
        self.assertEqual(result.data_axes[1].get_ylabel(), "Topic Rate (Hz)")

    def test_topic_hz_overlay_is_saved_in_recipe(self) -> None:
        run = plotter.load_csv(self.topic_csv_path)
        metrics = ("topic_cmd_vel_hz", "topic_odom_hz")
        config = plotter.PlotConfig(combine_topic_hz=True)
        output = self.directory / "topic_overlay.json"

        plotter.write_recipe([run], metrics, config, output)
        loaded_runs, loaded_metrics, loaded_config = plotter.load_recipe(
            output
        )

        self.assertEqual(loaded_runs[0].kind, "topic_rate")
        self.assertEqual(loaded_metrics, metrics)
        self.assertTrue(loaded_config.combine_topic_hz)

    def test_missing_metric_warning_ignores_unrelated_csv_kind(self) -> None:
        system_run = plotter.load_csv(self.csv_path)
        topic_run = plotter.load_csv(self.topic_csv_path)

        result = plotter.create_plot_figure(
            [system_run, topic_run],
            ["cpu_total_percent"],
            plotter.PlotConfig(time_unit="seconds"),
        )

        self.assertEqual(result.warnings, ())

    def test_missing_metric_warning_remains_for_same_csv_kind(self) -> None:
        incomplete_path = self.directory / "incomplete_system.csv"
        incomplete_path.write_text(
            "timestamp_iso,elapsed_s,sample,cpu_total_percent\n"
            "2026-07-22T21:00:00-04:00,0,1,80\n",
            encoding="utf-8",
        )
        complete_run = plotter.load_csv(self.csv_path)
        incomplete_run = plotter.load_csv(incomplete_path)

        result = plotter.create_plot_figure(
            [complete_run, incomplete_run],
            ["vdd_in_voltage_v"],
            plotter.PlotConfig(time_unit="seconds"),
        )

        self.assertEqual(
            result.warnings,
            (
                "incomplete_system [System]: metric "
                "vdd_in_voltage_v is not present",
            ),
        )

    def test_every_subplot_keeps_both_axis_labels(self) -> None:
        run = plotter.load_csv(self.csv_path)
        result = plotter.create_plot_figure(
            [run],
            ["vdd_in_voltage_v", "vdd_cpu_gpu_cv_power_w"],
            plotter.PlotConfig(figure_width="double"),
        )
        plotter.FigureCanvasAgg(result.figure).draw()
        for axis in result.data_axes:
            self.assertTrue(axis.get_xlabel())
            self.assertTrue(axis.get_ylabel())
            self.assertTrue(
                any(label.get_visible() for label in axis.get_xticklabels())
            )
            self.assertTrue(
                any(label.get_visible() for label in axis.get_yticklabels())
            )

    def test_figure_output_path_uses_selected_file_type(self) -> None:
        self.assertEqual(
            plotter.resolve_figure_output_path(
                self.directory / "figure",
                "",
            ).suffix,
            ".svg",
        )
        self.assertEqual(
            plotter.resolve_figure_output_path(
                self.directory / "figure",
                "SVG vector",
            ).suffix,
            ".svg",
        )
        self.assertEqual(
            plotter.resolve_figure_output_path(
                self.directory / "figure",
                "*.png",
            ).suffix,
            ".png",
        )
        self.assertEqual(
            plotter.resolve_figure_output_path(
                self.directory / "figure.svg",
                "PDF vector",
            ).suffix,
            ".svg",
        )

    def test_save_rendered_figure_keeps_current_figure_artists(self) -> None:
        run = plotter.load_csv(self.csv_path)
        result = plotter.create_plot_figure(
            [run],
            ["cpu_total_percent"],
            plotter.PlotConfig(time_unit="seconds"),
        )
        current_view_artist = result.data_axes[0].axhline(85)
        current_view_artist.set_gid("current-gui-view")
        result.data_axes[0].set_xlim(30, 90)
        output = self.directory / "current_view.svg"

        plotter.save_rendered_figure(
            result.figure,
            plotter.PlotConfig(dpi=300),
            output,
        )

        exported = output.read_text(encoding="utf-8")
        self.assertIn('id="current-gui-view"', exported)
        np.testing.assert_allclose(
            result.data_axes[0].get_xlim(),
            [30, 90],
        )

    def test_headless_exports_and_recipe_round_trip(self) -> None:
        run = plotter.load_csv(self.csv_path, name="1200 MHz")
        metrics = (
            "vdd_in_voltage_v",
            "vdd_cpu_gpu_cv_power_w",
            "cpu_mean_freq_mhz",
        )
        event_run = plotter.load_csv(self.event_csv_path)
        config = plotter.PlotConfig(
            start=0.0,
            end=2.0,
            dpi=300,
            figure_width="single",
            title="Power Test",
            event_types=("goal_accepted",),
        )
        for suffix in (".pdf", ".svg", ".png"):
            output = self.directory / f"figure{suffix}"
            plotter.save_figure([run, event_run], metrics, config, output)
            self.assertGreater(output.stat().st_size, 100)

        records = plotter.compute_statistics([run, event_run], metrics, config)
        csv_output = self.directory / "statistics.csv"
        latex_output = self.directory / "statistics.tex"
        plotter.write_statistics_csv(records, csv_output)
        plotter.write_statistics_latex(records, latex_output)
        self.assertIn("metric_key", csv_output.read_text(encoding="utf-8"))
        self.assertIn(
            r"\toprule",
            latex_output.read_text(encoding="utf-8"),
        )

        recipe_output = self.directory / "reproduction.json"
        plotter.write_recipe([run, event_run], metrics, config, recipe_output)
        loaded_runs, loaded_metrics, loaded_config = plotter.load_recipe(
            recipe_output
        )
        self.assertEqual(loaded_runs[0].name, "1200 MHz")
        self.assertEqual(loaded_runs[1].kind, "pipeline_event")
        self.assertEqual(loaded_metrics, metrics)
        self.assertEqual(loaded_config, config)

        exit_code = plotter.main(
            ["--no-gui", "--recipe", str(recipe_output)]
        )
        self.assertEqual(exit_code, 0)
        self.assertGreater(
            recipe_output.with_suffix(".pdf").stat().st_size,
            100,
        )


if __name__ == "__main__":
    unittest.main()
