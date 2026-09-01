// CARKit learning annotation: implements the behavior described by this file's package and module.
import Clutter from 'gi://Clutter';
import Gio from 'gi://Gio';
import GLib from 'gi://GLib';
import GObject from 'gi://GObject';
import Soup from 'gi://Soup?version=3.0';
import St from 'gi://St';

import {Extension} from 'resource:///org/gnome/shell/extensions/extension.js';
import * as Main from 'resource:///org/gnome/shell/ui/main.js';
import * as PanelMenu from 'resource:///org/gnome/shell/ui/panelMenu.js';
import * as PopupMenu from 'resource:///org/gnome/shell/ui/popupMenu.js';

const STATUS_URL = 'http://127.0.0.1:8080/api/status';
const DASHBOARD_URL = 'http://127.0.0.1:8080/';
// Chassis telemetry changes slowly enough that five-second polling keeps the
// top bar useful without waking GNOME Shell 30 times per minute.
const POLL_SECONDS = 5;

function numberOrNull(value) {
    const number = Number(value);
    return Number.isFinite(number) ? number : null;
}

function percentage(value) {
    const number = numberOrNull(value);
    return number === null ? '—' : `${number.toFixed(1)}%`;
}

function temperature(value) {
    const number = numberOrNull(value);
    return number === null ? '—' : `${number.toFixed(1)}°C`;
}

function batteryPercentage(value) {
    const number = numberOrNull(value);
    return number === null || number < 0 ? '—' : `${Math.round(number * 100)}%`;
}

const ResourceIndicator = GObject.registerClass(
class ResourceIndicator extends PanelMenu.Button {
    _init() {
        super._init(0.0, 'CARKit Resources');

        this._destroyed = false;
        this._requestPending = false;
        this._cancellable = new Gio.Cancellable();
        this._session = new Soup.Session({timeout: 3});
        this._label = new St.Label({
            text: 'CARKit connecting…',
            y_align: Clutter.ActorAlign.CENTER,
            style_class: 'carkit-resource-label',
        });
        this.add_child(this._label);

        this._sessionItem = this._detailItem('Session');
        this._cpuItem = this._detailItem('Jetson CPU');
        this._memoryItem = this._detailItem('Jetson RAM');
        this._temperatureItem = this._detailItem('Temperature');
        this._batteryItem = this._detailItem('Chassis battery');
        this._updatedItem = this._detailItem('Updated');
        this.menu.addMenuItem(new PopupMenu.PopupSeparatorMenuItem());

        const dashboardItem = new PopupMenu.PopupMenuItem('Open CARKit dashboard');
        dashboardItem.connect('activate', () => {
            Gio.AppInfo.launch_default_for_uri(DASHBOARD_URL, null);
        });
        this.menu.addMenuItem(dashboardItem);

        const refreshItem = new PopupMenu.PopupMenuItem('Refresh now');
        refreshItem.connect('activate', () => this._poll());
        this.menu.addMenuItem(refreshItem);

        this._poll();
        this._timerId = GLib.timeout_add_seconds(
            GLib.PRIORITY_DEFAULT,
            POLL_SECONDS,
            () => {
                this._poll();
                return GLib.SOURCE_CONTINUE;
            }
        );
    }

    _detailItem(name) {
        const item = new PopupMenu.PopupMenuItem(`${name}: —`, {
            reactive: false,
        });
        item.setSensitive(false);
        this.menu.addMenuItem(item);
        return item;
    }

    _setDetail(item, name, value) {
        item.label.text = `${name}: ${value}`;
    }

    _poll() {
        if (this._destroyed || this._requestPending)
            return;
        this._requestPending = true;
        const message = Soup.Message.new('GET', STATUS_URL);
        this._session.send_and_read_async(
            message,
            GLib.PRIORITY_DEFAULT,
            this._cancellable,
            (session, result) => {
                this._requestPending = false;
                if (this._destroyed)
                    return;
                try {
                    const bytes = session.send_and_read_finish(result);
                    if (message.get_status() !== Soup.Status.OK)
                        throw new Error(`HTTP ${message.get_status()}`);
                    const payload = JSON.parse(
                        new TextDecoder().decode(bytes.get_data())
                    );
                    this._render(payload);
                } catch (error) {
                    if (!this._cancellable.is_cancelled())
                        this._renderOffline(error.message);
                }
            }
        );
    }

    _render(payload) {
        const system = payload.system || {};
        const memory = system.memory || {};
        const cpu = numberOrNull(system.cpu_percent);
        const cpuCount = numberOrNull(system.cpu_count);
        const cpuCapacity = numberOrNull(system.cpu_capacity_percent) || 100;
        const memoryPercent = numberOrNull(memory.percent);
        const heat = numberOrNull(system.cpu_temperature_c);
        const battery = payload.chassis_telemetry || {};
        const batteryFresh = Boolean(battery.fresh);
        const voltage = batteryFresh ? numberOrNull(battery.voltage) : null;
        const batteryPercent = batteryFresh
            ? numberOrNull(battery.percentage)
            : null;
        const batteryLabel = voltage === null
            ? 'BATT —'
            : `BATT ${voltage.toFixed(1)}V ${batteryPercentage(batteryPercent)}`;

        this._label.text = [
            `CPU ${percentage(cpu)}/${cpuCapacity.toFixed(0)}%`,
            `RAM ${percentage(memoryPercent)}`,
            temperature(heat),
            batteryLabel,
        ].join(' · ');
        this._label.remove_style_class_name('carkit-resource-offline');
        this._label.remove_style_class_name('carkit-resource-warning');
        if (
            (heat !== null && heat >= 75)
            || (cpu !== null && cpu >= cpuCapacity * 0.85)
            || (batteryPercent !== null && batteryPercent <= 0.2)
        )
            this._label.add_style_class_name('carkit-resource-warning');

        const config = payload.launch_config || {};
        const session = payload.running
            ? `${config.profile || 'running'} · ${config.chassis || 'chassis'}`
            : payload.job_running
                ? payload.job || 'build running'
                : 'Stopped';
        this._setDetail(this._sessionItem, 'Session', session);
        const busyCores = cpu === null ? null : cpu / 100;
        const coreLabel = cpuCount === null
            ? 'cores —'
            : `${busyCores === null ? '—' : busyCores.toFixed(1)} / ${cpuCount.toFixed(0)} cores busy`;
        this._setDetail(
            this._cpuItem,
            'Jetson CPU',
            `${percentage(cpu)} / ${cpuCapacity.toFixed(0)}% · ${coreLabel}`
        );
        this._setDetail(
            this._memoryItem,
            'Jetson RAM',
            percentage(memoryPercent)
        );
        this._setDetail(this._temperatureItem, 'Temperature', temperature(heat));
        this._setDetail(
            this._batteryItem,
            'Chassis battery',
            voltage === null
                ? 'No recent telemetry'
                : `${voltage.toFixed(2)} V · ${batteryPercentage(batteryPercent)} estimated`
        );
        this._setDetail(
            this._updatedItem,
            'Updated',
            GLib.DateTime.new_now_local().format('%H:%M:%S')
        );
    }

    _renderOffline(reason) {
        this._label.text = 'CARKit offline';
        this._label.remove_style_class_name('carkit-resource-warning');
        this._label.add_style_class_name('carkit-resource-offline');
        this._setDetail(this._sessionItem, 'Session', 'WebUI unavailable');
        this._setDetail(this._updatedItem, 'Error', reason || 'Connection failed');
    }

    destroy() {
        this._destroyed = true;
        if (this._timerId) {
            GLib.source_remove(this._timerId);
            this._timerId = 0;
        }
        this._cancellable.cancel();
        this._session.abort();
        super.destroy();
    }
});

export default class CARKitResourcesExtension extends Extension {
    enable() {
        this._indicator = new ResourceIndicator();
        Main.panel.addToStatusArea(this.uuid, this._indicator, 1, 'right');
    }

    disable() {
        if (this._indicator)
            this._indicator.destroy();
        this._indicator = null;
    }
}
