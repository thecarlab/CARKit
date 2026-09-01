// CARKit learning annotation: implements the behavior described by this file's package and module.
const $ = id => document.getElementById(id);

function editorIdentity() {
  let clientId;
  let name;
  try {
    clientId = localStorage.getItem("carkit-editor-client");
    name = localStorage.getItem("carkit-editor-name");
  } catch { /* Private browsing may disable storage. */ }
  if (!clientId) {
    const random = globalThis.crypto && crypto.randomUUID
      ? crypto.randomUUID().replaceAll("-", "")
      : `${Date.now()}${Math.random().toString(16).slice(2)}`;
    clientId = `student_${random}`;
  }
  name = name || `Student ${clientId.slice(-4).toUpperCase()}`;
  try {
    localStorage.setItem("carkit-editor-client", clientId);
    localStorage.setItem("carkit-editor-name", name);
  } catch { /* Identity remains valid for this browser tab. */ }
  return {clientId, name};
}

const localEditorIdentity = editorIdentity();

const state = {
  map: null,
  mapCanvas: null,
  mapRender: null,
  mapCursor: null,
  mapPose: null,
  scan: null,
  path: null,
  odom: null,
  odomHistory: [],
  socket: null,
  activeChassis: null,
  chassisTelemetrySubscription: null,
  config: null,
  mapTool: null,
  advertised: new Set(),
  formHydrated: false,
  logCursor: 0,
  logLines: [],
  compileLines: [],
  drawPending: false,
  lastDrawTime: 0,
  perceptionOverlay: null,
  controlMode: null,
  pendingControlMode: null,
  controlModeTimer: null,
  pendingCameraFrame: null,
  cameraDecodeBusy: false,
  cameraObjectUrl: null,
  editorRevision: null,
  editorProfile: null,
  editorComponent: null,
  editorFile: null,
  editorTree: null,
  editorDirty: false,
  editorLoaded: false,
  editorVersion: null,
  editorBaseContent: "",
  editorLanguage: "python",
  editorDiagnostics: [],
  editorUsers: [],
  editorSyncing: false,
  editorSyncTimer: null,
  editorPollTimer: null,
  editorLoadGeneration: 0,
  editorClientId: localEditorIdentity.clientId,
  editorName: localEditorIdentity.name,
  lastChassisMessage: 0,
  lastBatteryMessage: 0,
  view: {zoom: 1, rotation: 0, panX: 0, panY: 0},
  viewDrag: null,
  viewMoved: false,
  poseDrag: null,
};

const components = ["chassis", "sensors", "planning", "control", "perception", "behavior"];
const profileNames = {
  ada_high_school: "ADA Academy",
  intro2av: "Intro2AV",
  reference: "Reference",
};

async function api(path, body) {
  const options = body === undefined ? {} : {
    method: "POST",
    headers: {"Content-Type": "application/json"},
    body: JSON.stringify(body),
  };
  const response = await fetch(path, options);
  const data = await response.json();
  if (!response.ok) throw new Error(data.error || `HTTP ${response.status}`);
  return data;
}

function option(value, label = value) {
  const item = document.createElement("option");
  item.value = value;
  item.textContent = label;
  return item;
}

function setActivity(message, type = "") {
  const node = $("activity");
  node.className = `activity ${type}`.trim();
  node.querySelector("span").textContent = message;
}

function updateSetupLabels(profile, chassis) {
  const label = profileNames[profile] || profile || "Not selected";
  $("sidebar-profile").textContent = label;
  $("sidebar-chassis").textContent = chassis === "f1tenth" ? "F1TENTH" : "OSRacer";
  $("metric-profile").textContent = label.replace(" Academy", "");
  state.activeChassis = chassis;
  subscribeChassisTelemetry(chassis);
}

async function configure() {
  state.config = await api("/api/config");
  $("editor-user-name").value = state.editorName;
  state.config.profiles.forEach(profile => $("profile").append(option(profile.id, profile.label)));
  state.config.chassis.forEach(chassis => {
    $("chassis").append(option(chassis, chassis === "f1tenth" ? "F1TENTH / VESC" : "OSRacer"));
  });
  ["planning", "control", "perception"].forEach(id => {
    state.config.implementations.forEach(value => $(id).append(option(value, implementationLabel(value))));
  });
  state.config.perception_models.forEach(model => {
    $("perception-model").append(option(model.id, model.label));
  });
  const mapFiles = state.config.maps || [];
  if (mapFiles.length) {
    mapFiles.forEach(mapFile => $("map").append(option(mapFile.path, mapFile.name)));
    $("map").value = state.config.default_map || mapFiles[0].path;
  } else {
    const unavailable = option("", "No map files found");
    unavailable.disabled = true;
    $("map").append(unavailable);
  }
  updateMapSelectionDisplay();
  updateActiveMapFile($("map").value);
  $("perception-model").value = "combined";
  updatePerceptionModelHelp();
  components.forEach(name => {
    const label = document.createElement("label");
    label.innerHTML = `<input type="checkbox" id="component-${name}" checked><span>${name}</span>`;
    $("components").append(label);
  });
  [["start-camera", "camera"], ["start-lidar", "lidar"]].forEach(([id, labelText]) => {
    const label = document.createElement("label");
    label.innerHTML = `<input type="checkbox" id="${id}" checked><span>${labelText}</span>`;
    $("components").append(label);
  });
  $("profile").value = "ada_high_school";
  updateProfileHelp();
  updateSetupLabels($("profile").value, $("chassis").value);
}

function implementationLabel(value) {
  return {
    reference: "Reference",
    ada_academy: "ADA Academy",
    intro2av_python: "Intro2AV · Python",
    intro2av_cpp: "Intro2AV · C++",
    off: "Off",
  }[value] || value;
}

function updateProfileHelp() {
  const profile = state.config
    ? state.config.profiles.find(item => item.id === $("profile").value)
    : null;
  $("profile-help").textContent = profile ? profile.help : "";
  Object.entries((profile && profile.implementations) || {}).forEach(([name, value]) => {
    if ($(name)) $(name).value = value;
  });
  if ($("component-behavior")) $("component-behavior").checked = Boolean(profile && profile.behavior);
  updateSetupLabels($("profile").value, $("chassis").value);
}

function updatePerceptionModelHelp() {
  const model = state.config
    ? state.config.perception_models.find(
      item => item.id === $("perception-model").value
    )
    : null;
  $("perception-model-help").textContent = model ? model.help : "";
  $("custom-model-field").hidden = $("perception-model").value !== "custom";
}

function mapFileName(path) {
  return path ? path.split("/").filter(Boolean).pop() : "—";
}

function updateActiveMapFile(path) {
  const name = mapFileName(path);
  $("active-map-file").textContent = `Map · ${name}`;
  $("active-map-file").title = path || "No map selected";
}

function updateMapSelectionDisplay(path = $("map").value) {
  $("map-file-path").textContent = path || "No .yaml maps found in the map folder.";
}

function applyLaunchConfig(config) {
  if (!config || state.formHydrated) return;
  if (config.profile) $("profile").value = config.profile;
  if (config.chassis) $("chassis").value = config.chassis;
  updateProfileHelp();
  Object.entries(config.implementations || {}).forEach(([name, value]) => {
    if ($(name)) $(name).value = value;
  });
  if (config.perception_model) {
    $("perception-model").value = config.perception_model;
  }
  if (config.custom_perception_model_path) {
    $("custom-model-path").value = config.custom_perception_model_path;
  }
  updatePerceptionModelHelp();
  Object.entries(config.components || {}).forEach(([name, value]) => {
    if ($(`component-${name}`)) $(`component-${name}`).checked = Boolean(value);
  });
  if (typeof config.camera === "boolean") $("start-camera").checked = config.camera;
  if (typeof config.lidar === "boolean") $("start-lidar").checked = config.lidar;
  if (config.map && Array.from($("map").options).some(item => item.value === config.map)) {
    $("map").value = config.map;
  }
  updateMapSelectionDisplay();
  updateActiveMapFile(config.map || $("map").value);
  state.formHydrated = true;
}

function requestBody() {
  return {
    profile: $("profile").value,
    chassis: $("chassis").value,
    map: $("map").value,
    implementations: {
      planning: $("planning").value,
      control: $("control").value,
      perception: $("perception").value,
    },
    perception_model: $("perception-model").value,
    custom_perception_model_path: $("custom-model-path").value.trim(),
    components: Object.fromEntries(components.map(name => [name, $(`component-${name}`).checked])),
    camera: $("start-camera").checked,
    lidar: $("start-lidar").checked,
  };
}

function openDrawer() {
  $("drawer-scrim").hidden = false;
  $("config-drawer").setAttribute("aria-hidden", "false");
  requestAnimationFrame(() => $("config-drawer").classList.add("open"));
  document.body.style.overflow = "hidden";
}

function closeDrawer() {
  $("config-drawer").classList.remove("open");
  $("config-drawer").setAttribute("aria-hidden", "true");
  document.body.style.overflow = "";
  setTimeout(() => { $("drawer-scrim").hidden = true; }, 220);
}

// The native CARKit C++ bridge sends binary CBOR so JPEGs remain byte strings
// and large scan/map arrays avoid JSON parsing. This decoder intentionally
// covers the definite-length values emitted by that fixed-protocol bridge.
function decodeCbor(buffer) {
  const bytes = buffer instanceof Uint8Array ? buffer : new Uint8Array(buffer);
  const view = new DataView(bytes.buffer, bytes.byteOffset, bytes.byteLength);
  const decoder = new TextDecoder();
  let offset = 0;

  function readLength(additional) {
    if (additional < 24) return additional;
    if (additional === 24) return view.getUint8(offset++);
    if (additional === 25) {
      const value = view.getUint16(offset, false);
      offset += 2;
      return value;
    }
    if (additional === 26) {
      const value = view.getUint32(offset, false);
      offset += 4;
      return value;
    }
    if (additional === 27) {
      const high = view.getUint32(offset, false);
      const low = view.getUint32(offset + 4, false);
      offset += 8;
      const value = high * 0x100000000 + low;
      if (!Number.isSafeInteger(value)) throw new Error("CBOR integer is too large");
      return value;
    }
    throw new Error("Indefinite-length CBOR is not supported");
  }

  function readFloat16() {
    const value = view.getUint16(offset, false);
    offset += 2;
    const sign = value & 0x8000 ? -1 : 1;
    const exponent = (value >> 10) & 0x1f;
    const fraction = value & 0x3ff;
    if (exponent === 0) return sign * 2 ** -14 * (fraction / 1024);
    if (exponent === 31) return fraction ? Number.NaN : sign * Number.POSITIVE_INFINITY;
    return sign * 2 ** (exponent - 15) * (1 + fraction / 1024);
  }

  function readItem() {
    if (offset >= bytes.length) throw new Error("Unexpected end of CBOR data");
    const initial = view.getUint8(offset++);
    const major = initial >> 5;
    const additional = initial & 0x1f;
    if (major === 0) return readLength(additional);
    if (major === 1) return -1 - readLength(additional);
    if (major === 2) {
      const length = readLength(additional);
      const value = bytes.subarray(offset, offset + length);
      offset += length;
      return value;
    }
    if (major === 3) {
      const length = readLength(additional);
      const value = decoder.decode(bytes.subarray(offset, offset + length));
      offset += length;
      return value;
    }
    if (major === 4) {
      return Array.from({length: readLength(additional)}, readItem);
    }
    if (major === 5) {
      const value = {};
      const length = readLength(additional);
      for (let index = 0; index < length; index += 1) {
        value[readItem()] = readItem();
      }
      return value;
    }
    if (major === 6) {
      readLength(additional);
      return readItem();
    }
    if (major !== 7) throw new Error("Unsupported CBOR value");
    if (additional === 20) return false;
    if (additional === 21) return true;
    if (additional === 22 || additional === 23) return null;
    if (additional === 25) return readFloat16();
    if (additional === 26) {
      const value = view.getFloat32(offset, false);
      offset += 4;
      return value;
    }
    if (additional === 27) {
      const value = view.getFloat64(offset, false);
      offset += 8;
      return value;
    }
    throw new Error("Unsupported CBOR simple value");
  }

  const value = readItem();
  if (offset !== bytes.length) throw new Error("Trailing CBOR data");
  return value;
}

function connectRos() {
  if (state.socket && state.socket.readyState < 2) return;
  const socket = new WebSocket(`ws://${location.hostname}:9090`);
  socket.binaryType = "arraybuffer";
  state.socket = socket;
  socket.onopen = () => {
    $("connection-text").textContent = "ROS connected";
    document.querySelector(".connection").classList.add("live");
    const subscriptions = [
      ["/map", "nav_msgs/msg/OccupancyGrid", 500],
      ["/scan", "sensor_msgs/msg/LaserScan", 33],
      ["/plan", "nav_msgs/msg/Path", 100],
      ["/odom", "nav_msgs/msg/Odometry", 40],
      ["/amcl_pose", "geometry_msgs/msg/PoseWithCovarianceStamped", 50],
      ["/control_center/main_state", "std_msgs/msg/String", 250],
      ["/ackermann_cmd", "ackermann_msgs/msg/AckermannDriveStamped", 50],
      ["/camera/camera/color/image_raw/compressed", "sensor_msgs/msg/CompressedImage", 0],
      ["/yolo/detections_2d", "carkit_perception_msgs/msg/YoloDetection2DArray", 0],
    ];
    subscriptions.forEach(([topic, type, throttle_rate]) => {
      const binaryImage = type === "sensor_msgs/msg/CompressedImage";
      socket.send(JSON.stringify({
        op: "subscribe",
        id: topic,
        topic,
        type,
        throttle_rate,
        queue_length: 1,
        ...(binaryImage ? {compression: "cbor"} : {}),
      }));
    });
    subscribeChassisTelemetry(state.activeChassis || $("chassis").value);
  };
  socket.onclose = () => {
    state.advertised.clear();
    state.chassisTelemetrySubscription = null;
    state.perceptionOverlay = null;
    state.pendingControlMode = null;
    renderControlMode(null);
    document.querySelector(".connection").classList.remove("live");
    $("connection-text").textContent = "ROS disconnected";
    setTimeout(connectRos, 1800);
  };
  socket.onerror = () => socket.close();
  socket.onmessage = event => {
    let packet;
    try {
      packet = typeof event.data === "string"
        ? JSON.parse(event.data)
        : decodeCbor(event.data);
    } catch { return; }
    if (packet.op === "publish") handleRos(packet.topic, packet.msg);
  };
}

function subscribeChassisTelemetry(chassis) {
  const socket = state.socket;
  if (!socket || socket.readyState !== WebSocket.OPEN) return;
  const selection = chassis === "f1tenth"
    ? ["/sensors/core", "vesc_msgs/msg/VescStateStamped"]
    : ["/battery_state", "sensor_msgs/msg/BatteryState"];
  if (state.chassisTelemetrySubscription === selection[0]) return;
  if (state.chassisTelemetrySubscription) {
    socket.send(JSON.stringify({
      op: "unsubscribe",
      id: state.chassisTelemetrySubscription,
      topic: state.chassisTelemetrySubscription,
    }));
  }
  socket.send(JSON.stringify({
    op: "subscribe",
    id: selection[0],
    topic: selection[0],
    type: selection[1],
    throttle_rate: 500,
    queue_length: 1,
  }));
  state.chassisTelemetrySubscription = selection[0];
}

function markHealth(id, label = "Live") {
  const item = $(`health-${id}`);
  item.classList.add("live");
  item.querySelector("b").textContent = label;
}

function finiteNumber(value) {
  if (value === null || value === undefined || value === "") return null;
  const number = Number(value);
  return Number.isFinite(number) ? number : null;
}

function formatBytes(value) {
  const bytes = finiteNumber(value);
  if (bytes === null) return "—";
  return `${(bytes / (1024 ** 3)).toFixed(1)} GB`;
}

function formatUptime(seconds) {
  const value = finiteNumber(seconds);
  if (value === null) return "Up —";
  const days = Math.floor(value / 86400);
  const hours = Math.floor((value % 86400) / 3600);
  return days ? `Up ${days}d ${hours}h` : `Up ${hours}h`;
}

function renderSystemTelemetry(system) {
  const cpu = finiteNumber(system && system.cpu_percent);
  const cpuCount = finiteNumber(system && system.cpu_count);
  const cpuCapacity = finiteNumber(system && system.cpu_capacity_percent) || 100;
  const memory = system && system.memory;
  const temperature = finiteNumber(system && system.cpu_temperature_c);
  $("cpu-usage").textContent = cpu === null
    ? "Sampling…"
    : `${cpu.toFixed(1)}%`;
  const load = system && Array.isArray(system.load_average)
    ? finiteNumber(system.load_average[0])
    : null;
  const busyCores = cpu === null ? null : cpu / 100;
  const coreLabel = cpuCount === null
    ? "cores —"
    : `${busyCores === null ? "—" : busyCores.toFixed(1)} of ${cpuCount.toFixed(0)} cores busy`;
  $("cpu-load").textContent = load === null
    ? `${coreLabel} · ${cpuCapacity.toFixed(0)}% max`
    : `${coreLabel} · ${cpuCapacity.toFixed(0)}% max · Load ${load.toFixed(1)}`;
  $("memory-usage").textContent = memory ? `${Number(memory.percent).toFixed(1)}%` : "—";
  $("memory-detail").textContent = memory
    ? `${formatBytes(memory.used_bytes)} / ${formatBytes(memory.total_bytes)}`
    : "— / —";
  $("cpu-temperature").textContent = temperature === null ? "—" : `${temperature.toFixed(1)}°C`;
  $("system-uptime").textContent = formatUptime(system && system.uptime_seconds);
  $("telemetry-cpu").classList.toggle(
    "warning",
    cpu !== null && cpu >= cpuCapacity * 0.85,
  );
  $("telemetry-memory").classList.toggle(
    "warning", Boolean(memory && Number(memory.percent) >= 85),
  );
  $("telemetry-temperature").classList.toggle(
    "warning", temperature !== null && temperature >= 75,
  );
}

function renderBattery(message) {
  const voltage = finiteNumber(message.voltage);
  const percentage = finiteNumber(message.percentage);
  state.lastBatteryMessage = Date.now();
  state.lastChassisMessage = Date.now();
  $("battery-voltage").textContent = voltage === null ? "—" : `${voltage.toFixed(2)} V`;
  $("battery-level").textContent = percentage !== null && percentage >= 0
    ? `${Math.round(percentage * 100)}% estimated`
    : "Voltage only";
  const low = (percentage !== null && percentage >= 0 && percentage <= 0.2)
    || (voltage !== null && voltage <= 11.1);
  $("telemetry-battery").classList.toggle("warning", low);
}

function renderChassisStatus(running) {
  const fresh = Date.now() - state.lastChassisMessage < 3000;
  const chip = $("telemetry-chassis");
  chip.classList.toggle("live", running && fresh);
  chip.classList.toggle("warning", running && !fresh);
  $("chassis-status").textContent = !running
    ? "Stopped"
    : fresh
      ? "Online"
      : "Waiting";
  if (!running) {
    state.lastChassisMessage = 0;
    state.lastBatteryMessage = 0;
    $("battery-voltage").textContent = "—";
    $("battery-level").textContent = "No data";
    $("telemetry-battery").classList.remove("warning");
  } else if (Date.now() - state.lastBatteryMessage >= 3000) {
    $("battery-voltage").textContent = "—";
    $("battery-level").textContent = "No battery topic";
  }
}

function publishRos(topic, type, msg) {
  const socket = state.socket;
  if (!socket || socket.readyState !== WebSocket.OPEN) {
    setActivity("ROS WebSocket is not connected", "error");
    return false;
  }
  if (!state.advertised.has(topic)) {
    socket.send(JSON.stringify({op: "advertise", topic, type}));
    state.advertised.add(topic);
  }
  socket.send(JSON.stringify({op: "publish", topic, msg}));
  return true;
}

function renderControlMode(mode) {
  const validMode = ["HUMAN_CONTROL", "AUTO_DRIVE", "EMERGENCY_STOP"].includes(mode)
    ? mode
    : null;
  state.controlMode = validMode;

  if (validMode === "EMERGENCY_STOP") {
    state.pendingControlMode = null;
  } else if (state.pendingControlMode === validMode) {
    state.pendingControlMode = null;
  }
  if (!state.pendingControlMode && state.controlModeTimer) {
    clearTimeout(state.controlModeTimer);
    state.controlModeTimer = null;
  }

  const human = $("mode-human");
  const autonomous = $("mode-autonomous");
  const pending = state.pendingControlMode;
  const locked = !validMode || validMode === "EMERGENCY_STOP" || Boolean(pending);
  human.disabled = locked;
  autonomous.disabled = locked;
  human.classList.toggle("active", validMode === "HUMAN_CONTROL");
  autonomous.classList.toggle("active", validMode === "AUTO_DRIVE");
  human.setAttribute("aria-pressed", String(validMode === "HUMAN_CONTROL"));
  autonomous.setAttribute("aria-pressed", String(validMode === "AUTO_DRIVE"));

  const switchNode = document.querySelector(".control-mode-switch");
  switchNode.classList.toggle("estop", validMode === "EMERGENCY_STOP");
  switchNode.classList.toggle("pending", Boolean(pending));
  $("mode-source").textContent = pending
    ? pending === "AUTO_DRIVE" ? "Switching to Ackermann /drive…" : "Switching to remote /teleop…"
    : validMode === "HUMAN_CONTROL"
      ? "Remote controller · /teleop"
      : validMode === "AUTO_DRIVE"
        ? "Ackermann commands · /drive"
        : validMode === "EMERGENCY_STOP"
          ? "Emergency stop active"
          : "Waiting for control center";
}

function requestControlMode(mode) {
  if (!["HUMAN_CONTROL", "AUTO_DRIVE"].includes(mode)) return;
  if (!state.controlMode || state.controlMode === "EMERGENCY_STOP") return;
  if (state.controlMode === mode && !state.pendingControlMode) return;
  const enabled = mode === "AUTO_DRIVE" ? 1 : 0;
  if (!publishRos("/enable_autonomous_control", "std_msgs/msg/Int8", {data: enabled})) return;

  state.pendingControlMode = mode;
  renderControlMode(state.controlMode);
  clearTimeout(state.controlModeTimer);
  state.controlModeTimer = setTimeout(() => {
    if (state.pendingControlMode !== mode) return;
    state.pendingControlMode = null;
    state.controlModeTimer = null;
    renderControlMode(state.controlMode);
    setActivity("Control center did not confirm the mode change", "error");
  }, 1600);
}

function renderNextCameraFrame() {
  const frame = state.pendingCameraFrame;
  if (!frame || state.cameraDecodeBusy) return;
  state.pendingCameraFrame = null;
  state.cameraDecodeBusy = true;
  let objectUrl;
  try {
    let bytes = frame.data;
    // Retain JSON/base64 as a compatibility fallback for an older deployment.
    if (typeof bytes === "string") {
      const binary = atob(bytes);
      bytes = new Uint8Array(binary.length);
      for (let index = 0; index < binary.length; index += 1) {
        bytes[index] = binary.charCodeAt(index);
      }
    }
    objectUrl = URL.createObjectURL(new Blob([bytes], {type: `image/${frame.format}`}));
  } catch {
    state.cameraDecodeBusy = false;
    renderNextCameraFrame();
    return;
  }
  const image = $("camera");
  image.onload = () => {
    if (state.cameraObjectUrl) URL.revokeObjectURL(state.cameraObjectUrl);
    state.cameraObjectUrl = objectUrl;
    state.cameraDecodeBusy = false;
    image.style.display = "block";
    $("camera-empty").style.display = "none";
    document.querySelector(".camera-frame").classList.add("streaming");
    $("camera-status").textContent = state.perceptionOverlay
      ? "Detections · 10 Hz"
      : "Camera · 10 Hz";
    drawPerceptionOverlay();
    renderNextCameraFrame();
  };
  image.onerror = () => {
    URL.revokeObjectURL(objectUrl);
    state.cameraDecodeBusy = false;
    renderNextCameraFrame();
  };
  image.src = objectUrl;
}

function queueCameraFrame(message) {
  state.pendingCameraFrame = {
    data: message.data,
    format: (message.format || "jpeg").includes("png") ? "png" : "jpeg",
  };
  renderNextCameraFrame();
}

function drawPerceptionOverlay() {
  const canvas = $("perception-overlay");
  const image = $("camera");
  const width = canvas.clientWidth;
  const height = canvas.clientHeight;
  const scaleFactor = Math.min(window.devicePixelRatio || 1, 2);
  const pixelWidth = Math.max(1, Math.round(width * scaleFactor));
  const pixelHeight = Math.max(1, Math.round(height * scaleFactor));
  if (canvas.width !== pixelWidth || canvas.height !== pixelHeight) {
    canvas.width = pixelWidth;
    canvas.height = pixelHeight;
  }
  const context = canvas.getContext("2d");
  context.setTransform(scaleFactor, 0, 0, scaleFactor, 0, 0);
  context.clearRect(0, 0, width, height);
  const overlay = state.perceptionOverlay;
  if (!overlay || !image.complete || !image.naturalWidth || !width || !height) return;

  const imageWidth = overlay.imageWidth || image.naturalWidth;
  const imageHeight = overlay.imageHeight || image.naturalHeight;
  const imageScale = Math.min(width / imageWidth, height / imageHeight);
  const offsetX = (width - imageWidth * imageScale) / 2;
  const offsetY = (height - imageHeight * imageScale) / 2;
  context.font = "700 10px ui-monospace, SFMono-Regular, Consolas, monospace";
  context.lineWidth = 2;

  for (const detection of overlay.detections) {
    const xMin = Number(detection.bbox_x_min || 0);
    const yMin = Number(detection.bbox_y_min || 0);
    const xMax = Number(detection.bbox_x_max || 0);
    const yMax = Number(detection.bbox_y_max || 0);
    const x = offsetX + xMin * imageScale;
    const y = offsetY + yMin * imageScale;
    const boxWidth = (xMax - xMin) * imageScale;
    const boxHeight = (yMax - yMin) * imageScale;
    if (boxWidth <= 0 || boxHeight <= 0) continue;
    const confidence = Number(detection.confidence);
    const label = `${detection.class_name || "object"}${
      Number.isFinite(confidence) ? ` ${confidence.toFixed(2)}` : ""
    }`;
    context.strokeStyle = "#25ff74";
    context.strokeRect(x, y, boxWidth, boxHeight);
    const labelWidth = context.measureText(label).width + 8;
    const labelY = Math.max(0, y - 16);
    context.fillStyle = "rgba(7, 32, 39, .88)";
    context.fillRect(x, labelY, labelWidth, 16);
    context.fillStyle = "#ffffff";
    context.fillText(label, x + 4, labelY + 11);
  }
}

function updateEditorLines() {
  const count = Math.max(1, $("code-editor").value.split("\n").length);
  $("editor-lines").textContent = Array.from(
    {length: count},
    (_, index) => index + 1,
  ).join("\n");
}

function escapeCode(value) {
  return value
    .replaceAll("&", "&amp;")
    .replaceAll("<", "&lt;")
    .replaceAll(">", "&gt;");
}

function highlightedLine(line, language) {
  const python = /(#[^\n]*|"(?:\\.|[^"\\])*"|'(?:\\.|[^'\\])*'|\b(?:False|None|True|and|as|assert|async|await|break|class|continue|def|del|elif|else|except|finally|for|from|global|if|import|in|is|lambda|nonlocal|not|or|pass|raise|return|try|while|with|yield)\b|\b(?:bool|bytes|dict|float|int|list|object|set|str|tuple)\b|\b\d+(?:\.\d+)?\b|@[A-Za-z_]\w*)/g;
  const cpp = /(\/\/.*|\/\*.*?\*\/|"(?:\\.|[^"\\])*"|'(?:\\.|[^'\\])*'|^\s*#\s*\w+|\b(?:alignas|auto|break|case|catch|class|const|constexpr|continue|default|delete|do|else|enum|explicit|false|for|if|namespace|new|nullptr|override|private|protected|public|return|sizeof|static|struct|switch|template|this|throw|true|try|using|virtual|while)\b|\b(?:bool|char|double|float|int|long|short|size_t|string|uint\d+_t|void)\b|\b\d+(?:\.\d+)?[fFuUlL]*\b)/g;
  const generic = /(#.*|<\/?[A-Za-z][^>]*>|"(?:\\.|[^"\\])*"|'(?:\\.|[^'\\])*'|\b(?:false|null|true|OFF|ON|REQUIRED)\b|\b\d+(?:\.\d+)?\b)/g;
  if (["md", "text", "txt"].includes(language)) return escapeCode(line);
  const expression = language === "cpp" ? cpp : language === "python" ? python : generic;
  let output = "";
  let position = 0;
  for (const match of line.matchAll(expression)) {
    output += escapeCode(line.slice(position, match.index));
    const token = match[0];
    let kind = "keyword";
    if (language === "cpp" && /^\s*#/.test(token)) kind = "preprocessor";
    else if (/^(#|\/\/|\/\*)/.test(token.trimStart())) kind = "comment";
    else if (/^["']/.test(token)) kind = "string";
    else if (/^\s*#/.test(token)) kind = "preprocessor";
    else if (/^@/.test(token)) kind = "decorator";
    else if (/^\d/.test(token)) kind = "number";
    else if (/^(bool|bytes|dict|float|int|list|object|set|str|tuple|char|double|long|short|size_t|string|uint\d+_t|void)$/.test(token)) kind = "type";
    output += `<span class="syntax-${kind}">${escapeCode(token)}</span>`;
    position = match.index + token.length;
  }
  return output + escapeCode(line.slice(position));
}

function renderEditorHighlight() {
  const diagnosticLines = new Set(
    (state.editorDiagnostics || []).map(issue => Number(issue.line)),
  );
  $("editor-highlight").innerHTML = $("code-editor").value.split("\n")
    .map((line, index) => {
      const error = diagnosticLines.has(index + 1) ? " diagnostic-line" : "";
      return `<span class="code-line${error}">${highlightedLine(line, state.editorLanguage) || " "}</span>`;
    })
    .join("\n") + "\n";
  renderRemoteCursors();
}

function cursorLineAndColumn(content, position) {
  const prefix = content.slice(0, Math.max(0, Math.min(position, content.length)));
  const lines = prefix.split("\n");
  return {line: lines.length - 1, column: lines[lines.length - 1].length};
}

function renderRemoteCursors() {
  const container = $("remote-cursors");
  if (!container || !state.editorLoaded) return;
  const editor = $("code-editor");
  const content = editor.value;
  container.replaceChildren();
  (state.editorUsers || [])
    .filter(user => user.client_id !== state.editorClientId)
    .forEach(user => {
      const location = cursorLineAndColumn(content, user.cursor);
      const caret = document.createElement("div");
      caret.className = "remote-caret";
      caret.style.setProperty("--cursor-color", user.color);
      caret.style.left = `${16 + location.column * 7.22 - editor.scrollLeft}px`;
      caret.style.top = `${14 + location.line * 18.6 - editor.scrollTop}px`;
      const label = document.createElement("span");
      label.textContent = user.name;
      caret.append(label);
      container.append(caret);
    });
}

function renderEditorPresence(users) {
  state.editorUsers = users || [];
  const container = $("editor-presence");
  container.replaceChildren();
  state.editorUsers.slice(0, 5).forEach(user => {
    const person = document.createElement("span");
    person.className = "presence-person";
    person.style.setProperty("--presence-color", user.color);
    const dot = document.createElement("i");
    const location = cursorLineAndColumn($("code-editor").value, user.cursor);
    person.append(dot, document.createTextNode(
      `${user.client_id === state.editorClientId ? "You" : user.name} · L${location.line + 1}`,
    ));
    container.append(person);
  });
  renderRemoteCursors();
}

function renderDiagnostics(diagnostics) {
  state.editorDiagnostics = diagnostics || [];
  const node = $("editor-diagnostics");
  if (!state.editorDiagnostics.length) {
    node.textContent = "No syntax errors";
    node.className = "";
  } else {
    const first = state.editorDiagnostics[0];
    node.textContent = `${first.message} · line ${first.line}:${first.column}`;
    node.className = "error";
  }
  renderEditorHighlight();
}

function syncEditorScroll() {
  const editor = $("code-editor");
  $("editor-lines").scrollTop = editor.scrollTop;
  $("editor-highlight").scrollTop = editor.scrollTop;
  $("editor-highlight").scrollLeft = editor.scrollLeft;
  renderRemoteCursors();
}

function setEditorStatus(text, type = "neutral") {
  $("editor-status").textContent = text;
  $("editor-status").className = `status-pill ${type}`;
}

function renderFileExplorer(tree) {
  const container = $("editor-file-tree");
  container.replaceChildren();
  $("editor-root").textContent = tree.root;
  const root = {directories: new Map(), files: []};
  tree.files.forEach(file => {
    const parts = file.path.split("/");
    let node = root;
    parts.slice(0, -1).forEach(part => {
      if (!node.directories.has(part)) {
        node.directories.set(part, {directories: new Map(), files: []});
      }
      node = node.directories.get(part);
    });
    node.files.push(file);
  });

  const appendNode = (node, parent, depth) => {
    [...node.directories.entries()].forEach(([name, child]) => {
      const folder = document.createElement("button");
      folder.type = "button";
      folder.className = "explorer-folder";
      folder.style.paddingLeft = `${7 + depth * 12}px`;
      const icon = document.createElement("span");
      icon.className = "explorer-icon";
      icon.textContent = "▾";
      folder.append(icon, document.createTextNode(name));
      const children = document.createElement("div");
      children.className = "explorer-children";
      folder.addEventListener("click", () => {
        children.hidden = !children.hidden;
        icon.textContent = children.hidden ? "▸" : "▾";
      });
      parent.append(folder, children);
      appendNode(child, children, depth + 1);
    });
    node.files.forEach(file => {
      const button = document.createElement("button");
      button.type = "button";
      button.className = "explorer-file";
      button.dataset.file = file.path;
      button.dataset.language = file.language;
      button.style.paddingLeft = `${7 + depth * 12}px`;
      const icon = document.createElement("span");
      icon.className = "explorer-icon";
      icon.textContent = file.language === "python" ? "Py" : file.language === "cpp" ? "C+" : "·";
      button.append(icon, document.createTextNode(file.name));
      button.title = file.path;
      button.addEventListener("click", () => loadEditorFile(false, file.path));
      parent.append(button);
    });
  };
  appendNode(root, container, 0);
  updateExplorerActive();
}

function updateExplorerActive() {
  document.querySelectorAll(".explorer-file").forEach(button => {
    button.classList.toggle("active", button.dataset.file === state.editorFile);
  });
}

function preferredAlgorithmFile(profile, component, tree) {
  const candidates = {
    intro2av_python: `carkit_intro2av/${component}_algorithm.py`,
    intro2av_cpp: `src/${component}_algorithm.cpp`,
  };
  const preferred = candidates[profile];
  if (preferred && tree.files.some(file => file.path === preferred)) {
    return preferred;
  }
  return tree.defaults[component] || (tree.files[0] && tree.files[0].path);
}

async function loadEditorWorkspace(force = false) {
  if (state.editorDirty && !force) await synchronizeEditor();
  const profile = $("editor-profile").value;
  const component = $("editor-component").value;
  setEditorStatus("Opening package");
  try {
    const tree = await api(`/api/editor/tree?profile=${encodeURIComponent(profile)}`);
    state.editorTree = tree;
    renderFileExplorer(tree);
    const initialFile = preferredAlgorithmFile(profile, component, tree);
    if (!initialFile) throw new Error("No editable files were found in this package");
    return loadEditorFile(force, initialFile);
  } catch (error) {
    setEditorStatus("Explorer failed", "error");
    $("editor-revision").textContent = error.message;
    return false;
  }
}

async function loadEditorFile(force = false, requestedFile = null) {
  if (state.editorDirty && !force) await synchronizeEditor();
  const previousProfile = state.editorProfile;
  const previousComponent = state.editorComponent;
  const previousFile = state.editorFile;
  if (previousProfile && previousComponent && previousFile) {
    api("/api/editor/leave", {
      profile: previousProfile,
      component: previousComponent,
      client_id: state.editorClientId,
      file: previousFile,
    }).catch(() => {});
  }
  const profile = $("editor-profile").value;
  const component = $("editor-component").value;
  const filePath = requestedFile
    || state.editorFile
    || (state.editorTree && state.editorTree.defaults[component]);
  const generation = ++state.editorLoadGeneration;
  setEditorStatus("Loading");
  $("editor-save").disabled = true;
  try {
    const file = await api(
      `/api/editor/collab?profile=${encodeURIComponent(profile)}&component=${encodeURIComponent(component)}&file=${encodeURIComponent(filePath)}&client_id=${encodeURIComponent(state.editorClientId)}&name=${encodeURIComponent(state.editorName)}`
    );
    if (generation !== state.editorLoadGeneration) return false;
    $("code-editor").value = file.content;
    $("editor-path").textContent = file.path;
    $("editor-revision").textContent = `Shared version ${file.version} · ${file.language.toUpperCase()}`;
    state.editorRevision = file.revision;
    state.editorVersion = file.version;
    state.editorBaseContent = file.content;
    state.editorLanguage = file.language;
    state.editorProfile = profile;
    state.editorComponent = component;
    state.editorFile = file.file;
    state.editorDirty = false;
    state.editorLoaded = true;
    updateEditorLines();
    renderDiagnostics(file.diagnostics);
    renderEditorPresence(file.users);
    updateExplorerActive();
    setEditorStatus("Live", "live");
    clearTimeout(state.editorPollTimer);
    state.editorPollTimer = setTimeout(pollCollaborativeEditor, 400);
    return true;
  } catch (error) {
    setEditorStatus("Load failed", "error");
    $("editor-revision").textContent = error.message;
    return false;
  }
}

function oneChange(before, after) {
  let start = 0;
  while (start < before.length && start < after.length && before[start] === after[start]) start += 1;
  let beforeEnd = before.length;
  let afterEnd = after.length;
  while (beforeEnd > start && afterEnd > start && before[beforeEnd - 1] === after[afterEnd - 1]) {
    beforeEnd -= 1;
    afterEnd -= 1;
  }
  return {start, end: beforeEnd, text: after.slice(start, afterEnd)};
}

function transformPosition(position, operation) {
  if (operation.start === operation.end) {
    return position < operation.start ? position : position + operation.text.length;
  }
  if (position <= operation.start) return position;
  if (position >= operation.end) {
    return position + operation.text.length - (operation.end - operation.start);
  }
  return operation.start + operation.text.length;
}

function preserveNewTyping(sentContent, currentContent, canonicalContent) {
  if (sentContent === currentContent) return canonicalContent;
  const local = oneChange(sentContent, currentContent);
  const remote = oneChange(sentContent, canonicalContent);
  local.start = transformPosition(local.start, remote);
  local.end = transformPosition(local.end, remote);
  return canonicalContent.slice(0, local.start)
    + local.text
    + canonicalContent.slice(Math.max(local.start, local.end));
}

async function synchronizeEditor() {
  if (!state.editorLoaded || state.editorVersion === null || state.editorSyncing) return;
  clearTimeout(state.editorSyncTimer);
  const editor = $("code-editor");
  const sentContent = editor.value;
  const sentVersion = state.editorVersion;
  const syncTarget = `${state.editorProfile}:${state.editorFile}`;
  state.editorSyncing = true;
  setEditorStatus("Syncing");
  $("editor-save").disabled = true;
  try {
    const result = await api("/api/editor/sync", {
      profile: state.editorProfile,
      component: state.editorComponent,
      file: state.editorFile,
      client_id: state.editorClientId,
      name: state.editorName,
      base_version: sentVersion,
      content: sentContent,
      cursor: editor.selectionStart,
      selection: editor.selectionEnd,
    });
    if (`${state.editorProfile}:${state.editorFile}` !== syncTarget) return;
    const currentContent = editor.value;
    editor.value = preserveNewTyping(sentContent, currentContent, result.content);
    state.editorRevision = result.revision;
    state.editorVersion = result.version;
    state.editorBaseContent = result.content;
    state.editorDirty = editor.value !== result.content;
    $("editor-revision").textContent = `Shared version ${result.version} · ${result.language.toUpperCase()}`;
    updateEditorLines();
    renderDiagnostics(result.diagnostics);
    renderEditorPresence(result.users);
    setEditorStatus(state.editorDirty ? "Syncing" : "Live", state.editorDirty ? "neutral" : "live");
  } catch (error) {
    setEditorStatus(error.message.includes("history expired") ? "Reload required" : "Sync failed", "error");
    $("editor-revision").textContent = error.message;
  } finally {
    state.editorSyncing = false;
    $("editor-save").disabled = !state.editorDirty;
    if (state.editorDirty) {
      state.editorSyncTimer = setTimeout(synchronizeEditor, 100);
    }
  }
}

async function pollCollaborativeEditor() {
  clearTimeout(state.editorPollTimer);
  if (!state.editorLoaded || state.editorSyncing) {
    state.editorPollTimer = setTimeout(pollCollaborativeEditor, 350);
    return;
  }
  const editor = $("code-editor");
  const pollTarget = `${state.editorProfile}:${state.editorFile}`;
  try {
    const snapshot = await api(
      `/api/editor/collab?profile=${encodeURIComponent(state.editorProfile)}&component=${encodeURIComponent(state.editorComponent)}&file=${encodeURIComponent(state.editorFile)}&client_id=${encodeURIComponent(state.editorClientId)}&name=${encodeURIComponent(state.editorName)}&cursor=${editor.selectionStart}&selection=${editor.selectionEnd}`
    );
    if (`${state.editorProfile}:${state.editorFile}` !== pollTarget) return;
    if (snapshot.version !== state.editorVersion) {
      if (state.editorDirty) {
        await synchronizeEditor();
      } else {
        const selectionStart = editor.selectionStart;
        const selectionEnd = editor.selectionEnd;
        const operation = oneChange(editor.value, snapshot.content);
        editor.value = snapshot.content;
        editor.setSelectionRange(
          transformPosition(selectionStart, operation),
          transformPosition(selectionEnd, operation),
        );
        state.editorVersion = snapshot.version;
        state.editorRevision = snapshot.revision;
        state.editorBaseContent = snapshot.content;
        $("editor-revision").textContent = `Shared version ${snapshot.version} · ${snapshot.language.toUpperCase()}`;
        updateEditorLines();
      }
    }
    renderDiagnostics(snapshot.diagnostics);
    renderEditorPresence(snapshot.users);
    if (!state.editorDirty) setEditorStatus("Live", "live");
  } catch (error) {
    setEditorStatus("Reconnecting", "error");
  }
  state.editorPollTimer = setTimeout(pollCollaborativeEditor, 400);
}

async function saveEditorFile() {
  await synchronizeEditor();
}

function armMapTool(tool) {
  state.poseDrag = null;
  state.mapTool = state.mapTool === tool ? null : tool;
  ["set-initial", "send-goal"].forEach(id => $(id).classList.toggle("armed", id === state.mapTool));
  $("map-tool-help").textContent = !state.mapTool
    ? "Drag to rotate · Shift-drag to pan · Scroll to zoom."
    : state.mapTool === "set-initial"
      ? "Click and drag from the vehicle location toward its heading."
      : "Click and drag from the destination toward its heading.";
}

function canvasPointToWorld(clientX, clientY) {
  const canvas = $("map-canvas");
  const render = state.mapRender;
  if (!state.map || !render) return null;
  const rect = canvas.getBoundingClientRect();
  const viewX = clientX - rect.left;
  const viewY = clientY - rect.top;
  const centerX = rect.width / 2;
  const centerY = rect.height / 2;
  const shiftedX = (
    viewX - centerX - state.view.panX
  ) / state.view.zoom;
  const shiftedY = (
    viewY - centerY - state.view.panY
  ) / state.view.zoom;
  const cosine = Math.cos(state.view.rotation);
  const sine = Math.sin(state.view.rotation);
  const px = centerX + shiftedX * cosine + shiftedY * sine;
  const py = centerY - shiftedX * sine + shiftedY * cosine;
  const mapX = (px - render.offsetX) / render.scale;
  const mapY = state.map.info.height - (py - render.offsetY) / render.scale;
  if (mapX < 0 || mapY < 0 || mapX > state.map.info.width || mapY > state.map.info.height) return null;
  return {
    x: state.map.info.origin.position.x + mapX * state.map.info.resolution,
    y: state.map.info.origin.position.y + mapY * state.map.info.resolution,
  };
}

function updateMapCoordinate(clientX, clientY) {
  const point = canvasPointToWorld(clientX, clientY);
  if (!point) return;
  state.mapCursor = point;
  $("map-coordinate").textContent = `Map x ${point.x.toFixed(2)} · y ${point.y.toFixed(2)}`;
}

function startPoseDrag(event) {
  if (!state.mapTool || event.button !== 0) return false;
  const point = canvasPointToWorld(event.clientX, event.clientY);
  if (!point) {
    setActivity(state.map ? "Start the drag inside the map area" : "Wait for /map before selecting a pose", "error");
    return true;
  }
  state.poseDrag = {
    pointerId: event.pointerId,
    tool: state.mapTool,
    start: point,
    end: point,
    clientX: event.clientX,
    clientY: event.clientY,
  };
  event.currentTarget.setPointerCapture(event.pointerId);
  event.currentTarget.classList.add("pose-drag");
  event.preventDefault();
  scheduleDraw();
  return true;
}

function movePoseDrag(event) {
  if (!state.poseDrag || state.poseDrag.pointerId !== event.pointerId) return false;
  const point = canvasPointToWorld(event.clientX, event.clientY);
  if (point) state.poseDrag.end = point;
  updateMapCoordinate(event.clientX, event.clientY);
  scheduleDraw();
  return true;
}

function stopPoseDrag(event, cancelled = false) {
  const drag = state.poseDrag;
  if (!drag || drag.pointerId !== event.pointerId) return false;
  const point = canvasPointToWorld(event.clientX, event.clientY);
  if (point) drag.end = point;
  state.poseDrag = null;
  event.currentTarget.classList.remove("pose-drag");
  if (event.currentTarget.hasPointerCapture(event.pointerId)) {
    event.currentTarget.releasePointerCapture(event.pointerId);
  }
  if (cancelled) {
    scheduleDraw();
    return true;
  }
  const pixelDistance = Math.hypot(
    event.clientX - drag.clientX,
    event.clientY - drag.clientY,
  );
  if (pixelDistance < 8) {
    setActivity("Drag farther to set the vehicle heading", "error");
    scheduleDraw();
    return true;
  }
  const heading = Math.atan2(drag.end.y - drag.start.y, drag.end.x - drag.start.x);
  const halfHeading = heading / 2;
  const header = {stamp: {sec: 0, nanosec: 0}, frame_id: "map"};
  const pose = {
    position: {x: drag.start.x, y: drag.start.y, z: 0},
    orientation: {x: 0, y: 0, z: Math.sin(halfHeading), w: Math.cos(halfHeading)},
  };
  let sent;
  if (drag.tool === "set-initial") {
    const covariance = Array(36).fill(0);
    covariance[0] = .25;
    covariance[7] = .25;
    covariance[35] = .0685;
    sent = publishRos(
      "/initialpose",
      "geometry_msgs/msg/PoseWithCovarianceStamped",
      {header, pose: {pose, covariance}},
    );
    if (sent) setMapPose(pose, null);
  } else {
    sent = publishRos("/goal_pose", "geometry_msgs/msg/PoseStamped", {header, pose});
  }
  if (sent) {
    const degrees = ((heading * 180 / Math.PI) + 360) % 360;
    setActivity(
      `${drag.tool === "set-initial" ? "Initial pose" : "Goal"} sent at ${drag.start.x.toFixed(2)}, ${drag.start.y.toFixed(2)} · ${degrees.toFixed(0)}°`,
      "success",
    );
    armMapTool(drag.tool);
  }
  scheduleDraw();
  return true;
}

function cacheMap(message) {
  const offscreen = document.createElement("canvas");
  offscreen.width = message.info.width;
  offscreen.height = message.info.height;
  const context = offscreen.getContext("2d");
  const pixels = context.createImageData(offscreen.width, offscreen.height);
  for (let index = 0; index < message.data.length; index += 1) {
    const value = message.data[index];
    const offset = index * 4;
    const color = value < 0 ? 221 : value > 65 ? 35 : 249;
    pixels.data[offset] = color;
    pixels.data[offset + 1] = value < 0 ? 228 : value > 65 ? 51 : 251;
    pixels.data[offset + 2] = value < 0 ? 235 : value > 65 ? 68 : 252;
    pixels.data[offset + 3] = 255;
  }
  context.putImageData(pixels, 0, 0);
  state.mapCanvas = offscreen;
}

function stampSeconds(header) {
  const stamp = header && header.stamp;
  if (!stamp) return null;
  const seconds = Number(stamp.sec) + Number(stamp.nanosec) * 1e-9;
  return Number.isFinite(seconds) && seconds > 0 ? seconds : null;
}

function rememberOdom(message) {
  const time = stampSeconds(message.header);
  if (time === null) return;
  const pose = message.pose.pose;
  const twist = message.twist && message.twist.twist;
  state.odomHistory.push({
    time,
    x: pose.position.x,
    y: pose.position.y,
    heading: yaw(pose.orientation),
    vx: twist ? twist.linear.x : 0,
    vy: twist ? twist.linear.y : 0,
    angular: twist ? twist.angular.z : 0,
  });
  state.odomHistory.sort((left, right) => left.time - right.time);
  const cutoff = state.odomHistory[state.odomHistory.length - 1].time - 3;
  state.odomHistory = state.odomHistory
    .filter(entry => entry.time >= cutoff)
    .slice(-120);
}

function setMapPose(pose, stamp) {
  const time = stampSeconds({stamp}) || (
    state.odomHistory.length
      ? state.odomHistory[state.odomHistory.length - 1].time
      : null
  );
  state.mapPose = {
    x: pose.position.x,
    y: pose.position.y,
    heading: yaw(pose.orientation),
    odom: time === null ? null : odomPoseAt(time),
  };
}

function mapPoseAt(time) {
  const odom = odomPoseAt(time);
  if (!state.mapPose) return odom;
  const anchor = state.mapPose.odom;
  if (!anchor || !odom) {
    return {
      x: state.mapPose.x,
      y: state.mapPose.y,
      heading: state.mapPose.heading,
    };
  }
  const rotation = state.mapPose.heading - anchor.heading;
  const deltaX = odom.x - anchor.x;
  const deltaY = odom.y - anchor.y;
  return {
    x: state.mapPose.x + deltaX * Math.cos(rotation) - deltaY * Math.sin(rotation),
    y: state.mapPose.y + deltaX * Math.sin(rotation) + deltaY * Math.cos(rotation),
    heading: state.mapPose.heading + Math.atan2(
      Math.sin(odom.heading - anchor.heading),
      Math.cos(odom.heading - anchor.heading),
    ),
  };
}

function extrapolatePose(sample, time) {
  const delta = Math.max(-0.25, Math.min(0.25, time - sample.time));
  const cosine = Math.cos(sample.heading);
  const sine = Math.sin(sample.heading);
  return {
    x: sample.x + (sample.vx * cosine - sample.vy * sine) * delta,
    y: sample.y + (sample.vx * sine + sample.vy * cosine) * delta,
    heading: sample.heading + sample.angular * delta,
  };
}

function odomPoseAt(time) {
  const history = state.odomHistory;
  if (!history.length || !Number.isFinite(time)) return null;
  if (time <= history[0].time) return extrapolatePose(history[0], time);
  if (time >= history[history.length - 1].time) {
    return extrapolatePose(history[history.length - 1], time);
  }
  let low = 0;
  let high = history.length - 1;
  while (high - low > 1) {
    const middle = Math.floor((low + high) / 2);
    if (history[middle].time <= time) low = middle;
    else high = middle;
  }
  const before = history[low];
  const after = history[high];
  const ratio = (time - before.time) / (after.time - before.time);
  const headingDelta = Math.atan2(
    Math.sin(after.heading - before.heading),
    Math.cos(after.heading - before.heading),
  );
  return {
    x: before.x + (after.x - before.x) * ratio,
    y: before.y + (after.y - before.y) * ratio,
    heading: before.heading + headingDelta * ratio,
  };
}

function handleRos(topic, message) {
  let mapChanged = false;
  if (topic === "/map") {
    state.map = message;
    cacheMap(message);
    $("map-status").textContent = `${message.info.width} × ${message.info.height}`;
    $("map-status").className = "status-pill live";
    markHealth("planning", "Map live");
    mapChanged = true;
  } else if (topic === "/scan") {
    state.scan = message;
    $("scan-count").textContent = message.ranges.length.toLocaleString();
    markHealth("sensors");
    mapChanged = true;
  } else if (topic === "/plan") {
    state.path = message;
    markHealth("planning", "Path live");
    mapChanged = true;
  } else if (topic === "/odom") {
    state.lastChassisMessage = Date.now();
    state.odom = message;
    rememberOdom(message);
    markHealth("chassis");
    mapChanged = true;
  } else if (topic === "/amcl_pose") {
    setMapPose(message.pose.pose, message.header && message.header.stamp);
    markHealth("planning", "Localized");
    mapChanged = true;
  } else if (topic === "/control_center/main_state") {
    renderControlMode(message.data);
  } else if (topic === "/ackermann_cmd") {
    $("speed").textContent = `${message.drive.speed.toFixed(2)} m/s`;
    $("steering").textContent = `${(message.drive.steering_angle * 57.296).toFixed(1)}°`;
    markHealth("chassis");
  } else if (topic === "/battery_state") {
    renderBattery(message);
  } else if (topic === "/sensors/core") {
    renderBattery({
      voltage: message.state && message.state.voltage_input,
      percentage: null,
    });
  } else if (topic === "/yolo/detections_2d") {
    const trafficLights = (message.traffic_lights || []).map(item => ({
      ...(item.detection || {}),
      class_name: item.traffic_light_color
        ? `traffic light ${item.traffic_light_color}`
        : "traffic light",
    }));
    const detections = [...(message.detections || []), ...trafficLights];
    state.perceptionOverlay = {
      imageWidth: Number(message.image_width),
      imageHeight: Number(message.image_height),
      detections,
    };
    const count = detections.length;
    markHealth("perception", `${count} objects`);
    $("camera-live-tag").textContent = "AI OVERLAY";
    $("camera-status").textContent = "Detections · 10 Hz";
    drawPerceptionOverlay();
  } else if (topic.endsWith("/compressed")) {
    queueCameraFrame(message);
    $("camera-live-tag").textContent = state.perceptionOverlay
      ? "AI OVERLAY"
      : "LIVE";
    $("camera-status").className = "status-pill live";
    markHealth("sensors");
  }
  if (mapChanged) scheduleDraw();
}

/** Send one JSON operation through the active ROS WebSocket. */
function socketSend(message) {
  if (state.socket && state.socket.readyState === WebSocket.OPEN) {
    state.socket.send(JSON.stringify(message));
  }
}

/** Coalesce visualization updates into the next animation frame. */
function scheduleDraw() {
  if (state.drawPending) return;
  state.drawPending = true;
  const delay = Math.max(0, 33 - (performance.now() - state.lastDrawTime));
  setTimeout(() => {
    requestAnimationFrame(() => {
      state.drawPending = false;
      state.lastDrawTime = performance.now();
      draw();
    });
  }, delay);
}

function startViewDrag(event) {
  if (state.mapTool || event.button !== 0) return;
  state.viewDrag = {
    x: event.clientX,
    y: event.clientY,
    pan: event.shiftKey,
  };
  state.viewMoved = false;
  event.currentTarget.setPointerCapture(event.pointerId);
  event.currentTarget.classList.add("view-drag");
}

function moveViewDrag(event) {
  if (!state.viewDrag) return;
  const deltaX = event.clientX - state.viewDrag.x;
  const deltaY = event.clientY - state.viewDrag.y;
  if (Math.abs(deltaX) + Math.abs(deltaY) > 2) state.viewMoved = true;
  if (state.viewDrag.pan) {
    const rect = event.currentTarget.getBoundingClientRect();
    state.view.panX += deltaX * event.currentTarget.width / rect.width;
    state.view.panY += deltaY * event.currentTarget.height / rect.height;
  } else {
    state.view.rotation += deltaX * 0.006;
  }
  state.viewDrag.x = event.clientX;
  state.viewDrag.y = event.clientY;
  scheduleDraw();
}

function stopViewDrag(event) {
  if (!state.viewDrag) return;
  state.viewDrag = null;
  event.currentTarget.classList.remove("view-drag");
}

function zoomView(event) {
  event.preventDefault();
  state.view.zoom = Math.min(
    5,
    Math.max(0.35, state.view.zoom * Math.exp(-event.deltaY * 0.001)),
  );
  updateMapCoordinate(event.clientX, event.clientY);
  scheduleDraw();
}

/** Reset view. */
function resetView() {
  state.view = {zoom: 1, rotation: 0, panX: 0, panY: 0};
  scheduleDraw();
  setActivity("Visualization view reset", "success");
}

/** Request the bridge's retained occupancy map for immediate display. */
function republishMap() {
  if (!state.socket || state.socket.readyState !== WebSocket.OPEN) {
    setActivity("ROS WebSocket is not connected", "error");
    return;
  }
  socketSend({
    op: "subscribe",
    id: "/map",
    topic: "/map",
    type: "nav_msgs/msg/OccupancyGrid",
    throttle_rate: 500,
    queue_length: 1,
  });
  $("map-status").textContent = "Refreshing map";
  $("map-status").className = "status-pill neutral";
  setActivity("Requested the latest retained map", "success");
}

function yaw(quaternion) {
  return Math.atan2(
    2 * (quaternion.w * quaternion.z + quaternion.x * quaternion.y),
    1 - 2 * (quaternion.y * quaternion.y + quaternion.z * quaternion.z),
  );
}

function gridSpacing(bounds) {
  const span = Math.max(
    bounds.maxX - bounds.minX,
    bounds.maxY - bounds.minY,
  );
  const options = [0.5, 1, 2, 5, 10, 20, 50];
  return options.find(value => span / value <= 80) || 100;
}

function drawMetricGrid(context, transform, bounds, spacing) {
  const startX = Math.ceil(bounds.minX / spacing);
  const endX = Math.floor(bounds.maxX / spacing);
  const startY = Math.ceil(bounds.minY / spacing);
  const endY = Math.floor(bounds.maxY / spacing);
  context.save();
  context.lineWidth = 1 / state.view.zoom;

  const drawLines = major => {
    context.beginPath();
    for (let index = startX; index <= endX; index += 1) {
      if ((Math.abs(index) % 2 === 0) !== major) continue;
      const x = index * spacing;
      const [x1, y1] = transform(x, bounds.minY);
      const [x2, y2] = transform(x, bounds.maxY);
      context.moveTo(x1, y1);
      context.lineTo(x2, y2);
    }
    for (let index = startY; index <= endY; index += 1) {
      if ((Math.abs(index) % 2 === 0) !== major) continue;
      const y = index * spacing;
      const [x1, y1] = transform(bounds.minX, y);
      const [x2, y2] = transform(bounds.maxX, y);
      context.moveTo(x1, y1);
      context.lineTo(x2, y2);
    }
    context.strokeStyle = major
      ? "rgba(64, 84, 104, .22)"
      : "rgba(79, 101, 122, .11)";
    context.stroke();
  };

  drawLines(false);
  drawLines(true);
  context.restore();
}

function drawVehicleFootprint(context, transform, position, heading) {
  const halfLength = 0.25;
  const halfWidth = 0.125;
  const cosine = Math.cos(heading);
  const sine = Math.sin(heading);
  const worldPoint = (forward, left) => transform(
    position.x + forward * cosine - left * sine,
    position.y + forward * sine + left * cosine,
  );
  const corners = [
    worldPoint(halfLength, halfWidth),
    worldPoint(halfLength, -halfWidth),
    worldPoint(-halfLength, -halfWidth),
    worldPoint(-halfLength, halfWidth),
  ];
  const center = transform(position.x, position.y);
  const nose = worldPoint(halfLength, 0);

  context.save();
  context.shadowColor = "rgba(16, 35, 63, .24)";
  context.shadowBlur = 7 / state.view.zoom;
  context.fillStyle = "rgba(244, 180, 26, .72)";
  context.strokeStyle = "#10233f";
  context.lineWidth = 2 / state.view.zoom;
  context.beginPath();
  corners.forEach(([x, y], index) => {
    index ? context.lineTo(x, y) : context.moveTo(x, y);
  });
  context.closePath();
  context.fill();
  context.stroke();
  context.shadowColor = "transparent";
  context.beginPath();
  context.moveTo(center[0], center[1]);
  context.lineTo(nose[0], nose[1]);
  context.strokeStyle = "#10233f";
  context.lineWidth = 3 / state.view.zoom;
  context.stroke();
  context.beginPath();
  context.arc(center[0], center[1], 3 / state.view.zoom, 0, Math.PI * 2);
  context.fillStyle = "#10233f";
  context.fill();
  context.restore();
}

function drawPoseDrag(context, transform) {
  if (!state.poseDrag) return;
  const start = transform(state.poseDrag.start.x, state.poseDrag.start.y);
  const end = transform(state.poseDrag.end.x, state.poseDrag.end.y);
  const heading = Math.atan2(
    state.poseDrag.end.y - state.poseDrag.start.y,
    state.poseDrag.end.x - state.poseDrag.start.x,
  );
  context.save();
  context.strokeStyle = state.poseDrag.tool === "set-initial" ? "#f4b41a" : "#0b5ca8";
  context.fillStyle = context.strokeStyle;
  context.lineWidth = 3 / state.view.zoom;
  context.setLineDash([8 / state.view.zoom, 5 / state.view.zoom]);
  context.beginPath();
  context.moveTo(start[0], start[1]);
  context.lineTo(end[0], end[1]);
  context.stroke();
  context.setLineDash([]);
  const screenHeading = Math.atan2(end[1] - start[1], end[0] - start[0]);
  const arrowSize = 10 / state.view.zoom;
  context.beginPath();
  context.moveTo(end[0], end[1]);
  context.lineTo(
    end[0] - arrowSize * Math.cos(screenHeading - Math.PI / 6),
    end[1] - arrowSize * Math.sin(screenHeading - Math.PI / 6),
  );
  context.lineTo(
    end[0] - arrowSize * Math.cos(screenHeading + Math.PI / 6),
    end[1] - arrowSize * Math.sin(screenHeading + Math.PI / 6),
  );
  context.closePath();
  context.fill();
  context.restore();
  drawVehicleFootprint(context, transform, state.poseDrag.start, heading);
}

function niceScaleDistance(rawDistance) {
  const magnitude = 10 ** Math.floor(Math.log10(rawDistance));
  const normalized = rawDistance / magnitude;
  const step = normalized <= 1 ? 1 : normalized <= 2 ? 2 : normalized <= 5 ? 5 : 10;
  return step * magnitude;
}

function drawScaleBar(context, width, height, pixelsPerMeter) {
  const meters = niceScaleDistance(105 / (pixelsPerMeter * state.view.zoom));
  const length = meters * pixelsPerMeter * state.view.zoom;
  const x = 20;
  const y = height - 25;
  context.save();
  context.fillStyle = "rgba(255, 255, 255, .88)";
  context.fillRect(x - 9, y - 22, length + 18, 34);
  context.strokeStyle = "#10233f";
  context.lineWidth = 2;
  context.beginPath();
  context.moveTo(x, y);
  context.lineTo(x + length, y);
  context.moveTo(x, y - 5);
  context.lineTo(x, y + 5);
  context.moveTo(x + length, y - 5);
  context.lineTo(x + length, y + 5);
  context.stroke();
  context.fillStyle = "#31455a";
  context.font = "700 12px system-ui, sans-serif";
  context.fillText(`${meters.toFixed(meters < 1 ? 2 : 1)} m`, x, y - 8);
  context.restore();
}

function resizeMapCanvas(canvas) {
  const rect = canvas.getBoundingClientRect();
  if (rect.width < 1 || rect.height < 1) return null;
  const pixelRatio = Math.min(window.devicePixelRatio || 1, 2);
  const backingWidth = Math.max(1, Math.round(rect.width * pixelRatio));
  const backingHeight = Math.max(1, Math.round(rect.height * pixelRatio));
  if (canvas.width !== backingWidth || canvas.height !== backingHeight) {
    canvas.width = backingWidth;
    canvas.height = backingHeight;
  }
  const context = canvas.getContext("2d");
  context.setTransform(pixelRatio, 0, 0, pixelRatio, 0, 0);
  return {context, width: rect.width, height: rect.height};
}

function draw() {
  const canvas = $("map-canvas");
  const metrics = resizeMapCanvas(canvas);
  if (!metrics) return;
  const {context, width, height} = metrics;
  context.fillStyle = "#e9eef3";
  context.fillRect(0, 0, width, height);
  context.save();
  context.translate(
    width / 2 + state.view.panX,
    height / 2 + state.view.panY,
  );
  context.rotate(state.view.rotation);
  context.scale(state.view.zoom, state.view.zoom);
  context.translate(-width / 2, -height / 2);

  let pixelsPerMeter = 45;
  let bounds = {
    minX: -width / (2 * pixelsPerMeter),
    maxX: width / (2 * pixelsPerMeter),
    minY: -height / (2 * pixelsPerMeter),
    maxY: height / (2 * pixelsPerMeter),
  };
  let transform = (x, y) => [
    width / 2 + x * pixelsPerMeter,
    height / 2 - y * pixelsPerMeter,
  ];
  if (state.map && state.mapCanvas) {
    const map = state.map.info;
    const scale = Math.min(width / map.width, height / map.height);
    const renderWidth = map.width * scale;
    const renderHeight = map.height * scale;
    const offsetX = (width - renderWidth) / 2;
    const offsetY = (height - renderHeight) / 2;
    pixelsPerMeter = scale / map.resolution;
    bounds = {
      minX: map.origin.position.x,
      maxX: map.origin.position.x + map.width * map.resolution,
      minY: map.origin.position.y,
      maxY: map.origin.position.y + map.height * map.resolution,
    };
    state.mapRender = {scale, offsetX, offsetY};
    context.save();
    context.imageSmoothingEnabled = false;
    context.translate(offsetX, offsetY + renderHeight);
    context.scale(1, -1);
    context.drawImage(state.mapCanvas, 0, 0, renderWidth, renderHeight);
    context.restore();
    transform = (x, y) => [
      offsetX + (x - map.origin.position.x) / map.resolution * scale,
      offsetY + renderHeight - (y - map.origin.position.y) / map.resolution * scale,
    ];
  }

  const spacing = gridSpacing(bounds);
  $("grid-size").textContent = `Grid ${spacing < 1 ? spacing.toFixed(2) : spacing.toFixed(1)} m`;
  drawMetricGrid(context, transform, bounds, spacing);
  drawPoseDrag(context, transform);

  if (
    state.path
    && state.path.poses
    && state.path.poses.length
  ) {
    context.strokeStyle = "#0b5ca8";
    context.lineWidth = 5;
    context.lineJoin = "round";
    context.beginPath();
    state.path.poses.forEach((entry, index) => {
      const [x, y] = transform(entry.pose.position.x, entry.pose.position.y);
      index ? context.lineTo(x, y) : context.moveTo(x, y);
    });
    context.stroke();
  }

  const latestOdomTime = state.odomHistory.length
    ? state.odomHistory[state.odomHistory.length - 1].time
    : null;
  const displayPose = latestOdomTime === null
    ? mapPoseAt(Number.NaN)
    : mapPoseAt(latestOdomTime);
  if (!displayPose) {
    context.restore();
    drawScaleBar(context, width, height, pixelsPerMeter);
    return;
  }
  const position = {x: displayPose.x, y: displayPose.y};
  const heading = displayPose.heading;
  if (state.scan) {
    const scanStart = stampSeconds(state.scan.header);
    context.fillStyle = "rgba(16, 151, 147, .78)";
    state.scan.ranges.forEach((range, index) => {
      if (!Number.isFinite(range) || range < state.scan.range_min || range > state.scan.range_max) return;
      const beamPose = scanStart === null
        ? null
        : mapPoseAt(scanStart + index * state.scan.time_increment);
      const beamHeading = beamPose ? beamPose.heading : heading;
      const beamX = beamPose ? beamPose.x : position.x;
      const beamY = beamPose ? beamPose.y : position.y;
      const angle = beamHeading + state.scan.angle_min + index * state.scan.angle_increment;
      const [x, y] = transform(beamX + range * Math.cos(angle), beamY + range * Math.sin(angle));
      context.fillRect(x - 1.5, y - 1.5, 3, 3);
    });
  }
  drawVehicleFootprint(context, transform, position, heading);
  context.restore();
  drawScaleBar(context, width, height, pixelsPerMeter);
}

function renderLogs(status) {
  if (status.log_cursor < state.logCursor) {
    state.logCursor = 0;
    state.logLines = [];
  }
  if (status.log_start > state.logCursor) {
    state.logLines.push("[web] Earlier runtime messages expired from the server buffer.");
  }
  if (status.logs && status.logs.length) {
    state.logLines.push(...status.logs);
    state.logLines = state.logLines.slice(-500);
    state.compileLines.push(...status.logs);
    state.compileLines = state.compileLines.slice(-500);
    $("logs").textContent = state.logLines.join("\n");
    $("logs").scrollTop = $("logs").scrollHeight;
    $("compile-logs").textContent = state.compileLines.join("\n");
    $("compile-logs").scrollTop = $("compile-logs").scrollHeight;
  }
  state.logCursor = status.log_cursor;
  const recent = state.logLines.slice(-80);
  const waitingForPose = recent.some(line => (
    /please set the initial pose/i.test(line)
    || /timed out waiting for transform from base_link to map/i.test(line)
  ));
  const actionable = recent.filter(line => !(
    /please set the initial pose/i.test(line)
    || /timed out waiting for transform from base_link to map/i.test(line)
  ));
  const hasError = actionable.some(line => (
    /\b(error|fatal|traceback|failed)\b/i.test(line)
    || /exited with code [1-9]/i.test(line)
  ));
  $("console-state").textContent = hasError
    ? "Review runtime messages"
    : waitingForPose
      ? "Initial pose required"
      : "No active errors";
  document.querySelector(".console-summary i").style.background = hasError
    ? "#c63f3f"
    : waitingForPose
      ? "#f4b41a"
      : "#16845b";
}

async function refresh() {
  try {
    const status = await api(`/api/status?log_after=${state.logCursor}`);
    $("install").disabled = status.job_running;
    $("start").disabled = status.job_running;
    $("stop").disabled = !status.running;
    $("compile-start").disabled = status.job_running;
    const compiling = Boolean(
      status.job_running && status.job && status.job.startsWith("compile:")
    );
    const compileFinished = Boolean(
      !status.job_running && status.job && status.job.startsWith("compile:")
    );
    const compileFailed = compileFinished && status.job_return_code !== 0;
    $("compile-state").textContent = compiling
      ? "Compiling"
      : compileFinished
        ? compileFailed ? "Build failed" : "Build complete"
        : "Ready";
    $("compile-state").className = `status-pill ${compileFailed ? "error" : compiling || compileFinished ? "live" : "neutral"}`;
    document.querySelector(".session-banner").classList.toggle("stopped", !status.running);
    $("runtime-state").textContent = status.job_running
      ? compiling ? "Compiling CARKit" : "Installing CARKit"
      : status.running
        ? "CARKit running"
        : status.installed
          ? "Ready to launch"
          : "Installation required";
    setActivity(
      status.job_running
        ? compiling
          ? `Compiling ${status.job.split(":")[1]} packages…`
          : "Installing and building selected chassis…"
        : status.running
          ? `Session running · PID ${status.pid}`
          : status.installed
            ? "Installed and ready"
            : "Install a chassis to continue",
      status.running || status.installed ? "success" : "",
    );
    if (status.launch_config) applyLaunchConfig(status.launch_config);
    updateActiveMapFile(
      status.launch_config && status.launch_config.map
        ? status.launch_config.map
        : $("map").value,
    );
    const activeProfile = (status.launch_config && status.launch_config.profile) || $("profile").value;
    const activeChassis = (status.launch_config && status.launch_config.chassis)
      || (status.selection && status.selection.chassis)
      || $("chassis").value;
    updateSetupLabels(activeProfile, activeChassis);
    renderSystemTelemetry(status.system || {});
    if (status.chassis_telemetry && status.chassis_telemetry.fresh) {
      renderBattery(status.chassis_telemetry);
    }
    renderChassisStatus(status.running);
    renderLogs(status);
    if (status.running) connectRos();
    if (!status.installed && !state.formHydrated) {
      state.formHydrated = true;
      openDrawer();
    }
  } catch (error) {
    setActivity(error.message, "error");
    $("runtime-state").textContent = "Web service unavailable";
  }
  setTimeout(refresh, 1500);
}

$("profile").addEventListener("change", updateProfileHelp);
$("perception-model").addEventListener("change", updatePerceptionModelHelp);
window.addEventListener("resize", drawPerceptionOverlay);
$("mode-human").addEventListener("click", () => requestControlMode("HUMAN_CONTROL"));
$("mode-autonomous").addEventListener("click", () => requestControlMode("AUTO_DRIVE"));
$("chassis").addEventListener("change", () => updateSetupLabels($("profile").value, $("chassis").value));
$("map").addEventListener("change", () => updateMapSelectionDisplay());
$("open-config").addEventListener("click", openDrawer);
$("close-config").addEventListener("click", closeDrawer);
$("drawer-scrim").addEventListener("click", closeDrawer);
document.addEventListener("keydown", event => { if (event.key === "Escape") closeDrawer(); });

$("install").onclick = async () => {
  try {
    await api("/api/install", {chassis: $("chassis").value});
    setActivity("Installation started", "success");
  } catch (error) { setActivity(error.message, "error"); }
};

$("start").onclick = async () => {
  try {
    const request = requestBody();
    state.odomHistory = [];
    await api("/api/launch", request);
    state.formHydrated = true;
    updateSetupLabels(request.profile, request.chassis);
    setActivity("Starting CARKit…", "success");
    setTimeout(closeDrawer, 300);
    setTimeout(connectRos, 1200);
  } catch (error) { setActivity(error.message, "error"); }
};

$("stop").onclick = async () => {
  try {
    await api("/api/stop", {});
    setActivity("CARKit stopped", "success");
  } catch (error) { setActivity(error.message, "error"); }
};

$("clear-log").onclick = async () => {
  state.logLines = [];
  $("logs").textContent = "Console view cleared.";
};

$("clear-compile-log").onclick = () => {
  state.compileLines = [];
  $("compile-logs").textContent = "Compiler view cleared.";
};

$("editor-profile").addEventListener("change", () => loadEditorWorkspace());
$("editor-component").addEventListener("change", () => loadEditorWorkspace());
$("editor-reload").addEventListener("click", () => loadEditorFile(true));
$("editor-save").addEventListener("click", saveEditorFile);
$("code-editor").addEventListener("input", () => {
  state.editorDirty = true;
  $("editor-save").disabled = false;
  setEditorStatus("Syncing");
  updateEditorLines();
  renderEditorHighlight();
  clearTimeout(state.editorSyncTimer);
  state.editorSyncTimer = setTimeout(synchronizeEditor, 120);
});
$("code-editor").addEventListener("scroll", syncEditorScroll);
$("code-editor").addEventListener("click", renderRemoteCursors);
$("code-editor").addEventListener("keyup", renderRemoteCursors);
$("editor-user-name").addEventListener("change", event => {
  state.editorName = event.target.value.trim().slice(0, 32) || state.editorName;
  event.target.value = state.editorName;
  try { localStorage.setItem("carkit-editor-name", state.editorName); } catch { /* Optional. */ }
  if (state.editorLoaded) pollCollaborativeEditor();
});
$("code-editor").addEventListener("keydown", event => {
  if ((event.ctrlKey || event.metaKey) && event.key.toLowerCase() === "s") {
    event.preventDefault();
    saveEditorFile();
    return;
  }
  if (event.key !== "Tab") return;
  event.preventDefault();
  const editor = event.currentTarget;
  editor.setRangeText("    ", editor.selectionStart, editor.selectionEnd, "end");
  editor.dispatchEvent(new Event("input"));
});

function updateCompileSelection() {
  const selected = document.querySelector(
    'input[name="compile-target"]:checked'
  );
  if (!selected) return;
  const targetLabel = selected
    .closest("label")
    .querySelector("strong")
    .textContent;
  const implementation = ["planning", "control", "perception"]
    .includes(selected.value)
    ? $(selected.value).value
    : null;
  $("compile-selection").textContent = implementation
    ? `${targetLabel} · ${implementationLabel(implementation)}`
    : targetLabel;
}

document.querySelectorAll('input[name="compile-target"]').forEach(input => {
  input.addEventListener("change", updateCompileSelection);
});
for (const component of ["planning", "control", "perception"]) {
  $(component).addEventListener("change", updateCompileSelection);
}

$("compile-start").onclick = async () => {
  const selected = document.querySelector(
    'input[name="compile-target"]:checked'
  );
  try {
    state.compileLines = [];
    $("compile-logs").textContent = "Starting compilation…";
    const setup = requestBody();
    const result = await api("/api/compile", {
      target: selected.value,
      implementations: setup.implementations,
    });
    const packages = result.packages && result.packages.length
      ? result.packages.join(", ")
      : "entire repository";
    state.compileLines = [`[compile] Selected packages: ${packages}`];
    $("compile-logs").textContent = state.compileLines[0];
    setActivity(`Compiling ${selected.value}: ${packages}`, "success");
  } catch (error) {
    setActivity(error.message, "error");
  }
};

$("set-initial").onclick = () => armMapTool("set-initial");
$("send-goal").onclick = () => armMapTool("send-goal");
$("reset-view").onclick = resetView;
$("republish-map").onclick = republishMap;
const mapCanvas = $("map-canvas");
mapCanvas.addEventListener("pointerdown", event => {
  if (!startPoseDrag(event)) startViewDrag(event);
});
mapCanvas.addEventListener("pointermove", event => {
  if (!movePoseDrag(event)) moveViewDrag(event);
});
mapCanvas.addEventListener("pointerup", event => {
  if (!stopPoseDrag(event)) stopViewDrag(event);
});
mapCanvas.addEventListener("pointercancel", event => {
  if (!stopPoseDrag(event, true)) stopViewDrag(event);
});
mapCanvas.addEventListener("wheel", zoomView, {passive: false});
mapCanvas.addEventListener("dblclick", resetView);
mapCanvas.addEventListener("mousemove", event => {
  updateMapCoordinate(event.clientX, event.clientY);
});
if (window.ResizeObserver) {
  new ResizeObserver(scheduleDraw).observe(mapCanvas);
} else {
  window.addEventListener("resize", scheduleDraw);
}

const pageConfiguration = {
  "#overview": ["overview", "Vehicle workspace", "Overview"],
  "#code": ["code-page", "Code editor", "Development / Code editor"],
  "#compile": ["compile-page", "Compile workspace", "Development / Compile"],
};

function showPage(hash) {
  const selectedHash = pageConfiguration[hash] ? hash : "#overview";
  const [pageId, title, breadcrumb] = pageConfiguration[selectedHash];
  document.querySelectorAll("main.page-view").forEach(page => {
    page.hidden = page.id !== pageId;
  });
  document.querySelectorAll(".side-nav a").forEach(link => {
    link.classList.toggle("active", link.hash === selectedHash);
    link.toggleAttribute("aria-current", link.hash === selectedHash);
  });
  document.querySelector(".page-title h1").textContent = title;
  document.querySelector(".breadcrumb").textContent = breadcrumb;
  if (selectedHash === "#code" && !state.editorLoaded) loadEditorWorkspace(true);
  scheduleDraw();
}

document.querySelectorAll(".side-nav a").forEach(link => {
  link.addEventListener("click", () => showPage(link.hash));
});
window.addEventListener("hashchange", () => showPage(location.hash));
window.addEventListener("beforeunload", () => {
  if (!state.editorLoaded) return;
  navigator.sendBeacon("/api/editor/leave", new Blob([JSON.stringify({
    profile: state.editorProfile,
    component: state.editorComponent,
    client_id: state.editorClientId,
    file: state.editorFile,
  })], {type: "application/json"}));
});

configure()
  .then(() => { showPage(location.hash); refresh(); connectRos(); })
  .catch(error => setActivity(error.message, "error"));
