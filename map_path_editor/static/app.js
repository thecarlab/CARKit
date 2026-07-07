const statusPill = document.getElementById("status-pill");
const mapSelect = document.getElementById("map-select");
const loadMapButton = document.getElementById("load-map");
const refreshMapsButton = document.getElementById("refresh-maps");
const mapErrors = document.getElementById("map-errors");
const localYamlInput = document.getElementById("local-yaml");
const localPgmInput = document.getElementById("local-pgm");
const loadLocalButton = document.getElementById("load-local");
const fitMapButton = document.getElementById("fit-map");
const canvas = document.getElementById("map-canvas");
const ctx = canvas.getContext("2d");
const mapTitle = document.getElementById("map-title");
const mapMeta = document.getElementById("map-meta");
const cursorReadout = document.getElementById("cursor-readout");
const shapeType = document.getElementById("shape-type");
const centerXInput = document.getElementById("center-x");
const centerYInput = document.getElementById("center-y");
const rotationInput = document.getElementById("rotation");
const rotationValue = document.getElementById("rotation-value");
const lineLengthInput = document.getElementById("line-length");
const radiusInput = document.getElementById("radius");
const trackStraightInput = document.getElementById("track-straight");
const samplesInput = document.getElementById("samples");
const lineLengthRow = document.getElementById("line-length-row");
const radiusRow = document.getElementById("radius-row");
const trackStraightRow = document.getElementById("track-straight-row");
const zoomInput = document.getElementById("zoom");
const zoomValue = document.getElementById("zoom-value");
const exportOutput = document.getElementById("export-output");
const copyJsonButton = document.getElementById("copy-json");
const downloadJsonButton = document.getElementById("download-json");

const state = {
  map: null,
  shape: {
    type: "line",
    centerX: 0,
    centerY: 0,
    rotation: 0,
    lineLength: 2,
    radius: 1,
    trackStraight: 3,
    samples: 120,
  },
  view: {
    width: 1,
    height: 1,
    fitScale: 1,
    scale: 1,
    zoom: 1,
    offsetX: 0,
    offsetY: 0,
  },
  drag: null,
};

function setStatus(kind, label) {
  statusPill.className = `pill pill-${kind}`;
  statusPill.textContent = label;
}

function showErrors(errors) {
  if (!errors.length) {
    mapErrors.classList.add("hidden");
    mapErrors.innerHTML = "";
    return;
  }
  mapErrors.classList.remove("hidden");
  mapErrors.innerHTML = errors.map((error) => `<div>${escapeHtml(error)}</div>`).join("");
}

function escapeHtml(value) {
  return String(value).replace(/[&<>"']/g, (char) => {
    const replacements = { "&": "&amp;", "<": "&lt;", ">": "&gt;", '"': "&quot;", "'": "&#39;" };
    return replacements[char];
  });
}

function clamp(value, min, max) {
  return Math.max(min, Math.min(max, value));
}

function round(value, digits = 4) {
  return Number(value.toFixed(digits));
}

function parseYaml(text) {
  const config = {};
  for (const line of text.split(/\r?\n/)) {
    const trimmed = line.trim();
    if (!trimmed || trimmed.startsWith("#") || !trimmed.includes(":")) {
      continue;
    }
    const [key, ...rest] = trimmed.split(":");
    const rawValue = rest.join(":").split("#", 1)[0].trim();
    config[key.trim()] = parseYamlValue(rawValue);
  }
  if (!config.image || config.resolution === undefined || !Array.isArray(config.origin)) {
    throw new Error("YAML must define image, resolution, and origin");
  }
  return config;
}

function parseYamlValue(rawValue) {
  if (!rawValue) {
    return "";
  }
  if (rawValue.startsWith("[") && rawValue.endsWith("]")) {
    return rawValue
      .slice(1, -1)
      .split(",")
      .map((item) => parseYamlValue(item.trim()));
  }
  if (
    (rawValue.startsWith('"') && rawValue.endsWith('"')) ||
    (rawValue.startsWith("'") && rawValue.endsWith("'"))
  ) {
    return rawValue.slice(1, -1);
  }
  const numberValue = Number(rawValue);
  return Number.isNaN(numberValue) ? rawValue : numberValue;
}

function parsePgm(arrayBuffer) {
  const bytes = new Uint8Array(arrayBuffer);
  let index = 0;

  function isWhitespace(byte) {
    return byte === 9 || byte === 10 || byte === 13 || byte === 32;
  }

  function skipWhitespaceAndComments() {
    while (index < bytes.length) {
      if (bytes[index] === 35) {
        while (index < bytes.length && bytes[index] !== 10 && bytes[index] !== 13) {
          index += 1;
        }
        continue;
      }
      if (isWhitespace(bytes[index])) {
        index += 1;
        continue;
      }
      break;
    }
  }

  function nextToken() {
    skipWhitespaceAndComments();
    const start = index;
    while (index < bytes.length && !isWhitespace(bytes[index])) {
      index += 1;
    }
    if (start === index) {
      throw new Error("Unexpected end of PGM file");
    }
    return new TextDecoder("ascii").decode(bytes.slice(start, index));
  }

  const magic = nextToken();
  const width = Number(nextToken());
  const height = Number(nextToken());
  const maxValue = Number(nextToken());
  if (!["P2", "P5"].includes(magic) || !width || !height || !maxValue) {
    throw new Error("Invalid PGM header");
  }

  const pixelCount = width * height;
  const pixels = new Uint8ClampedArray(pixelCount);

  if (magic === "P5") {
    if (bytes[index] === 13 && bytes[index + 1] === 10) {
      index += 2;
    } else if (isWhitespace(bytes[index])) {
      index += 1;
    }
    if (maxValue < 256) {
      if (bytes.length - index < pixelCount) {
        throw new Error("PGM image data is shorter than expected");
      }
      for (let i = 0; i < pixelCount; i += 1) {
        pixels[i] = Math.round((bytes[index + i] / maxValue) * 255);
      }
    } else {
      if (bytes.length - index < pixelCount * 2) {
        throw new Error("16-bit PGM image data is shorter than expected");
      }
      for (let i = 0; i < pixelCount; i += 1) {
        const value = bytes[index + i * 2] * 256 + bytes[index + i * 2 + 1];
        pixels[i] = Math.round((value / maxValue) * 255);
      }
    }
  } else {
    for (let i = 0; i < pixelCount; i += 1) {
      pixels[i] = Math.round((Number(nextToken()) / maxValue) * 255);
    }
  }

  return { magic, width, height, maxValue, pixels };
}

async function refreshServerMaps() {
  setStatus("idle", "Loading maps");
  try {
    const response = await fetch("/api/maps", { cache: "no-store" });
    if (!response.ok) {
      throw new Error(`HTTP ${response.status}`);
    }
    const payload = await response.json();
    mapSelect.innerHTML = "";
    for (const map of payload.maps || []) {
      const option = document.createElement("option");
      option.value = map.id;
      option.textContent = `${map.yamlFile} (${map.width} x ${map.height})`;
      mapSelect.appendChild(option);
    }
    showErrors(payload.errors || []);
    if (!mapSelect.options.length) {
      setStatus("error", "No maps found");
      return;
    }
    setStatus("live", "Maps ready");
    await loadServerMap(mapSelect.value);
  } catch (error) {
    setStatus("error", "Map API offline");
    showErrors([`Failed to load repository maps: ${error.message}`]);
  }
}

async function loadServerMap(mapId) {
  if (!mapId) {
    return;
  }
  setStatus("idle", "Loading map");
  const metadataResponse = await fetch(`/api/maps/${encodeURIComponent(mapId)}`, { cache: "no-store" });
  if (!metadataResponse.ok) {
    throw new Error(`Failed to load map metadata: HTTP ${metadataResponse.status}`);
  }
  const metadata = await metadataResponse.json();
  const imageResponse = await fetch(metadata.imageUrl, { cache: "no-store" });
  if (!imageResponse.ok) {
    throw new Error(`Failed to load PGM image: HTTP ${imageResponse.status}`);
  }
  await loadMapData(metadata, await imageResponse.arrayBuffer(), "repository");
}

async function loadLocalMap() {
  const yamlFile = localYamlInput.files[0];
  const pgmFile = localPgmInput.files[0];
  if (!yamlFile || !pgmFile) {
    showErrors(["Choose both a YAML file and a PGM file."]);
    return;
  }
  try {
    const config = parseYaml(await yamlFile.text());
    const metadata = {
      id: yamlFile.name,
      yamlFile: yamlFile.name,
      imageFile: pgmFile.name,
      resolution: Number(config.resolution),
      origin: [Number(config.origin[0]), Number(config.origin[1]), Number(config.origin[2] || 0)],
      mode: config.mode || "trinary",
      negate: Number(config.negate || 0),
      occupiedThresh: Number(config.occupied_thresh || 0.65),
      freeThresh: Number(config.free_thresh || 0.25),
    };
    await loadMapData(metadata, await pgmFile.arrayBuffer(), "local files");
    showErrors([]);
  } catch (error) {
    setStatus("error", "Local load failed");
    showErrors([error.message]);
  }
}

async function loadMapData(metadata, pgmBuffer, sourceLabel) {
  const pgm = parsePgm(pgmBuffer);
  const mapCanvas = document.createElement("canvas");
  mapCanvas.width = pgm.width;
  mapCanvas.height = pgm.height;
  const mapCtx = mapCanvas.getContext("2d");
  const imageData = mapCtx.createImageData(pgm.width, pgm.height);
  for (let i = 0; i < pgm.pixels.length; i += 1) {
    const value = metadata.negate ? 255 - pgm.pixels[i] : pgm.pixels[i];
    imageData.data[i * 4] = value;
    imageData.data[i * 4 + 1] = value;
    imageData.data[i * 4 + 2] = value;
    imageData.data[i * 4 + 3] = 255;
  }
  mapCtx.putImageData(imageData, 0, 0);

  state.map = {
    ...metadata,
    format: pgm.magic,
    width: pgm.width,
    height: pgm.height,
    maxValue: pgm.maxValue,
    canvas: mapCanvas,
    sourceLabel,
  };

  state.shape.centerX = metadata.origin[0] + (pgm.width * metadata.resolution) / 2;
  state.shape.centerY = metadata.origin[1] + (pgm.height * metadata.resolution) / 2;
  updateControlsFromState();
  fitMapToCanvas();
  setStatus("live", "Map loaded");
}

function resizeCanvas() {
  const rect = canvas.parentElement.getBoundingClientRect();
  const dpr = window.devicePixelRatio || 1;
  state.view.width = Math.max(1, rect.width);
  state.view.height = Math.max(1, rect.height);
  canvas.width = Math.round(state.view.width * dpr);
  canvas.height = Math.round(state.view.height * dpr);
  ctx.setTransform(dpr, 0, 0, dpr, 0, 0);
  render();
}

function fitMapToCanvas() {
  resizeCanvas();
  if (!state.map) {
    return;
  }
  const padding = 28;
  state.view.fitScale = Math.min(
    (state.view.width - padding * 2) / state.map.width,
    (state.view.height - padding * 2) / state.map.height
  );
  state.view.fitScale = Math.max(0.001, state.view.fitScale);
  state.view.zoom = 1;
  state.view.scale = state.view.fitScale;
  state.view.offsetX = (state.view.width - state.map.width * state.view.scale) / 2;
  state.view.offsetY = (state.view.height - state.map.height * state.view.scale) / 2;
  zoomInput.value = "100";
  zoomValue.textContent = "100%";
  render();
}

function applyZoom(nextZoom, anchor) {
  if (!state.map) {
    return;
  }
  const before = screenToMapPixel(anchor.x, anchor.y);
  state.view.zoom = clamp(nextZoom, 0.5, 4);
  state.view.scale = state.view.fitScale * state.view.zoom;
  state.view.offsetX = anchor.x - before.x * state.view.scale;
  state.view.offsetY = anchor.y - before.y * state.view.scale;
  zoomInput.value = String(Math.round(state.view.zoom * 100));
  zoomValue.textContent = `${Math.round(state.view.zoom * 100)}%`;
  render();
}

function screenToMapPixel(x, y) {
  return {
    x: (x - state.view.offsetX) / state.view.scale,
    y: (y - state.view.offsetY) / state.view.scale,
  };
}

function worldToMapPixel(point) {
  return {
    x: (point.x - state.map.origin[0]) / state.map.resolution,
    y: state.map.height - (point.y - state.map.origin[1]) / state.map.resolution,
  };
}

function mapPixelToWorld(point) {
  return {
    x: state.map.origin[0] + point.x * state.map.resolution,
    y: state.map.origin[1] + (state.map.height - point.y) * state.map.resolution,
  };
}

function worldToScreen(point) {
  const pixel = worldToMapPixel(point);
  return {
    x: state.view.offsetX + pixel.x * state.view.scale,
    y: state.view.offsetY + pixel.y * state.view.scale,
  };
}

function screenToWorld(x, y) {
  return mapPixelToWorld(screenToMapPixel(x, y));
}

function rotatePoint(point, radians) {
  const cos = Math.cos(radians);
  const sin = Math.sin(radians);
  return {
    x: point.x * cos - point.y * sin,
    y: point.x * sin + point.y * cos,
  };
}

function toWorld(localPoint) {
  const rotated = rotatePoint(localPoint, state.shape.rotation);
  return {
    x: state.shape.centerX + rotated.x,
    y: state.shape.centerY + rotated.y,
  };
}

function generatePathPoints() {
  const samples = clamp(Math.round(state.shape.samples), 8, 720);
  const points = [];

  if (state.shape.type === "line") {
    const length = Math.max(0.05, state.shape.lineLength);
    for (let i = 0; i < samples; i += 1) {
      const t = samples === 1 ? 0 : i / (samples - 1);
      points.push(toWorld({ x: -length / 2 + t * length, y: 0 }));
    }
    return { points, closed: false };
  }

  if (state.shape.type === "circle") {
    const radius = Math.max(0.05, state.shape.radius);
    for (let i = 0; i <= samples; i += 1) {
      const theta = (i / samples) * Math.PI * 2;
      points.push(toWorld({ x: Math.cos(theta) * radius, y: Math.sin(theta) * radius }));
    }
    return { points, closed: true };
  }

  if (state.shape.type === "halfCircle") {
    const radius = Math.max(0.05, state.shape.radius);
    for (let i = 0; i <= samples; i += 1) {
      const theta = Math.PI - (i / samples) * Math.PI;
      points.push(toWorld({ x: Math.cos(theta) * radius, y: Math.sin(theta) * radius }));
    }
    return { points, closed: false };
  }

  const radius = Math.max(0.05, state.shape.radius);
  const straight = Math.max(0.05, state.shape.trackStraight);
  const lineSamples = Math.max(3, Math.floor(samples * straight / (straight * 2 + Math.PI * radius * 2)));
  const arcSamples = Math.max(8, Math.floor((samples - lineSamples * 2) / 2));

  for (let i = 0; i <= lineSamples; i += 1) {
    const t = i / lineSamples;
    points.push(toWorld({ x: -straight / 2 + t * straight, y: radius }));
  }
  for (let i = 1; i <= arcSamples; i += 1) {
    const theta = Math.PI / 2 - (i / arcSamples) * Math.PI;
    points.push(toWorld({ x: straight / 2 + Math.cos(theta) * radius, y: Math.sin(theta) * radius }));
  }
  for (let i = 1; i <= lineSamples; i += 1) {
    const t = i / lineSamples;
    points.push(toWorld({ x: straight / 2 - t * straight, y: -radius }));
  }
  for (let i = 1; i <= arcSamples; i += 1) {
    const theta = -Math.PI / 2 - (i / arcSamples) * Math.PI;
    points.push(toWorld({ x: -straight / 2 + Math.cos(theta) * radius, y: Math.sin(theta) * radius }));
  }
  points.push(points[0]);
  return { points, closed: true };
}

function render() {
  ctx.clearRect(0, 0, state.view.width, state.view.height);
  ctx.fillStyle = "#0a0f14";
  ctx.fillRect(0, 0, state.view.width, state.view.height);

  if (!state.map) {
    drawEmptyState();
    updateExport();
    return;
  }

  ctx.imageSmoothingEnabled = false;
  ctx.drawImage(
    state.map.canvas,
    state.view.offsetX,
    state.view.offsetY,
    state.map.width * state.view.scale,
    state.map.height * state.view.scale
  );

  ctx.strokeStyle = "rgba(76, 201, 240, 0.65)";
  ctx.lineWidth = 1;
  ctx.strokeRect(
    state.view.offsetX,
    state.view.offsetY,
    state.map.width * state.view.scale,
    state.map.height * state.view.scale
  );

  drawPath();
  updateMapLabels();
  updateExport();
}

function drawEmptyState() {
  ctx.fillStyle = "#93a1b3";
  ctx.textAlign = "center";
  ctx.font = "16px Segoe UI, sans-serif";
  ctx.fillText("Load a YAML/PGM occupancy map to begin.", state.view.width / 2, state.view.height / 2);
}

function drawPath() {
  const { points } = generatePathPoints();
  if (!points.length) {
    return;
  }

  ctx.save();
  ctx.lineWidth = 3;
  ctx.lineJoin = "round";
  ctx.lineCap = "round";
  ctx.strokeStyle = "#ffb703";
  ctx.shadowBlur = 8;
  ctx.shadowColor = "rgba(255, 183, 3, 0.45)";
  ctx.beginPath();
  points.forEach((point, index) => {
    const screen = worldToScreen(point);
    if (index === 0) {
      ctx.moveTo(screen.x, screen.y);
    } else {
      ctx.lineTo(screen.x, screen.y);
    }
  });
  ctx.stroke();
  ctx.restore();

  const center = worldToScreen({ x: state.shape.centerX, y: state.shape.centerY });
  ctx.fillStyle = "#80ed99";
  ctx.strokeStyle = "#071015";
  ctx.lineWidth = 2;
  ctx.beginPath();
  ctx.arc(center.x, center.y, 6, 0, Math.PI * 2);
  ctx.fill();
  ctx.stroke();

  drawHeadingMarker(center);
}

function drawHeadingMarker(center) {
  const length = 34;
  const end = {
    x: center.x + Math.cos(state.shape.rotation) * length,
    y: center.y - Math.sin(state.shape.rotation) * length,
  };
  ctx.strokeStyle = "#80ed99";
  ctx.lineWidth = 2;
  ctx.beginPath();
  ctx.moveTo(center.x, center.y);
  ctx.lineTo(end.x, end.y);
  ctx.stroke();
}

function updateMapLabels() {
  if (!state.map) {
    mapTitle.textContent = "No map loaded";
    mapMeta.textContent = "Choose a repository map or load a local YAML/PGM pair.";
    return;
  }
  mapTitle.textContent = state.map.yamlFile;
  const widthM = state.map.width * state.map.resolution;
  const heightM = state.map.height * state.map.resolution;
  mapMeta.textContent = `${state.map.sourceLabel} - ${state.map.width} x ${state.map.height} px, ${round(widthM, 2)} x ${round(heightM, 2)} m, resolution ${state.map.resolution} m/px`;
}

function updateControlsFromState() {
  shapeType.value = state.shape.type;
  centerXInput.value = round(state.shape.centerX, 3);
  centerYInput.value = round(state.shape.centerY, 3);
  rotationInput.value = String(Math.round((state.shape.rotation * 180) / Math.PI));
  rotationValue.textContent = `${rotationInput.value} deg`;
  lineLengthInput.value = round(state.shape.lineLength, 3);
  radiusInput.value = round(state.shape.radius, 3);
  trackStraightInput.value = round(state.shape.trackStraight, 3);
  samplesInput.value = String(state.shape.samples);
  updateVisibleShapeControls();
}

function updateStateFromControls() {
  state.shape.type = shapeType.value;
  state.shape.centerX = Number(centerXInput.value) || 0;
  state.shape.centerY = Number(centerYInput.value) || 0;
  state.shape.rotation = ((Number(rotationInput.value) || 0) * Math.PI) / 180;
  state.shape.lineLength = Math.max(0.05, Number(lineLengthInput.value) || 0.05);
  state.shape.radius = Math.max(0.05, Number(radiusInput.value) || 0.05);
  state.shape.trackStraight = Math.max(0.05, Number(trackStraightInput.value) || 0.05);
  state.shape.samples = clamp(Number(samplesInput.value) || 120, 8, 720);
  rotationValue.textContent = `${rotationInput.value} deg`;
  updateVisibleShapeControls();
  render();
}

function updateVisibleShapeControls() {
  lineLengthRow.classList.toggle("hidden", state.shape.type !== "line");
  radiusRow.classList.toggle("hidden", state.shape.type === "line");
  trackStraightRow.classList.toggle("hidden", state.shape.type !== "track");
}

function pathDistanceToScreenPoint(x, y) {
  const { points } = generatePathPoints();
  let minDistance = Infinity;
  for (let i = 1; i < points.length; i += 1) {
    const a = worldToScreen(points[i - 1]);
    const b = worldToScreen(points[i]);
    minDistance = Math.min(minDistance, distanceToSegment(x, y, a, b));
  }
  const center = worldToScreen({ x: state.shape.centerX, y: state.shape.centerY });
  minDistance = Math.min(minDistance, Math.hypot(x - center.x, y - center.y));
  return minDistance;
}

function distanceToSegment(px, py, a, b) {
  const dx = b.x - a.x;
  const dy = b.y - a.y;
  const lengthSquared = dx * dx + dy * dy;
  if (!lengthSquared) {
    return Math.hypot(px - a.x, py - a.y);
  }
  const t = clamp(((px - a.x) * dx + (py - a.y) * dy) / lengthSquared, 0, 1);
  return Math.hypot(px - (a.x + t * dx), py - (a.y + t * dy));
}

function canvasPoint(event) {
  const rect = canvas.getBoundingClientRect();
  return { x: event.clientX - rect.left, y: event.clientY - rect.top };
}

function updateExport() {
  if (!state.map) {
    exportOutput.value = "";
    return;
  }
  const { points, closed } = generatePathPoints();
  const payload = {
    frame_id: "map",
    map: {
      id: state.map.id,
      yaml_file: state.map.yamlFile,
      image_file: state.map.imageFile,
      resolution: state.map.resolution,
      origin: state.map.origin,
    },
    shape: {
      type: state.shape.type,
      center: { x: round(state.shape.centerX), y: round(state.shape.centerY) },
      rotation_degrees: round((state.shape.rotation * 180) / Math.PI, 2),
      line_length: round(state.shape.lineLength),
      radius: round(state.shape.radius),
      track_straight_length: round(state.shape.trackStraight),
      samples: state.shape.samples,
    },
    path: {
      closed,
      points: points.map((point) => ({ x: round(point.x), y: round(point.y) })),
    },
  };
  exportOutput.value = JSON.stringify(payload, null, 2);
}

async function copyExport() {
  if (!exportOutput.value) {
    return;
  }
  await navigator.clipboard.writeText(exportOutput.value);
  setStatus("live", "JSON copied");
}

function downloadExport() {
  if (!exportOutput.value) {
    return;
  }
  const blob = new Blob([exportOutput.value], { type: "application/json" });
  const url = URL.createObjectURL(blob);
  const link = document.createElement("a");
  link.href = url;
  link.download = `${state.map.id.replace(/\.(yaml|yml)$/i, "")}_${state.shape.type}_path.json`;
  link.click();
  URL.revokeObjectURL(url);
}

canvas.addEventListener("mousedown", (event) => {
  if (!state.map) {
    return;
  }
  const point = canvasPoint(event);
  if (event.shiftKey || event.button === 1) {
    state.drag = { mode: "pan", startX: point.x, startY: point.y, offsetX: state.view.offsetX, offsetY: state.view.offsetY };
    return;
  }

  const world = screenToWorld(point.x, point.y);
  const distance = pathDistanceToScreenPoint(point.x, point.y);
  if (distance > 16) {
    state.shape.centerX = world.x;
    state.shape.centerY = world.y;
    updateControlsFromState();
    render();
  }
  state.drag = {
    mode: "path",
    offsetX: world.x - state.shape.centerX,
    offsetY: world.y - state.shape.centerY,
  };
});

canvas.addEventListener("mousemove", (event) => {
  const point = canvasPoint(event);
  if (state.map) {
    const world = screenToWorld(point.x, point.y);
    cursorReadout.textContent = `x: ${round(world.x, 3)}, y: ${round(world.y, 3)}`;
  }

  if (!state.drag) {
    return;
  }

  if (state.drag.mode === "pan") {
    state.view.offsetX = state.drag.offsetX + point.x - state.drag.startX;
    state.view.offsetY = state.drag.offsetY + point.y - state.drag.startY;
    render();
    return;
  }

  const world = screenToWorld(point.x, point.y);
  state.shape.centerX = world.x - state.drag.offsetX;
  state.shape.centerY = world.y - state.drag.offsetY;
  updateControlsFromState();
  render();
});

canvas.addEventListener("mouseup", () => {
  state.drag = null;
});

canvas.addEventListener("mouseleave", () => {
  state.drag = null;
  cursorReadout.textContent = "x: --, y: --";
});

canvas.addEventListener("contextmenu", (event) => {
  event.preventDefault();
});

canvas.addEventListener("wheel", (event) => {
  if (!state.map) {
    return;
  }
  event.preventDefault();
  const point = canvasPoint(event);
  const nextZoom = state.view.zoom * (event.deltaY < 0 ? 1.12 : 0.88);
  applyZoom(nextZoom, point);
});

for (const input of [
  shapeType,
  centerXInput,
  centerYInput,
  rotationInput,
  lineLengthInput,
  radiusInput,
  trackStraightInput,
  samplesInput,
]) {
  input.addEventListener("input", updateStateFromControls);
}

zoomInput.addEventListener("input", () => {
  applyZoom(Number(zoomInput.value) / 100, { x: state.view.width / 2, y: state.view.height / 2 });
});

loadMapButton.addEventListener("click", async () => {
  try {
    await loadServerMap(mapSelect.value);
    showErrors([]);
  } catch (error) {
    setStatus("error", "Load failed");
    showErrors([error.message]);
  }
});

refreshMapsButton.addEventListener("click", refreshServerMaps);
loadLocalButton.addEventListener("click", loadLocalMap);
fitMapButton.addEventListener("click", fitMapToCanvas);
copyJsonButton.addEventListener("click", copyExport);
downloadJsonButton.addEventListener("click", downloadExport);

window.addEventListener("resize", resizeCanvas);

resizeCanvas();
updateControlsFromState();
refreshServerMaps();
