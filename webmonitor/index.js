const ONLINE_WINDOW_MS = 90 * 60 * 1000;
const JSON_HEADERS = {
  "content-type": "application/json; charset=utf-8",
  "cache-control": "no-store",
  "x-content-type-options": "nosniff",
};

const PAGE = String.raw`<!doctype html>
<html lang="en">
<head>
  <meta charset="utf-8">
  <meta name="viewport" content="width=device-width, initial-scale=1">
  <meta name="description" content="Live CARLab ADA vehicle WebUI addresses">
  <title>CARLab ADA Fleet Monitor</title>
  <style>
    :root {
      color-scheme: dark;
      --ink: #f7f5ed;
      --muted: #9da8b5;
      --panel: rgba(18, 26, 38, .88);
      --panel-2: #111925;
      --line: rgba(255, 255, 255, .1);
      --gold: #f7c948;
      --green: #42d392;
      --red: #ff6b72;
      --blue: #69a9ff;
    }
    * { box-sizing: border-box; }
    body {
      margin: 0;
      min-height: 100vh;
      color: var(--ink);
      background:
        radial-gradient(circle at 14% 10%, rgba(42, 92, 145, .32), transparent 32rem),
        radial-gradient(circle at 90% 0%, rgba(247, 201, 72, .14), transparent 28rem),
        #07101a;
      font-family: Inter, ui-sans-serif, system-ui, -apple-system, BlinkMacSystemFont, "Segoe UI", sans-serif;
    }
    body::before {
      content: "";
      position: fixed;
      inset: 0;
      pointer-events: none;
      opacity: .14;
      background-image: linear-gradient(rgba(255,255,255,.045) 1px, transparent 1px), linear-gradient(90deg, rgba(255,255,255,.045) 1px, transparent 1px);
      background-size: 42px 42px;
      mask-image: linear-gradient(to bottom, black, transparent 78%);
    }
    .shell { width: min(1180px, calc(100% - 32px)); margin: 0 auto; padding: 42px 0 64px; position: relative; }
    header { display: flex; align-items: center; justify-content: space-between; gap: 28px; margin-bottom: 34px; }
    .brand { display: flex; align-items: center; gap: 16px; }
    .mark { width: 54px; height: 54px; display: grid; place-items: center; border: 1px solid rgba(247,201,72,.45); border-radius: 17px; background: linear-gradient(145deg, #273448, #111925); color: var(--gold); font-weight: 900; letter-spacing: -.08em; box-shadow: 0 14px 40px rgba(0,0,0,.32); }
    .eyebrow { margin: 0 0 4px; color: var(--gold); text-transform: uppercase; letter-spacing: .16em; font-size: 11px; font-weight: 800; }
    h1 { margin: 0; font-size: clamp(25px, 4vw, 40px); letter-spacing: -.035em; }
    .clock { color: var(--muted); font: 600 13px/1.4 ui-monospace, SFMono-Regular, Menlo, monospace; text-align: right; }
    .summary { display: grid; grid-template-columns: 1.6fr repeat(3, 1fr); gap: 14px; margin-bottom: 18px; }
    .card { border: 1px solid var(--line); border-radius: 18px; background: var(--panel); box-shadow: 0 18px 60px rgba(0,0,0,.2); backdrop-filter: blur(16px); }
    .intro { padding: 24px; }
    .intro p { margin: 0; max-width: 44rem; color: var(--muted); line-height: 1.6; }
    .stat { padding: 20px; min-height: 112px; display: flex; flex-direction: column; justify-content: space-between; }
    .stat-label { color: var(--muted); font-size: 12px; font-weight: 700; text-transform: uppercase; letter-spacing: .08em; }
    .stat-value { font-size: 32px; font-weight: 850; letter-spacing: -.04em; }
    .stat.online .stat-value { color: var(--green); }
    .stat.offline .stat-value { color: var(--red); }
    .controls { display: grid; grid-template-columns: minmax(220px, 1fr) auto minmax(170px, auto); gap: 12px; padding: 13px; margin-bottom: 14px; }
    input, select, button { font: inherit; }
    input, select { width: 100%; min-height: 43px; border: 1px solid var(--line); border-radius: 11px; color: var(--ink); background: #0a121d; padding: 0 13px; outline: none; }
    input:focus, select:focus { border-color: var(--gold); box-shadow: 0 0 0 3px rgba(247,201,72,.12); }
    .filters { display: flex; gap: 7px; }
    button { border: 0; border-radius: 10px; padding: 0 14px; color: var(--muted); background: transparent; cursor: pointer; font-weight: 750; }
    button:hover { color: var(--ink); background: rgba(255,255,255,.06); }
    button.active { color: #1a1608; background: var(--gold); }
    .table-wrap { overflow: hidden; }
    table { width: 100%; border-collapse: collapse; }
    th { padding: 14px 18px; color: var(--muted); background: rgba(0,0,0,.17); text-align: left; font-size: 11px; text-transform: uppercase; letter-spacing: .1em; }
    td { padding: 17px 18px; border-top: 1px solid var(--line); vertical-align: middle; }
    .vehicle { display: flex; align-items: center; gap: 12px; font-weight: 820; font-size: 16px; }
    .dot { width: 10px; height: 10px; flex: 0 0 auto; border-radius: 50%; background: var(--red); box-shadow: 0 0 0 5px rgba(255,107,114,.1); }
    .dot.online { background: var(--green); box-shadow: 0 0 0 5px rgba(66,211,146,.1), 0 0 16px rgba(66,211,146,.45); }
    .ip { font: 650 14px ui-monospace, SFMono-Regular, Menlo, monospace; }
    .status { display: inline-flex; align-items: center; border-radius: 999px; padding: 6px 10px; font-size: 11px; font-weight: 850; text-transform: uppercase; letter-spacing: .06em; color: var(--red); background: rgba(255,107,114,.11); }
    .status.online { color: var(--green); background: rgba(66,211,146,.11); }
    .open-link { display: inline-flex; align-items: center; justify-content: center; min-height: 36px; padding: 0 12px; border: 1px solid rgba(105,169,255,.3); border-radius: 10px; color: #a9ccff; text-decoration: none; font-size: 12px; font-weight: 800; }
    .open-link:hover { color: white; border-color: var(--blue); background: rgba(105,169,255,.1); }
    .empty { display: none; padding: 48px 24px; text-align: center; color: var(--muted); }
    .empty.show { display: block; }
    .error { display: none; margin-bottom: 14px; padding: 12px 16px; border: 1px solid rgba(255,107,114,.35); border-radius: 12px; color: #ffc2c5; background: rgba(255,107,114,.09); }
    footer { display: flex; justify-content: space-between; gap: 16px; margin-top: 17px; color: var(--muted); font-size: 12px; }
    @media (max-width: 820px) {
      .summary { grid-template-columns: repeat(3, 1fr); }
      .intro { grid-column: 1 / -1; }
      .controls { grid-template-columns: 1fr; }
      .filters { order: 3; }
      .filters button { min-height: 40px; flex: 1; }
      .hide-small { display: none; }
    }
    @media (max-width: 520px) {
      .shell { width: min(100% - 20px, 1180px); padding-top: 24px; }
      header { align-items: flex-start; }
      .clock { display: none; }
      .summary { gap: 9px; }
      .stat { min-height: 92px; padding: 15px 12px; }
      .stat-value { font-size: 25px; }
      th, td { padding-left: 12px; padding-right: 12px; }
      footer { flex-direction: column; }
    }
  </style>
</head>
<body>
  <main class="shell">
    <header>
      <div class="brand"><div class="mark" aria-hidden="true">CL</div><div><p class="eyebrow">CARLab operations</p><h1>ADA Fleet Monitor</h1></div></div>
      <div class="clock"><div>Automatic refresh</div><div id="refreshed">Waiting for data…</div></div>
    </header>
    <section class="summary" aria-label="Fleet summary">
      <div class="card intro"><p>Find the current address of every ADA vehicle without running <code>ifconfig</code>. Vehicles check in at startup and once per hour.</p></div>
      <div class="card stat"><span class="stat-label">Vehicles</span><strong class="stat-value" id="total">—</strong></div>
      <div class="card stat online"><span class="stat-label">Online</span><strong class="stat-value" id="online">—</strong></div>
      <div class="card stat offline"><span class="stat-label">Offline</span><strong class="stat-value" id="offline">—</strong></div>
    </section>
    <div id="error" class="error" role="alert"></div>
    <section class="card controls" aria-label="Vehicle filters">
      <input id="search" type="search" autocomplete="off" placeholder="Filter by ADA number or IP…" aria-label="Filter vehicles">
      <div class="filters" role="group" aria-label="Filter by status"><button class="active" data-status="all">All</button><button data-status="online">Online</button><button data-status="offline">Offline</button></div>
      <select id="sort" aria-label="Sort vehicles"><option value="name-asc">ADA number ↑</option><option value="name-desc">ADA number ↓</option><option value="online">Online first</option><option value="recent">Most recent</option></select>
    </section>
    <section class="card table-wrap">
      <table aria-label="ADA vehicle addresses">
        <thead><tr><th>Vehicle</th><th>Address</th><th>Status</th><th class="hide-small">Last check-in</th><th>WebUI</th></tr></thead>
        <tbody id="rows"></tbody>
      </table>
      <div id="empty" class="empty">No vehicles match this filter.</div>
    </section>
    <footer><span>Online = checked in during the last 90 minutes</span><span>CARLab · University of Delaware</span></footer>
  </main>
  <script>
    const state = { vehicles: [], status: "all", query: "", sort: "name-asc" };
    const rows = document.querySelector("#rows");
    const empty = document.querySelector("#empty");
    const errorBox = document.querySelector("#error");
    const vehicleNumber = id => Number(id.slice(3)) || 0;
    const relative = value => {
      const seconds = Math.max(0, Math.round((Date.now() - value) / 1000));
      if (seconds < 60) return seconds + "s ago";
      if (seconds < 3600) return Math.floor(seconds / 60) + "m ago";
      if (seconds < 86400) return Math.floor(seconds / 3600) + "h ago";
      return Math.floor(seconds / 86400) + "d ago";
    };
    const hostForUrl = ip => ip.includes(":") ? "[" + ip + "]" : ip;
    function render() {
      const query = state.query.toLowerCase();
      let list = state.vehicles.filter(v => (state.status === "all" || v.status === state.status) && (!query || v.vehicle_id.toLowerCase().includes(query) || v.ip_address.toLowerCase().includes(query)));
      const comparisons = {
        "name-asc": (a,b) => vehicleNumber(a.vehicle_id) - vehicleNumber(b.vehicle_id),
        "name-desc": (a,b) => vehicleNumber(b.vehicle_id) - vehicleNumber(a.vehicle_id),
        "online": (a,b) => Number(b.online) - Number(a.online) || vehicleNumber(a.vehicle_id) - vehicleNumber(b.vehicle_id),
        "recent": (a,b) => b.last_seen - a.last_seen,
      };
      list.sort(comparisons[state.sort]);
      rows.replaceChildren(...list.map(v => {
        const tr = document.createElement("tr");
        const vehicle = document.createElement("td");
        const vehicleWrap = document.createElement("div"); vehicleWrap.className = "vehicle";
        const dot = document.createElement("span"); dot.className = "dot " + v.status;
        const name = document.createElement("span"); name.textContent = v.vehicle_id;
        vehicleWrap.append(dot, name); vehicle.append(vehicleWrap);
        const address = document.createElement("td"); address.className = "ip"; address.textContent = v.ip_address;
        const statusCell = document.createElement("td");
        const badge = document.createElement("span"); badge.className = "status " + v.status; badge.textContent = v.status; statusCell.append(badge);
        const seen = document.createElement("td"); seen.className = "hide-small"; seen.textContent = relative(v.last_seen); seen.title = new Date(v.last_seen).toLocaleString();
        const action = document.createElement("td");
        const link = document.createElement("a"); link.className = "open-link"; link.textContent = "Open"; link.href = "http://" + hostForUrl(v.ip_address) + ":" + v.webui_port; link.target = "_blank"; link.rel = "noopener"; action.append(link);
        tr.append(vehicle, address, statusCell, seen, action); return tr;
      }));
      empty.classList.toggle("show", list.length === 0);
      const online = state.vehicles.filter(v => v.online).length;
      document.querySelector("#total").textContent = state.vehicles.length;
      document.querySelector("#online").textContent = online;
      document.querySelector("#offline").textContent = state.vehicles.length - online;
    }
    async function refresh() {
      try {
        const response = await fetch("/api/vehicles", { cache: "no-store" });
        if (!response.ok) throw new Error("Server returned " + response.status);
        const payload = await response.json();
        state.vehicles = payload.vehicles;
        errorBox.style.display = "none";
        document.querySelector("#refreshed").textContent = "Updated " + new Date().toLocaleTimeString();
        render();
      } catch (error) {
        errorBox.textContent = "Could not load vehicle status: " + error.message;
        errorBox.style.display = "block";
      }
    }
    document.querySelector("#search").addEventListener("input", event => { state.query = event.target.value.trim(); render(); });
    document.querySelector("#sort").addEventListener("change", event => { state.sort = event.target.value; render(); });
    document.querySelectorAll("[data-status]").forEach(button => button.addEventListener("click", () => {
      state.status = button.dataset.status;
      document.querySelectorAll("[data-status]").forEach(item => item.classList.toggle("active", item === button));
      render();
    }));
    refresh(); setInterval(refresh, 60_000); setInterval(render, 10_000);
  </script>
</body>
</html>`;

function response(body, status = 200, headers = {}) {
  return new Response(body, { status, headers: { ...headers, "x-frame-options": "DENY", "referrer-policy": "no-referrer", "permissions-policy": "camera=(), microphone=(), geolocation=()" } });
}

function json(value, status = 200) {
  return response(JSON.stringify(value), status, JSON_HEADERS);
}

function constantTimeEqual(left, right) {
  const a = new TextEncoder().encode(left);
  const b = new TextEncoder().encode(right);
  let mismatch = a.length ^ b.length;
  const size = Math.max(a.length, b.length);
  for (let i = 0; i < size; i += 1) mismatch |= (a[i % Math.max(1, a.length)] || 0) ^ (b[i % Math.max(1, b.length)] || 0);
  return mismatch === 0;
}

function validIp(value) {
  if (typeof value !== "string" || value.length > 45) return false;
  const parts = value.split(".");
  if (parts.length === 4) return parts.every(part => /^\d{1,3}$/.test(part) && Number(part) >= 0 && Number(part) <= 255 && String(Number(part)) === part);
  return value.includes(":") && /^[0-9a-f:]+$/i.test(value) && value.split(":").length <= 8;
}

async function ensureSchema(db) {
  await db.prepare(`CREATE TABLE IF NOT EXISTS vehicles (
    vehicle_id TEXT PRIMARY KEY NOT NULL,
    ip_address TEXT NOT NULL,
    webui_port INTEGER NOT NULL DEFAULT 8080,
    last_seen INTEGER NOT NULL
  )`).run();
  await db.prepare("CREATE INDEX IF NOT EXISTS vehicles_last_seen_idx ON vehicles(last_seen DESC)").run();
}

async function listVehicles(env) {
  await ensureSchema(env.DB);
  const result = await env.DB.prepare("SELECT vehicle_id, ip_address, webui_port, last_seen FROM vehicles ORDER BY last_seen DESC").all();
  const now = Date.now();
  return (result.results || []).map(row => ({
    vehicle_id: row.vehicle_id,
    ip_address: row.ip_address,
    webui_port: row.webui_port,
    last_seen: row.last_seen,
    online: now - row.last_seen <= ONLINE_WINDOW_MS,
    status: now - row.last_seen <= ONLINE_WINDOW_MS ? "online" : "offline",
  }));
}

async function checkIn(request, env) {
  if (!env.CARKIT_REPORTER_TOKEN) return json({ error: "Reporter authentication is not configured" }, 503);
  const authorization = request.headers.get("authorization") || "";
  const expected = `Bearer ${env.CARKIT_REPORTER_TOKEN}`;
  if (!constantTimeEqual(authorization, expected)) return json({ error: "Unauthorized" }, 401);
  const length = Number(request.headers.get("content-length") || 0);
  if (length > 2048) return json({ error: "Request is too large" }, 413);
  let payload;
  try { payload = await request.json(); } catch { return json({ error: "Expected a JSON body" }, 400); }
  const vehicleId = typeof payload.vehicle_id === "string" ? payload.vehicle_id.trim().toUpperCase() : "";
  const ipAddress = typeof payload.ip_address === "string" ? payload.ip_address.trim() : "";
  const webuiPort = Number(payload.webui_port || 8080);
  if (!/^ADA[1-9]\d{0,2}$/.test(vehicleId)) return json({ error: "vehicle_id must look like ADA5" }, 400);
  if (!validIp(ipAddress)) return json({ error: "ip_address is invalid" }, 400);
  if (!Number.isInteger(webuiPort) || webuiPort < 1 || webuiPort > 65535) return json({ error: "webui_port is invalid" }, 400);
  await ensureSchema(env.DB);
  const now = Date.now();
  await env.DB.prepare(`INSERT INTO vehicles (vehicle_id, ip_address, webui_port, last_seen)
    VALUES (?, ?, ?, ?)
    ON CONFLICT(vehicle_id) DO UPDATE SET ip_address = excluded.ip_address, webui_port = excluded.webui_port, last_seen = excluded.last_seen`)
    .bind(vehicleId, ipAddress, webuiPort, now).run();
  return json({ ok: true, vehicle_id: vehicleId, ip_address: ipAddress, last_seen: now });
}

export default {
  async fetch(request, env) {
    const url = new URL(request.url);
    try {
      if (request.method === "GET" && url.pathname === "/") return response(PAGE, 200, { "content-type": "text/html; charset=utf-8", "cache-control": "public, max-age=300", "content-security-policy": "default-src 'self'; style-src 'unsafe-inline'; script-src 'unsafe-inline'; connect-src 'self'; base-uri 'none'; frame-ancestors 'none'; form-action 'none'" });
      if (request.method === "GET" && url.pathname === "/api/vehicles") return json({ vehicles: await listVehicles(env), online_window_ms: ONLINE_WINDOW_MS });
      if (request.method === "POST" && url.pathname === "/api/check-in") return await checkIn(request, env);
      if (request.method === "GET" && url.pathname === "/healthz") return json({ ok: true });
      return json({ error: "Not found" }, 404);
    } catch (error) {
      console.error("request_failed", error);
      return json({ error: "Internal server error" }, 500);
    }
  },
};
