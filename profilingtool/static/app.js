const statusPill = document.getElementById("status-pill");
const errorsBox = document.getElementById("errors");
const refreshSelect = document.getElementById("refresh-interval");
const nodesBody = document.getElementById("nodes-body");
const launchBody = document.getElementById("launch-body");

let timer = null;
let inFlight = false;

function formatNumber(value, digits = 1) {
  if (value === null || value === undefined || Number.isNaN(value)) {
    return "--";
  }
  return Number(value).toFixed(digits);
}

function setStatus(state, label) {
  statusPill.className = `pill pill-${state}`;
  statusPill.textContent = label;
}

function renderUsageBar(value, max = 100) {
  const width = Math.max(0, Math.min(100, (value / max) * 100));
  return `<div class="usage-bar"><span style="width:${width}%"></span></div>`;
}

function renderNodes(nodes) {
  if (!nodes.length) {
    nodesBody.innerHTML = '<tr><td colspan="6" class="empty">No ROS 2 nodes detected in the container.</td></tr>';
    return;
  }

  nodesBody.innerHTML = nodes
    .map((node) => {
      const label = node.node_name || "(unknown)";
      const missing = node.pid === 0;
      return `
        <tr>
          <td class="node-name">${label}${missing ? " <span class='hint'>(no PID)</span>" : ""}</td>
          <td>${missing ? "--" : node.pid}</td>
          <td>${formatNumber(node.cpu_percent)}</td>
          <td>${formatNumber(node.memory_percent)}</td>
          <td>${formatNumber(node.rss_mb)}</td>
          <td class="usage-cell">${renderUsageBar(node.cpu_percent, 25)}</td>
        </tr>
      `;
    })
    .join("");
}

function renderLaunchProcesses(processes) {
  if (!processes.length) {
    launchBody.innerHTML = '<tr><td colspan="5" class="empty">No launch processes.</td></tr>';
    return;
  }

  launchBody.innerHTML = processes
    .map(
      (proc) => `
        <tr>
          <td>${proc.pid}</td>
          <td>${formatNumber(proc.cpu_percent)}</td>
          <td>${formatNumber(proc.memory_percent)}</td>
          <td>${formatNumber(proc.rss_mb)}</td>
          <td class="command" title="${proc.command}">${proc.command}</td>
        </tr>
      `
    )
    .join("");
}

function renderErrors(errors) {
  if (!errors.length) {
    errorsBox.classList.add("hidden");
    errorsBox.textContent = "";
    return;
  }
  errorsBox.classList.remove("hidden");
  errorsBox.innerHTML = errors.map((item) => `<div>${item}</div>`).join("");
}

function renderSnapshot(data) {
  document.getElementById("container-cpu").textContent = `${formatNumber(data.container_cpu_percent)}%`;
  document.getElementById("container-mem").textContent =
    data.container_memory_used_mb != null
      ? `${formatNumber(data.container_memory_used_mb)} / ${formatNumber(data.container_memory_limit_mb)} MB`
      : "--";
  document.getElementById("node-count").textContent = String(data.ros2_nodes?.length ?? data.nodes?.length ?? 0);
  document.getElementById("load-avg").textContent = `Load (${data.system_cpus} CPUs): ${(data.system_load_avg || [])
    .map((value) => formatNumber(value, 2))
    .join(", ")}`;
  document.getElementById("mem-available").textContent = `${formatNumber(data.system_mem_available_mb)} MB`;
  document.getElementById("mem-total").textContent = `Total: ${formatNumber(data.system_mem_total_mb)} MB`;
  document.getElementById("container-cpu-bar").style.width = `${Math.min(100, data.container_cpu_percent || 0)}%`;
  document.getElementById("container-mem-bar").style.width = `${Math.min(100, data.container_memory_percent || 0)}%`;
  document.getElementById("last-updated").textContent = `Updated ${new Date(data.timestamp).toLocaleTimeString()}`;

  renderNodes(data.nodes || []);
  renderLaunchProcesses(data.launch_processes || []);
  renderErrors(data.errors || []);

  if (!data.container_running) {
    setStatus("error", "Container stopped");
  } else if (data.errors?.length) {
    setStatus("idle", "Partial data");
  } else {
    setStatus("live", "Live");
  }
}

async function fetchMetrics() {
  if (inFlight) {
    return;
  }
  inFlight = true;
  try {
    const response = await fetch("/api/metrics", { cache: "no-store" });
    if (!response.ok) {
      throw new Error(`HTTP ${response.status}`);
    }
    const data = await response.json();
    renderSnapshot(data);
  } catch (error) {
    setStatus("error", "Offline");
    renderErrors([`Failed to fetch metrics: ${error.message}`]);
  } finally {
    inFlight = false;
  }
}

function schedulePolling() {
  if (timer) {
    clearInterval(timer);
  }
  const interval = Number(refreshSelect.value);
  fetchMetrics();
  timer = setInterval(fetchMetrics, interval);
}

refreshSelect.addEventListener("change", schedulePolling);
schedulePolling();
