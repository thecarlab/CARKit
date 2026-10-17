/* Guided steering calibration. Focus and visibility do not cancel motion. */
(() => {
  const dialog = document.getElementById('calibration-dialog');
  const button = document.getElementById('open-calibration');
  const el = id => document.getElementById(`calibration-${id}`);
  let snapshot = null;
  let owner = null;
  let polling = false;
  let submitting = false;
  let pollTimer;
  const format = (value, places = 2) => Number.isFinite(value) ? value.toFixed(places) : '—';

  function message(text, error = false) {
    el('message').textContent = text;
    el('message').classList.toggle('error', error);
  }
  function controls() {
    const running = Boolean(snapshot?.running);
    const ready = state.runtimeRunning && state.bridgeAuthenticated && state.operator && state.controlMode === 'HUMAN_CONTROL';
    const fresh = el('new').checked;
    const completed = snapshot?.completed_stages?.length || 0;
    el('new').disabled = running;
    const next = fresh ? {id:'forward', label:'Forward only'} : snapshot?.next_stage || (completed === 6 ? null : {id:'forward', label:'Forward only'});
    el('start').textContent = !fresh && snapshot?.progress?.phase === 'complete' && snapshot?.same_configuration
      ? 'Sequence complete' : next ? `Run 1 check: ${next.label}` : 'Analyze saved checks';
    el('confirm-text').textContent = next
      ? `I confirm the vehicle is stationary, the area is clear and attended, and I will stay with it during the selected check or full sequence.`
      : 'I confirm the vehicle is stationary and the area is clear and attended.';
    el('start').disabled = (!fresh && snapshot?.progress?.phase === 'complete' && snapshot?.same_configuration) || submitting || running || !snapshot?.supported || !ready || !el('clear').checked;
    el('all').disabled = el('start').disabled || (!fresh && completed === 6);
    el('all').textContent = !fresh && completed > 0 && completed < 6 ? `Run remaining ${6-completed} checks` : 'Run all 6 checks';
    el('take-control').hidden = !snapshot?.supported || state.operator || running;
    el('take-control').disabled = !state.bridgeAuthenticated || !state.operatorAvailable;
    el('reset').hidden = state.controlMode !== 'EMERGENCY_STOP' || running;
    el('reset').disabled = !state.operator || !state.bridgeAuthenticated;
    el('apply').disabled = submitting || running || !snapshot?.can_apply || !el('restart').checked;
    el('stop').hidden = !snapshot?.supported;
    el('stop').disabled = !running;
    el('clear').disabled = running;
    el('speed').disabled = running;
  }
  function render(value) {
    if (snapshot && snapshot.chassis !== value.chassis) {
      el('clear').checked = false; el('restart').checked = false; el('new').checked = false;
    }
    if (snapshot && ((snapshot.running && !value.running) || snapshot.run_id !== value.run_id || snapshot.next_stage?.id !== value.next_stage?.id)) el('clear').checked = false;
    snapshot = value;
    el('title').textContent = value.method_label || 'Chassis calibration';
    const notRequired = value.chassis === 'osracer' || value.status === 'not_required';
    el('intro').textContent = notRequired ? 'No calibration needed for the OSRacer chassis in CARKit.' : value.supported
      ? 'Choose a single check or all six checks. A single check returns and stops with E-stop latched, waiting for your next confirmation. Run all continues through the unfinished checks automatically, returning between maneuvers, then stops. Green means all six checks passed at the tested speed.'
      : 'Select a supported chassis to configure calibration.';
    el('stages').hidden = !value.supported;
    el('start').hidden = !value.supported;
    el('all').hidden = !value.supported;
    state.calibrationRunning = Boolean(value.running);
    const status = notRequired || value.status === 'good' ? 'good' : value.running ? 'running' : 'needs-calibration';
    button.classList.remove('good', 'running', 'needs-calibration');
    button.classList.add(status);
    button.title = notRequired ? 'No calibration needed for OSRacer' : value.reason;
    button.setAttribute('aria-label', `Calibration: ${notRequired ? 'not needed' : value.running ? 'in progress' : value.status === 'good' ? 'verified' : 'verification required'}`);
    el('state').className = `calibration-state ${status}`;
    el('status-label').textContent = notRequired ? 'No calibration needed' : value.running ? 'Calibration in progress' : value.status === 'good' ? 'Steering verified' : value.progress?.phase === 'awaiting_confirmation' ? 'Waiting for your confirmation' : !value.supported ? 'Calibration method unavailable' : 'Calibration required';
    el('reason').textContent = notRequired ? 'No calibration needed for the OSRacer chassis in CARKit.' : (value.reason || '').replace(/[.!?]$/, '') + '.' + (value.supported ? ` ${value.completed_stages?.length || 0}/6 measurements have enough steady data.` : '') + (value.retry_stages?.length ? ` ${value.retry_stages.length} prior maneuvers need more data; usable samples are saved for the next confirmed attempt.` : '') + (value.legacy_measurements ? ' Older samples without a saved IMU baseline must be remeasured.' : '');
    const progress = value.progress || {};
    el('progress').hidden = !value.running;
    el('setup').hidden = !value.supported || Boolean(value.running);
    el('progress-text').textContent = progress.message || 'Checking sensors…';
    const completed = new Set(value.completed_stages || progress.completed_stages || []);
    el('progress-bar').value = completed.size;
    el('live-values').textContent = value.running
      ? `${format(progress.elapsed_seconds, 0)} s elapsed · ${format(progress.distance_m)} m this stage · ${format(progress.speed_mps)} m/s` : '';
    [...el('stages').children].forEach((node, index) => {
      node.classList.toggle('current', value.running && index === progress.stage_index);
      node.classList.toggle('complete', completed.has(value.stages?.[index]?.id));
    });
    const result = value.result;
    el('results').hidden = !result;
    el('results-body').replaceChildren();
    if (result) {
      for (const stage of result.stages) {
        const row = document.createElement('tr');
        for (const text of [stage.label, `${format(stage.commanded_degrees)}°`, `${format(stage.measured_degrees)}°`, `${format(stage.error_degrees)}°`]) {
          const cell = document.createElement('td'); cell.textContent = text; row.append(cell);
        }
        el('results-body').append(row);
      }
      el('recommendation').replaceChildren();
      const title = document.createElement('h3'); title.textContent = result.good ? 'Current calibration passed' : 'Best-fit recommendation';
      el('recommendation').append(title);
      const list = document.createElement('dl'); list.className = 'calibration-fit';
      for (const [key, label, scale, units] of [
        ['steering_angle_to_servo_offset', 'Servo center', 1, ''],
        ['steering_angle_to_servo_gain', 'Steering gain', 1, ''],
        ['max_steering_angle', 'Steering limit', 180 / Math.PI, '°'],
      ]) {
        const item = document.createElement('div');
        const dt = document.createElement('dt'); dt.textContent = label;
        const dd = document.createElement('dd'); dd.textContent = `${format(result.current[key] * scale, 3)}${units} → ${format(result.recommendation[key] * scale, 3)}${units}`;
        item.append(dt, dd); list.append(item);
      }
      el('recommendation').append(list);
      el('scope').textContent = `${result.scope} Pass limits: straight error ≤ ${result.thresholds.straight_degrees}°; turning RMS error ≤ ${result.thresholds.turn_rms_degrees}°. ${value.same_configuration ? '' : 'These results describe earlier settings; test the current configuration again.'}`;
    }
    if (result?.returns?.length) {
      const last = result.returns[result.returns.length - 1];
      el('scope').textContent += ` Completed return passes: ${result.returns.length}. Estimated offset from the latest session start ${format(last.from_initial_position_m * 100, 1)} cm, heading ${format(last.from_initial_heading_degrees, 1)}°. Position uses IMU/odometry dead reckoning; wheel slip is not independently measured.`;
    }
    el('apply').hidden = !value.can_apply;
    el('apply-confirm').hidden = !value.can_apply;
    if (!value.running && owner) {
      owner = null;
      el('clear').checked = false;
      message(progress.phase === 'complete' ? 'Test finished. The vehicle is stopped with E-stop latched.' : progress.message || value.reason, !['complete','awaiting_confirmation'].includes(progress.phase));
    }
    if (!value.supported) message(value.reason);
    controls();
  }
  async function poll() {
    clearTimeout(pollTimer);
    if (polling) return;
    polling = true;
    try {
      if (state.signedIn) render(await api('/api/calibration'));
    } catch (error) {
      if (dialog.open) message(error.message, true);
      button.classList.remove('good', 'running'); button.classList.add('needs-calibration');
      button.title = 'Calibration status unavailable';

    } finally {
      polling = false;
      pollTimer = setTimeout(poll, dialog.open ? 500 : 2000);
    }
  }
  async function cancel(reason = 'Stop calibration requested') {
    // E-stop is accepted even from an observer; the HTTP cancellation also works
    // if the ROS WebSocket has disconnected. Onboard command and sensor watchdogs remain active.
    owner = null;
    keyboard.stop();
    if (state.bridgeAuthenticated) publishRos('/control_center/emergency_stop', 'std_msgs/msg/Bool', {data:true});
    try { await api('/api/calibration/cancel', {reason}); }
    catch (error) { message(`Stop could not be confirmed. Use the vehicle E-stop if it is still moving. ${error.message}`, true); }
    poll();
  }
  button.onclick = () => {
    keyboard.stop();
    dialog.showModal();
    message('Select Human mode, release the controls, and confirm the next maneuver. Choose one check or the full sequence.');
    poll();
  };
  el('close').onclick = () => dialog.close();
  el('stop').onclick = () => cancel('Stop calibration button pressed');
  el('take-control').onclick = () => document.getElementById('take-control').click();
  el('reset').onclick = () => document.getElementById('keyboard-reset').click();
  el('clear').onchange = controls;
  el('new').onchange = () => { el('clear').checked = false; controls(); };
  el('restart').onchange = controls;
  async function startChecks(mode) {
    if (el(mode === 'all' ? 'all' : 'start').disabled) return;
    submitting = true; controls(); keyboard.stop();
    try {
      owner = await api('/api/calibration/start', {mode, clear_area_confirmed:el('clear').checked, speed:Number(el('speed').value), restart_sequence:el('new').checked, stage_id:el('new').checked ? 'forward' : snapshot.next_stage?.id ?? null});
      el('new').checked = false; el('clear').checked = false;
      message(mode === 'all' ? 'All unfinished checks will run automatically, returning between maneuvers. Use Stop calibration to stop the sequence.' : 'Only the confirmed maneuver and its return will run, then stop and wait for your next confirmation. Use Stop calibration to stop immediately.');
      await poll();
    } catch (error) { message(error.message, true); }
    finally { submitting = false; controls(); }
  };
  el('start').onclick = () => startChecks('single');
  el('all').onclick = () => startChecks('all');
  el('apply').onclick = async () => {
    if (el('apply').disabled) return;
    submitting = true; controls(); keyboard.stop();
    try {
      const result = await api('/api/calibration/apply', {run_id:snapshot.run_id, restart_confirmed:el('restart').checked});
      el('restart').checked = false;
      message(result.message);
      await refresh(); await poll();
    } catch (error) { message(error.message, true); }
    finally { submitting = false; controls(); }
  };
  setInterval(controls, 400);
  poll();
})();
