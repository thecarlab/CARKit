/* Keyboard driving has no ROS dependency; every command still passes the arbiter. */
(function (root) {
  class KeyboardDrive {
    constructor({publish, toggle, emergencyStop, mode, enabled, onChange = () => {}, now = () => performance.now()}) {
      Object.assign(this, {publish, toggle, emergencyStop, mode, enabled, onChange});
      this.now = now;
      this.lastTick = now();
      this.speed = 0;
      this.steering = 0;
      this.keys = new Set();
      this.sending = false;
    }

    editable(target) {
      return Boolean(target && (target.isContentEditable || (target.closest &&
        target.closest('input, textarea, select, [contenteditable], [role="application"]'))));
    }

    keydown(event) {
      if (this.editable(event.target) || event.ctrlKey || event.altKey || event.metaKey ||
          !this.enabled()) return;
      const code = event.code;
      if (code === 'CapsLock') {
        event.preventDefault();
        if (!event.repeat && ['HUMAN_CONTROL', 'AUTO_DRIVE'].includes(this.mode())) {
          this.stop();
          this.toggle();
        }
        return;
      }
      if (code === 'Space') {
        event.preventDefault();
        if (!event.repeat) { this.stop(); this.emergencyStop(); }
        return;
      }
      if (!['KeyW', 'KeyA', 'KeyS', 'KeyD'].includes(code) || this.mode() !== 'HUMAN_CONTROL') return;
      event.preventDefault();
      // Never resume a held key after focus, mode, or connection loss.
      if (event.repeat) return;
      if (!this.keys.size) this.lastTick = this.now();
      this.keys.add(code);
      this.tick();
    }

    keyup(event) {
      if (this.keys.delete(event.code)) {
        event.preventDefault();
        this.tick();
      }
    }

    tick() {
      if (!this.enabled() || this.mode() !== 'HUMAN_CONTROL') { this.stop(); return; }
      if (!this.keys.size) { this.stop(); return; }
      const time = this.now();
      const dt = Math.max(0, Math.min(0.1, (time - this.lastTick) / 1000));
      this.lastTick = time;
      const longitudinal = Number(this.keys.has('KeyW')) - Number(this.keys.has('KeyS'));
      const lateral = Number(this.keys.has('KeyA')) - Number(this.keys.has('KeyD'));
      // Release stops that axis immediately; opposing keys also cancel it.
      if (!longitudinal) this.speed = 0;
      else {
        if (this.speed * longitudinal < 0) this.speed = 0;
        this.speed = longitudinal * Math.min(4, Math.abs(this.speed) + dt * 1.0);
      }
      if (!lateral) this.steering = 0;
      else {
        if (this.steering * lateral < 0) this.steering = 0;
        this.steering = lateral * Math.min(0.25, Math.abs(this.steering) + dt * 0.5);
      }
      if (!this.publish(this.speed, this.steering)) { this.stop(); return; }
      this.sending = true;
      this.onChange([...this.keys]);
    }

    stop() {
      this.speed = 0;
      this.steering = 0;
      this.lastTick = this.now();
      this.keys.clear();
      if (this.sending) this.publish(0, 0);
      this.sending = false;
      this.onChange([]);
    }
  }
  root.CARKitKeyboard = KeyboardDrive;
  if (typeof module !== 'undefined') module.exports = KeyboardDrive;
})(globalThis);
