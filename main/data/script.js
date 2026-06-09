'use strict';

// ============================================================
// CONFIG
// ============================================================

var GATEWAY = 'ws://' + window.location.hostname + '/ws';
var CHART_WINDOW_MS = 180 * 1000;   // 180s
var CHART_WINDOW_S = CHART_WINDOW_MS / 1000;
var CHART_MAX_POINTS = 960;
var Y_MIN = 0;
var Y_MAX = 15.5;                       // cm — limite de proteção da bomba / escala do gráfico
var PWM_Y_MIN = 0;
var PWM_Y_MAX = 100;                  // % — potência da bomba
var ERR_Y_MIN = 0;
var ERR_Y_MAX = 100;                  // % — erro |nível − setpoint|
var COLOR_ERR = '#fb923c';            // laranja — curva de erro %

// ============================================================
// ESTADO
// ============================================================

var ws = null;
var state = {
  mode: 'manual',
  setpoint: 0,
  manualPwm: 0,
  pwm: 0,
  level: 0,
  raw: null,
  kp: 5.0, ki: 0.5, kd: 0.0,
  alpha: 0.3,
  buz: 0,
  alrm: true,
  tank: 15.5,
  status: 'waiting',
  t0: null
};

// Buffer: { t, h, sp (cm), pwm (%), err (%) }
var chartBuffer = [];

// ============================================================
// WEBSOCKET
// ============================================================

window.addEventListener('load', function () {
  var legendMeta = document.querySelector('.legend-meta');
  if (legendMeta) legendMeta.textContent = 'Janela: ' + CHART_WINDOW_S + ' s';
  bindUi();
  initChart();
  initWebSocket();
});

function initWebSocket() {
  ws = new WebSocket(GATEWAY);
  ws.onopen = onOpen;
  ws.onclose = onClose;
  ws.onmessage = onMessage;
  ws.onerror = function () { /* o onclose tratará reconexão */ };
}

function send(msg) {
  if (ws && ws.readyState === WebSocket.OPEN) {
    ws.send(msg);
  }
}

function onOpen() {
  setStatus(true);
  send('getValues');
}

function onClose() {
  setStatus(false);
  setTimeout(initWebSocket, 2000);
}

function setStatus(active) {
  var el = document.getElementById('status');
  el.textContent = active ? 'Sistema ativo' : 'Reconectando…';
  el.classList.toggle('active', active);
}

function onMessage(event) {
  var data;
  try { data = JSON.parse(event.data); } catch (e) { return; }
  applyState(data);
}

// ============================================================
// APLICA ESTADO RECEBIDO
// ============================================================

function applyState(d) {
  if (d.t !== undefined && state.t0 === null) state.t0 = d.t;

  if (d.mode !== undefined)      state.mode = d.mode;
  if (d.sp !== undefined)        state.setpoint = d.sp;
  if (d.h !== undefined)         state.level = d.h;
  if (d.raw !== undefined)       state.raw = d.raw;
  if (d.pwm !== undefined)       state.pwm = d.pwm;
  if (d.manualPwm !== undefined) state.manualPwm = d.manualPwm;
  if (d.kp !== undefined)        state.kp = d.kp;
  if (d.ki !== undefined)        state.ki = d.ki;
  if (d.kd !== undefined)        state.kd = d.kd;
  if (d.alpha !== undefined)     state.alpha = d.alpha;
  if (d.buz !== undefined)       state.buz = d.buz;
  if (d.alrm !== undefined)      state.alrm = d.alrm === 1 || d.alrm === true;
  if (d.tank !== undefined)      state.tank = d.tank;
  if (d.status !== undefined)    state.status = d.status;

  if (d.t !== undefined && d.status === 'ok' && d.h !== undefined) {
    pushChartPoint(
      d.t - state.t0,
      d.h,
      d.sp !== undefined ? d.sp : state.setpoint,
      d.pwm !== undefined ? d.pwm : state.pwm
    );
  }

  renderUi();
  drawChart();
}

// ============================================================
// SLIDER HELPERS
// ============================================================

function updateSetpointDisplay(value) {
  document.getElementById('setpointValue').textContent = parseFloat(value).toFixed(1);
}

function sendSetpoint(value) {
  send('sp' + parseFloat(value).toFixed(2));
}

function updateManualPwmDisplay(value) {
  document.getElementById('manualPwm').textContent = Math.round(value);
}

function sendManualPwm(value) {
  send('1s' + Math.round(value));
}

function bindSliderWheel(slider, opts) {
  slider.addEventListener('wheel', function (event) {
    event.preventDefault();
    var delta = event.deltaY < 0 ? opts.step : -opts.step;
    var value = parseFloat(slider.value) + delta;
    value = Math.max(opts.getMin(), Math.min(opts.getMax(), value));
    if (value === parseFloat(slider.value)) return;
    slider.value = value;
    opts.formatDisplay(value);
    opts.onSend(value);
  }, { passive: false });
}

// ============================================================
// UI BINDINGS
// ============================================================

function bindUi() {
  document.getElementById('modeManual').addEventListener('click', function () {
    send('modem');
  });
  document.getElementById('modeAuto').addEventListener('click', function () {
    send('modea');
  });

  document.getElementById('alarmOn').addEventListener('click', function () {
    send('alrm1');
  });
  document.getElementById('alarmOff').addEventListener('click', function () {
    send('alrm0');
  });

  var sp = document.getElementById('setpoint');
  sp.addEventListener('input', function () {
    updateSetpointDisplay(sp.value);
  });
  sp.addEventListener('change', function () {
    sendSetpoint(sp.value);
  });
  bindSliderWheel(sp, {
    step: 0.1,
    getMin: function () { return 0; },
    getMax: function () { return parseFloat(sp.max); },
    formatDisplay: updateSetpointDisplay,
    onSend: sendSetpoint
  });

  var slider1 = document.getElementById('slider1');
  slider1.addEventListener('input', function () {
    updateManualPwmDisplay(slider1.value);
  });
  slider1.addEventListener('change', function () {
    sendManualPwm(slider1.value);
  });
  bindSliderWheel(slider1, {
    step: 1,
    getMin: function () { return 0; },
    getMax: function () { return 100; },
    formatDisplay: updateManualPwmDisplay,
    onSend: sendManualPwm
  });

  bindNumberInput('kp');
  bindNumberInput('ki');
  bindNumberInput('kd');
  bindAlphaInput();
}

function bindAlphaInput() {
  var el = document.getElementById('alpha');
  el.addEventListener('change', function () {
    var v = parseFloat(el.value);
    if (isNaN(v)) v = 0.3;
    v = Math.max(0.05, Math.min(1, v));
    el.value = v.toFixed(2);
    send('em' + v.toFixed(3));
  });
}

function bindNumberInput(id) {
  var el = document.getElementById(id);
  el.addEventListener('change', function () {
    var v = parseFloat(el.value);
    if (isNaN(v) || v < 0) { v = 0; el.value = '0'; }
    send(id + v.toFixed(3));
  });
}

// ============================================================
// RENDER UI
// ============================================================

function renderUi() {
  setText('levelValue', state.level.toFixed(2));
  setText('pwmValue', state.pwm);

  setText('setpointValue', state.setpoint.toFixed(1));
  var spEl = document.getElementById('setpoint');
  if (document.activeElement !== spEl) spEl.value = state.setpoint;
  spEl.max = state.tank;

  setText('manualPwm', state.manualPwm);
  var slider1 = document.getElementById('slider1');
  if (document.activeElement !== slider1) slider1.value = state.manualPwm;

  syncNumber('kp', state.kp);
  syncNumber('ki', state.ki);
  syncNumber('kd', state.kd);
  syncNumber('alpha', state.alpha);

  var buzEl = document.getElementById('buzValue');
  var alertActive = state.alrm && state.buz > 0;
  buzEl.classList.toggle('alert-dot--on', alertActive);
  if (!state.alrm) {
    buzEl.setAttribute('aria-label', 'Alarme desligado');
  } else if (alertActive) {
    buzEl.setAttribute('aria-label', 'Alarme ativo, ' + state.buz + '%');
  } else {
    buzEl.setAttribute('aria-label', 'Alarme ligado, sem alerta');
  }

  var alarmOn = state.alrm;
  document.getElementById('alarmOn').classList.toggle('active', alarmOn);
  document.getElementById('alarmOff').classList.toggle('active', !alarmOn);
  document.getElementById('alarmOn').setAttribute('aria-pressed', alarmOn);
  document.getElementById('alarmOff').setAttribute('aria-pressed', !alarmOn);

  var manualSection = document.getElementById('manualSection');
  manualSection.classList.toggle('hidden', state.mode !== 'manual');

  document.getElementById('modeManual').classList.toggle('active', state.mode === 'manual');
  document.getElementById('modeAuto').classList.toggle('active', state.mode === 'auto');
  document.getElementById('modeManual').setAttribute('aria-selected', state.mode === 'manual');
  document.getElementById('modeAuto').setAttribute('aria-selected', state.mode === 'auto');

  if (state.status === 'timeout') {
    setStatusText('Sensor sem leitura (timeout)');
  } else if (state.status === 'ok' || state.status === 'waiting') {
    if (ws && ws.readyState === WebSocket.OPEN) setStatusText('Sistema ativo');
  }
}

function setText(id, value) {
  var el = document.getElementById(id);
  if (el) el.textContent = value;
}

function syncNumber(id, value) {
  var el = document.getElementById(id);
  if (document.activeElement !== el) el.value = Number(value).toFixed(2);
}

function setStatusText(text) {
  var s = document.getElementById('status');
  if (s.textContent !== text) s.textContent = text;
}

// ============================================================
// CHART (Canvas 2D) — nível, setpoint e PWM sobrepostos, 180 s
// ============================================================

var canvas, ctx, dpr = 1;

function initChart() {
  canvas = document.getElementById('chart');
  ctx = canvas.getContext('2d');
  resizeChart();
  window.addEventListener('resize', resizeChart);
  if (window.ResizeObserver && canvas.parentElement) {
    new ResizeObserver(resizeChart).observe(canvas.parentElement);
  }
}

function resizeChart() {
  dpr = window.devicePixelRatio || 1;
  var rect = canvas.getBoundingClientRect();
  canvas.width = Math.max(1, Math.floor(rect.width * dpr));
  canvas.height = Math.max(1, Math.floor(rect.height * dpr));
  drawChart();
}

function chartErrPct(h, sp) {
  var tank = state.tank > 0 ? state.tank : Y_MAX;
  return Math.min(ERR_Y_MAX, Math.max(ERR_Y_MIN, Math.abs(h - sp) / tank * 100));
}

function pushChartPoint(tRel, h, sp, pwm) {
  chartBuffer.push({ t: tRel, h: h, sp: sp, pwm: pwm, err: chartErrPct(h, sp) });
  var cutoff = tRel - CHART_WINDOW_MS;
  while (chartBuffer.length > 0 && chartBuffer[0].t < cutoff) chartBuffer.shift();
  while (chartBuffer.length > CHART_MAX_POINTS) chartBuffer.shift();
}

function drawChart() {
  if (!ctx) return;
  var W = canvas.width, H = canvas.height;
  ctx.clearRect(0, 0, W, H);

  ctx.fillStyle = '#0b1220';
  ctx.fillRect(0, 0, W, H);

  var ml = 36 * dpr, mr = 40 * dpr, mt = 12 * dpr, mb = 22 * dpr;
  var plotW = W - ml - mr;
  var plotH = H - mt - mb;
  if (plotW < 10 || plotH < 10) return;

  var tNow = chartBuffer.length > 0 ? chartBuffer[chartBuffer.length - 1].t : 0;
  var tStart = tNow - CHART_WINDOW_MS;

  function xOf(t) {
    return ml + ((t - tStart) / CHART_WINDOW_MS) * plotW;
  }
  function yLevel(v) {
    var clamped = Math.max(Y_MIN, Math.min(Y_MAX, v));
    return mt + (1 - (clamped - Y_MIN) / (Y_MAX - Y_MIN)) * plotH;
  }
  function yPwm(v) {
    var clamped = Math.max(PWM_Y_MIN, Math.min(PWM_Y_MAX, v));
    return mt + (1 - (clamped - PWM_Y_MIN) / (PWM_Y_MAX - PWM_Y_MIN)) * plotH;
  }
  function yErr(v) {
    var clamped = Math.max(ERR_Y_MIN, Math.min(ERR_Y_MAX, v));
    return mt + (1 - (clamped - ERR_Y_MIN) / (ERR_Y_MAX - ERR_Y_MIN)) * plotH;
  }

  ctx.strokeStyle = '#1e293b';
  ctx.lineWidth = 1 * dpr;
  ctx.font = (10 * dpr) + 'px system-ui, sans-serif';

  // Eixo esquerdo — nível (cm), mesma cor da curva de nível
  ctx.fillStyle = '#38bdf8';
  ctx.textAlign = 'right';
  ctx.textBaseline = 'middle';
  for (var lv = Y_MIN; lv <= Y_MAX; lv += 2) {
    var yL = yLevel(lv);
    ctx.beginPath();
    ctx.moveTo(ml, yL);
    ctx.lineTo(W - mr, yL);
    ctx.stroke();
    ctx.fillText(lv + '', ml - 4 * dpr, yL);
  }

  // Eixo direito — PWM e erro (%), escala 0–100
  ctx.fillStyle = '#64748b';
  ctx.textAlign = 'left';
  for (var pv = PWM_Y_MIN; pv <= PWM_Y_MAX; pv += 25) {
    var yP = yPwm(pv);
    ctx.fillText(pv + '%', W - mr + 4 * dpr, yP);
  }

  // Eixo temporal
  ctx.fillStyle = '#64748b';
  ctx.textAlign = 'center';
  ctx.textBaseline = 'top';
  var tickStep = 60;
  for (var sec = 0; sec <= CHART_WINDOW_S; sec += tickStep) {
    var tx = ml + (sec / CHART_WINDOW_S) * plotW;
    ctx.beginPath();
    ctx.moveTo(tx, mt);
    ctx.lineTo(tx, H - mb);
    ctx.stroke();
    var label = '-' + (CHART_WINDOW_S - sec) + 's';
    if (sec === CHART_WINDOW_S) label = 'agora';
    ctx.fillText(label, tx, H - mb + 4 * dpr);
  }

  ctx.strokeStyle = '#334155';
  ctx.strokeRect(ml, mt, plotW, plotH);

  if (chartBuffer.length === 0) return;

  function strokeSeries(getY, color, dashed) {
    ctx.beginPath();
    for (var i = 0; i < chartBuffer.length; i++) {
      var p = chartBuffer[i];
      var x = xOf(p.t);
      var y = getY(p);
      if (i === 0) ctx.moveTo(x, y); else ctx.lineTo(x, y);
    }
    ctx.save();
    if (dashed) ctx.setLineDash([6 * dpr, 5 * dpr]);
    ctx.strokeStyle = color;
    ctx.lineWidth = 2 * dpr;
    ctx.stroke();
    ctx.restore();
  }

  strokeSeries(function (p) { return yLevel(p.sp); }, '#f87171', true);
  strokeSeries(function (p) { return yLevel(p.h); }, '#38bdf8', false);
  strokeSeries(function (p) { return yPwm(p.pwm); }, '#4ade80', false);
  strokeSeries(function (p) { return yErr(p.err); }, COLOR_ERR, false);
}
