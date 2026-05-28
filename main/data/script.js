'use strict';

// ============================================================
// CONFIG
// ============================================================

var GATEWAY = 'ws://' + window.location.hostname + '/ws';
var CHART_WINDOW_MS = 120 * 1000;   // 120s
var CHART_MAX_POINTS = 480;          // limite de segurança (240 esperados)
var Y_MIN = 0;
var Y_MAX = 16;                       // cm — TANK_HEIGHT + folga

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
  buz: 0,
  tank: 15.5,
  status: 'waiting',
  t0: null
};

// Buffer do gráfico: { t: ms relativo, h: cm, sp: cm }
var chartBuffer = [];

// ============================================================
// WEBSOCKET
// ============================================================

window.addEventListener('load', function () {
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
  if (d.buz !== undefined)       state.buz = d.buz;
  if (d.tank !== undefined)      state.tank = d.tank;
  if (d.status !== undefined)    state.status = d.status;

  // Adiciona ao buffer apenas quando há nova leitura válida (ok)
  if (d.t !== undefined && d.h !== undefined && d.status === 'ok') {
    pushChartPoint(d.t - state.t0, d.h, d.sp !== undefined ? d.sp : state.setpoint);
  }

  renderUi();
  drawChart();
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

  var sp = document.getElementById('setpoint');
  sp.addEventListener('input', function () {
    document.getElementById('setpointValue').textContent = parseFloat(sp.value).toFixed(1);
  });
  sp.addEventListener('change', function () {
    send('sp' + parseFloat(sp.value).toFixed(2));
  });

  var slider1 = document.getElementById('slider1');
  slider1.addEventListener('input', function () {
    document.getElementById('manualPwm').textContent = slider1.value;
  });
  slider1.addEventListener('change', function () {
    send('1s' + slider1.value);
  });

  bindNumberInput('kp');
  bindNumberInput('ki');
  bindNumberInput('kd');
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
  setText('rawValue', state.raw !== null && state.raw !== undefined
                       ? state.raw.toFixed(1) : '--');

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

  var buzEl = document.getElementById('buzValue');
  if (state.buz > 0) {
    buzEl.textContent = 'ATIVO ' + state.buz + '%';
    buzEl.classList.add('alert');
  } else {
    buzEl.textContent = 'inativo';
    buzEl.classList.remove('alert');
  }

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
// CHART (Canvas 2D)
// ============================================================

var canvas, ctx, dpr = 1;

function initChart() {
  canvas = document.getElementById('chart');
  ctx = canvas.getContext('2d');
  resizeChart();
  window.addEventListener('resize', resizeChart);
}

function resizeChart() {
  dpr = window.devicePixelRatio || 1;
  var rect = canvas.getBoundingClientRect();
  canvas.width = Math.max(1, Math.floor(rect.width * dpr));
  canvas.height = Math.max(1, Math.floor(rect.height * dpr));
  drawChart();
}

function pushChartPoint(tRel, h, sp) {
  chartBuffer.push({ t: tRel, h: h, sp: sp });
  // Mantém buffer dentro da janela; tolera burst com guarda extra
  var cutoff = tRel - CHART_WINDOW_MS;
  while (chartBuffer.length > 0 && chartBuffer[0].t < cutoff) chartBuffer.shift();
  while (chartBuffer.length > CHART_MAX_POINTS) chartBuffer.shift();
}

function drawChart() {
  if (!ctx) return;
  var W = canvas.width, H = canvas.height;
  ctx.clearRect(0, 0, W, H);

  // Fundo
  ctx.fillStyle = '#0b1220';
  ctx.fillRect(0, 0, W, H);

  // Margens
  var ml = 36 * dpr, mr = 12 * dpr, mt = 12 * dpr, mb = 22 * dpr;
  var plotW = W - ml - mr;
  var plotH = H - mt - mb;
  if (plotW < 10 || plotH < 10) return;

  // Eixo: janela = 120s; X cresce para a direita; tNow é o último ponto
  var tNow = chartBuffer.length > 0
    ? chartBuffer[chartBuffer.length - 1].t
    : 0;
  var tStart = tNow - CHART_WINDOW_MS;

  function xOf(t) {
    return ml + ((t - tStart) / CHART_WINDOW_MS) * plotW;
  }
  function yOf(v) {
    var clamped = Math.max(Y_MIN, Math.min(Y_MAX, v));
    return mt + (1 - (clamped - Y_MIN) / (Y_MAX - Y_MIN)) * plotH;
  }

  // Grid horizontal a cada 2 cm
  ctx.strokeStyle = '#1e293b';
  ctx.lineWidth = 1 * dpr;
  ctx.font = (10 * dpr) + 'px system-ui, sans-serif';
  ctx.fillStyle = '#64748b';
  ctx.textAlign = 'right';
  ctx.textBaseline = 'middle';
  for (var v = Y_MIN; v <= Y_MAX; v += 2) {
    var y = yOf(v);
    ctx.beginPath();
    ctx.moveTo(ml, y);
    ctx.lineTo(W - mr, y);
    ctx.stroke();
    ctx.fillText(v + '', ml - 4 * dpr, y);
  }

  // Grid vertical a cada 30s
  ctx.textAlign = 'center';
  ctx.textBaseline = 'top';
  for (var sec = 0; sec <= 120; sec += 30) {
    var tx = ml + (sec / 120) * plotW;
    ctx.beginPath();
    ctx.moveTo(tx, mt);
    ctx.lineTo(tx, H - mb);
    ctx.stroke();
    var label = '-' + (120 - sec) + 's';
    if (sec === 120) label = 'agora';
    ctx.fillText(label, tx, H - mb + 4 * dpr);
  }

  // Borda do plot
  ctx.strokeStyle = '#334155';
  ctx.strokeRect(ml, mt, plotW, plotH);

  if (chartBuffer.length === 0) return;

  // Linha do setpoint (vermelho tracejado)
  ctx.save();
  ctx.setLineDash([6 * dpr, 5 * dpr]);
  ctx.strokeStyle = '#f87171';
  ctx.lineWidth = 2 * dpr;
  ctx.beginPath();
  for (var i = 0; i < chartBuffer.length; i++) {
    var p = chartBuffer[i];
    var x = xOf(p.t);
    var y = yOf(p.sp);
    if (i === 0) ctx.moveTo(x, y); else ctx.lineTo(x, y);
  }
  ctx.stroke();
  ctx.restore();

  // Linha de nível (azul sólido)
  ctx.strokeStyle = '#38bdf8';
  ctx.lineWidth = 2 * dpr;
  ctx.beginPath();
  for (var j = 0; j < chartBuffer.length; j++) {
    var pj = chartBuffer[j];
    var xj = xOf(pj.t);
    var yj = yOf(pj.h);
    if (j === 0) ctx.moveTo(xj, yj); else ctx.lineTo(xj, yj);
  }
  ctx.stroke();
}
