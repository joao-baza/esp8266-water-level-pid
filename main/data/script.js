var gateway = 'ws://' + window.location.hostname + '/ws';
var websocket;

window.addEventListener('load', function () {
  initWebSocket();
});

function getValues() {
  websocket.send('getValues');
}

function initWebSocket() {
  websocket = new WebSocket(gateway);
  websocket.onopen = onOpen;
  websocket.onclose = onClose;
  websocket.onmessage = onMessage;
}

function onOpen() {
  var status = document.getElementById('status');
  status.textContent = 'Sistema ativo';
  status.classList.add('active');
  getValues();
}

function onClose() {
  var status = document.getElementById('status');
  status.textContent = 'Reconectando…';
  status.classList.remove('active');
  setTimeout(initWebSocket, 2000);
}

function updateSliderPWM(element) {
  var sliderNumber = element.id.charAt(element.id.length - 1);
  var sliderValue = element.value;
  document.getElementById('sliderValue' + sliderNumber).textContent = sliderValue;
  websocket.send(sliderNumber + 's' + sliderValue);
}

function updateDistance(data) {
  var el = document.getElementById('distance');
  if (data.distanceStatus === 'timeout') {
    el.textContent = 'Sem leitura (timeout)';
    el.className = 'distance error';
  } else if (data.distanceStatus === 'waiting') {
    el.textContent = '—';
    el.className = 'distance';
  } else {
    el.textContent = data.distance + ' cm';
    el.className = 'distance';
  }
}

function onMessage(event) {
  var data = JSON.parse(event.data);

  if (data.distance !== undefined) {
    updateDistance(data);
  }

  if (data.sliderValue1 !== undefined) {
    document.getElementById('sliderValue1').textContent = data.sliderValue1;
    document.getElementById('slider1').value = data.sliderValue1;
  }
  if (data.sliderValue2 !== undefined) {
    document.getElementById('sliderValue2').textContent = data.sliderValue2;
    document.getElementById('slider2').value = data.sliderValue2;
  }
}
