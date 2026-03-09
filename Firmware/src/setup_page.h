#ifndef SETUP_PAGE_H
#define SETUP_PAGE_H

#include <pgmspace.h>

// Self-contained captive portal page — no LittleFS dependency.
// Dark theme matching main UI. Inline CSS/JS.
const char SETUP_PAGE[] PROGMEM = R"rawliteral(
<!DOCTYPE html>
<html lang="en">
<head>
<meta charset="UTF-8">
<meta name="viewport" content="width=device-width,initial-scale=1">
<title>KH Controller Setup</title>
<style>
*{box-sizing:border-box;margin:0;padding:0}
body{font-family:-apple-system,BlinkMacSystemFont,'SF Pro Display',system-ui,sans-serif;
background:#000;color:#fff;padding:16px;max-width:420px;margin:0 auto;
-webkit-font-smoothing:antialiased}
h1{font-size:1.3em;margin-bottom:4px}
.sub{color:#8e8e93;font-size:0.85em;margin-bottom:20px}
.card{background:#1c1c1e;border-radius:12px;padding:16px;margin-bottom:16px}
.card h2{font-size:0.95em;margin-bottom:12px;color:#8e8e93;text-transform:uppercase;letter-spacing:0.5px}
label{display:block;font-size:0.8em;color:#8e8e93;text-transform:uppercase;margin-bottom:4px;margin-top:10px}
label:first-child{margin-top:0}
input,select{width:100%;padding:10px;background:#2c2c2e;border:1px solid #38383a;border-radius:8px;
color:#fff;font-size:1em;outline:none}
input:focus{border-color:#0a84ff;box-shadow:0 0 0 3px rgba(10,132,255,.15)}
.btn{display:block;width:100%;padding:14px;border:none;border-radius:10px;font-size:1em;
font-weight:600;cursor:pointer;margin-top:16px;-webkit-tap-highlight-color:transparent}
.btn-primary{background:rgba(10,132,255,.15);color:#0a84ff}
.btn-primary:active{opacity:0.7}
.btn-scan{background:rgba(48,209,88,.12);color:#30d158;margin-top:8px;padding:10px;font-size:0.9em}
.btn-scan:active{opacity:0.7}
.toggle{display:flex;align-items:center;gap:8px;margin-top:6px;font-size:0.85em;color:#8e8e93;cursor:pointer}
.toggle input[type=checkbox]{width:auto;cursor:pointer}
.collapse-hdr{cursor:pointer;display:flex;justify-content:space-between;align-items:center}
.collapse-hdr::after{content:'';border:solid #636366;border-width:0 2px 2px 0;padding:3px;
transform:rotate(45deg);transition:transform .2s}
.collapsed .collapse-hdr::after{transform:rotate(-45deg)}
.collapsed .collapse-body{display:none}
.networks{max-height:160px;overflow-y:auto;margin-top:8px}
.net-item{padding:8px 10px;background:#2c2c2e;border-radius:6px;margin-bottom:4px;cursor:pointer;
display:flex;justify-content:space-between;font-size:0.9em}
.net-item:active{background:#38383a}
.net-rssi{color:#8e8e93;font-size:0.85em}
#status{margin-top:12px;padding:10px;border-radius:8px;font-size:0.9em;display:none}
.status-ok{background:rgba(48,209,88,.15);color:#30d158;display:block}
.status-err{background:rgba(255,69,58,.15);color:#ff453a;display:block}
.status-info{background:rgba(10,132,255,.15);color:#0a84ff;display:block}
.pw-wrap{position:relative}
.pw-wrap input{padding-right:50px}
.pw-toggle{position:absolute;right:8px;top:50%;transform:translateY(-50%);
background:none;border:none;color:#0a84ff;font-size:0.8em;cursor:pointer;padding:4px}
.ota-section{margin-top:12px}
.ota-section label{margin-top:8px}
.progress{height:6px;background:#2c2c2e;border-radius:3px;margin-top:8px;display:none}
.progress-fill{height:100%;background:linear-gradient(90deg,#0a84ff,#30d158);border-radius:3px;width:0%;transition:width .3s}
</style>
</head>
<body>
<h1 id="title">KH Controller Setup</h1>
<p class="sub" id="devname"></p>

<div class="card">
<h2>WiFi Network</h2>
<label for="ssid">SSID</label>
<input type="text" id="ssid" placeholder="Network name" maxlength="32" autocomplete="off">
<button class="btn btn-scan" id="btn-scan" onclick="scanNetworks()">Scan for Networks</button>
<div class="networks" id="networks" style="display:none"></div>
<label for="pass">Password</label>
<div class="pw-wrap">
<input type="password" id="pass" placeholder="WiFi password" maxlength="63">
<button class="pw-toggle" onclick="togglePw()">Show</button>
</div>
</div>

<div class="card collapsed" id="mqtt-card">
<div class="collapse-hdr" onclick="this.parentElement.classList.toggle('collapsed')">
<h2 style="margin:0">MQTT (Optional)</h2>
</div>
<div class="collapse-body">
<label for="mqtt_srv">Broker</label>
<input type="text" id="mqtt_srv" value="homeassistant.local" maxlength="63">
<label for="mqtt_port">Port</label>
<input type="number" id="mqtt_port" value="1883" min="1" max="65535">
<label for="mqtt_user">Username</label>
<input type="text" id="mqtt_user" maxlength="31" autocomplete="off">
<label for="mqtt_pass">Password</label>
<input type="password" id="mqtt_pass" maxlength="31">
</div>
</div>

<div class="card collapsed" id="ota-card">
<div class="collapse-hdr" onclick="this.parentElement.classList.toggle('collapsed')">
<h2 style="margin:0">Firmware Update</h2>
</div>
<div class="collapse-body ota-section">
<label for="ota-file">Firmware (.bin)</label>
<input type="file" id="ota-file" accept=".bin">
<button class="btn btn-scan" onclick="uploadFirmware()">Upload Firmware</button>
<div class="progress" id="ota-progress"><div class="progress-fill" id="ota-fill"></div></div>
<div id="ota-status" style="margin-top:8px;font-size:0.85em;color:#8e8e93"></div>
</div>
</div>

<button class="btn btn-primary" onclick="save()">Save & Connect</button>
<div id="status"></div>

<script>
function scanNetworks(){
  var btn=document.getElementById('btn-scan');
  btn.textContent='Scanning...';btn.disabled=true;
  fetch('/api/scan').then(function(r){return r.json()}).then(function(nets){
    var c=document.getElementById('networks');c.innerHTML='';c.style.display='block';
    if(!nets.length){c.innerHTML='<div style="color:#8e8e93;padding:8px">No networks found</div>';return}
    nets.sort(function(a,b){return b.rssi-a.rssi});
    nets.forEach(function(n){
      var d=document.createElement('div');d.className='net-item';
      d.innerHTML='<span>'+esc(n.ssid)+(n.open?'':' &#128274;')+'</span><span class="net-rssi">'+n.rssi+' dBm</span>';
      d.onclick=function(){document.getElementById('ssid').value=n.ssid;document.getElementById('pass').focus()};
      c.appendChild(d);
    });
  }).catch(function(){
    setStatus('Scan failed','err');
  }).finally(function(){
    btn.textContent='Scan for Networks';btn.disabled=false;
  });
}
function togglePw(){
  var p=document.getElementById('pass');
  var b=p.nextElementSibling;
  if(p.type==='password'){p.type='text';b.textContent='Hide'}
  else{p.type='password';b.textContent='Show'}
}
function save(){
  var ssid=document.getElementById('ssid').value.trim();
  if(!ssid){setStatus('Please enter WiFi SSID','err');return}
  var data={
    ssid:ssid,
    password:document.getElementById('pass').value,
    mqtt_server:document.getElementById('mqtt_srv').value.trim(),
    mqtt_port:parseInt(document.getElementById('mqtt_port').value)||1883,
    mqtt_user:document.getElementById('mqtt_user').value.trim(),
    mqtt_pass:document.getElementById('mqtt_pass').value
  };
  setStatus('Saving...','info');
  fetch('/api/setup',{method:'POST',headers:{'Content-Type':'application/json'},body:JSON.stringify(data)})
  .then(function(r){return r.json()}).then(function(j){
    if(j.ok){setStatus('Saved! Restarting device... Connect to your WiFi network and access the device.','ok');
    }else{setStatus('Error: '+(j.error||'Unknown'),'err')}
  }).catch(function(){setStatus('Connection failed','err')});
}
function uploadFirmware(){
  var input=document.getElementById('ota-file');
  if(!input.files.length){setStatus('Select a firmware file first','err');return}
  var file=input.files[0];
  var xhr=new XMLHttpRequest();
  var prog=document.getElementById('ota-progress');
  var fill=document.getElementById('ota-fill');
  var st=document.getElementById('ota-status');
  prog.style.display='block';st.textContent='Uploading...';
  xhr.upload.onprogress=function(e){if(e.lengthComputable){var p=Math.round(e.loaded/e.total*100);fill.style.width=p+'%';st.textContent=p+'%'}};
  xhr.onload=function(){if(xhr.status===200){try{var r=JSON.parse(xhr.responseText);st.textContent=r.ok?'Success! Restarting...':'Error: '+(r.error||'Failed')}catch(e){st.textContent='Done'}}else{st.textContent='Upload failed'}};
  xhr.onerror=function(){st.textContent='Upload failed (connection error)'};
  var fd=new FormData();fd.append('file',file);
  xhr.open('POST','/api/update?type=firmware');xhr.send(fd);
}
function setStatus(msg,type){
  var s=document.getElementById('status');
  s.textContent=msg;s.className='status-'+type;
}
function esc(t){var d=document.createElement('div');d.textContent=t;return d.innerHTML}
fetch('/api/setup').then(function(r){return r.json()}).then(function(d){
  if(d.device_name)document.getElementById('devname').textContent='Device: '+d.device_name;
  if(d.mqtt_server)document.getElementById('mqtt_srv').value=d.mqtt_server;
  if(d.mqtt_port)document.getElementById('mqtt_port').value=d.mqtt_port;
  if(d.mqtt_user)document.getElementById('mqtt_user').value=d.mqtt_user;
}).catch(function(){});
</script>
</body>
</html>
)rawliteral";

#endif // SETUP_PAGE_H
