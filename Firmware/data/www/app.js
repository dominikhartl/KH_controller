(function() {
  'use strict';

  var MAX_SCHEDULES = 8;

  // (Volume conversion now done firmware-side)

  // --- WebSocket ---
  var ws, wsOk = false, reconnTimer;

  function connect() {
    var host = location.hostname || 'khcontrollerv3.local';
    ws = new WebSocket('ws://' + host + '/ws');
    ws.onopen = function() {
      wsOk = true;
      setDot('ws', true);
      ws.send(JSON.stringify({type:'getHistory', sensor:'kh'}));
      ws.send(JSON.stringify({type:'getHistory', sensor:'ph'}));
      ws.send(JSON.stringify({type:'getHistory', sensor:'gran'}));
    };
    ws.onclose = function() {
      wsOk = false;
      setDot('ws', false);
      reconnTimer = setTimeout(connect, 3000);
    };
    ws.onmessage = function(e) {
      try { handleMsg(JSON.parse(e.data)); } catch(ex) { console.error('WS parse error:', ex, e.data); }
    };
  }

  function send(obj) {
    if (ws && ws.readyState === 1) ws.send(JSON.stringify(obj));
  }

  // --- Event log ---
  var LOG_MAX = 50;

  function addLogEntry(type, text, epochTs) {
    var container = document.getElementById('log-container');
    if (!container) return;
    var entry = document.createElement('div');
    entry.className = 'log-entry ' + type;
    var t = epochTs ? new Date(epochTs * 1000) : new Date();
    entry.innerHTML = '<span class="log-time">' + pad(t.getHours()) + ':' + pad(t.getMinutes()) + ':' + pad(t.getSeconds()) + '</span><span class="log-text">' + escHtml(text) + '</span>';
    container.insertBefore(entry, container.firstChild);
    while (container.children.length > LOG_MAX) {
      container.removeChild(container.lastChild);
    }
  }

  function loadLogData(entries) {
    var container = document.getElementById('log-container');
    if (!container || !entries || entries.length === 0) return;
    container.innerHTML = '';
    // entries are oldest-first from server; insert each at top so newest ends up on top
    for (var i = 0; i < entries.length; i++) {
      addLogEntry(entries[i].t, entries[i].text, entries[i].ts);
    }
  }

  function escHtml(s) {
    var d = document.createElement('div');
    d.textContent = s;
    return d.innerHTML;
  }

  // --- Progress bar ---
  function updateProgress(pct) {
    var section = document.getElementById('progress-section');
    var fill = document.getElementById('progress-fill');
    var label = document.getElementById('progress-label');
    if (!section || !fill || !label) return;
    if (pct >= 100) {
      section.style.display = 'none';
      return;
    }
    section.style.display = 'flex';
    fill.style.width = pct + '%';
    label.textContent = pct + '%';
  }

  // --- State handling ---
  function handleMsg(d) {
    if (d.type === 'state') updateState(d);
    else if (d.type === 'mesPh') updateLivePH(d);
    else if (d.type === 'mesStart') clearLiveChart();
    else if (d.type === 'mesData') loadMesData(d);
    else if (d.type === 'history') updateHistory(d);
    else if (d.type === 'msg') addLogEntry('msg', d.text);
    else if (d.type === 'error') addLogEntry('error', d.text);
    else if (d.type === 'logData') loadLogData(d.entries);
    else if (d.type === 'granData') updateGranChart(d);
    else if (d.type === 'progress') updateProgress(d.pct);
  }

  function clearLiveChart() {
    if (liveChart) {
      liveChart.data.labels = [];
      liveChart.data.datasets[0].data = [];
      liveChart.update();
    }
    if (granChart) {
      granChart.data.datasets[0].data = [];
      granChart.data.datasets[1].data = [];
      granChart.update();
    }
    var info = document.getElementById('gran-info');
    if (info) { info.style.display = 'none'; info.textContent = ''; }
  }

  function loadMesData(d) {
    if (!liveChart || !d.data || d.data.length === 0) return;
    if (d.chunk === 0) {
      liveChart.data.labels = [];
      liveChart.data.datasets[0].data = [];
    }
    for (var i = 0; i < d.data.length; i++) {
      liveChart.data.labels.push(d.data[i][0].toFixed(2));
      liveChart.data.datasets[0].data.push(d.data[i][1]);
    }
    if (d.chunk === d.total - 1) liveChart.update();
  }

  function updateState(d) {
    // KH gauge
    var khVal = (d.kh > 0) ? d.kh : 0;
    setGaugeArc('gauge-kh-arc', khVal, 0, 15);
    setText('val-kh', (d.kh > 0) ? d.kh.toFixed(1) : '--');

    // pH gauge
    var phVal = (d.lastStartPh > 0) ? d.lastStartPh : 0;
    setGaugeArc('gauge-ph-arc', phVal, 6, 9);
    setText('val-ph', (d.lastStartPh > 0) ? d.lastStartPh.toFixed(2) : '--');

    // HCl tank
    var hclMax = 5000;
    var hclPct = Math.max(0, Math.min(100, ((d.hclVol || 0) / hclMax) * 100));
    var fill = document.getElementById('hcl-fill');
    if (fill) fill.style.height = hclPct + '%';
    setText('val-hcl', d.hclVol ? Math.round(d.hclVol) : '--');

    // Status dots
    setDot('wifi', d.wifiOk);
    setDot('mqtt', d.mqttOk);
    setDot('ntp', d.ntpOk);

    // Status bar
    setText('rssi', d.rssi || '--');
    setText('uptime', fmtUptime(d.uptime || 0));

    // Next measurement (server sends formatted string)
    if (d.nextMeas) setText('next-meas', d.nextMeas);

    // Config values
    if (d.config) {
      setInput('cfg-titration_vol', d.config.titration_vol);
      setInput('cfg-sample_vol', d.config.sample_vol);
      setInput('cfg-correction_factor', d.config.correction_factor);
      setInput('cfg-hcl_molarity', d.config.hcl_molarity);
      setInput('cfg-hcl_volume', d.config.hcl_volume);
      setInput('cfg-cal_drops', d.config.cal_drops);
      setInput('cfg-fast_ph', d.config.fast_ph);
    }

    // Schedule
    if (d.schedule) updateScheduleInputs(d.schedule);

    // Schedule mode
    if (d.schedMode !== undefined) {
      currentSchedMode = d.schedMode;
      setSchedModeUI(currentSchedMode);
    }
    if (d.intervalHours !== undefined) {
      currentIntervalHours = d.intervalHours;
      var sel = document.getElementById('sched-interval-hours');
      if (sel && document.activeElement !== sel) sel.value = d.intervalHours;
    }
    if (d.anchorTime !== undefined) {
      currentAnchorTime = d.anchorTime;
      var anchInp = document.getElementById('sched-anchor-time');
      if (anchInp && document.activeElement !== anchInp) anchInp.value = minsToTime(d.anchorTime);
    }
    updateIntervalPreview();

    // Probe health
    if (d.probe) updateProbeHealth(d.probe);
  }

  function effClass(v) {
    return (v >= 95) ? 'good' : (v >= 85) ? 'fair' : 'replace';
  }

  function updateProbeHealth(p) {
    var healthEl = document.getElementById('probe-health');
    if (healthEl) {
      var h = p.health || '--';
      var cls = (h === 'Good') ? 'good' : (h === 'Fair') ? 'fair' : (h === 'Replace') ? 'replace' : '';
      healthEl.innerHTML = '<span class="health-dot ' + cls + '"></span>' + h;
    }

    // Acid slope Nernst efficiency
    var acidEl = document.getElementById('probe-acid-eff');
    if (acidEl && p.acidEff != null) {
      var av = isNaN(p.acidEff) ? '--' : p.acidEff.toFixed(1);
      acidEl.innerHTML = '<span class="health-dot ' + (isNaN(p.acidEff) ? '' : effClass(p.acidEff)) + '"></span>' + av + ' <small>%</small>';
    }

    // Alkaline slope Nernst efficiency
    var alkEl = document.getElementById('probe-alk-eff');
    if (alkEl && p.alkEff != null) {
      var bv = isNaN(p.alkEff) ? '--' : p.alkEff.toFixed(1);
      alkEl.innerHTML = '<span class="health-dot ' + (isNaN(p.alkEff) ? '' : effClass(p.alkEff)) + '"></span>' + bv + ' <small>%</small>';
    }

    var asymEl = document.getElementById('probe-asymmetry');
    if (asymEl && p.asymmetry != null) {
      var aVal = isNaN(p.asymmetry) ? '--' : p.asymmetry.toFixed(1);
      var aCls = (p.asymmetry < 15) ? 'good' : (p.asymmetry < 25) ? 'fair' : 'replace';
      asymEl.innerHTML = '<span class="health-dot ' + (isNaN(p.asymmetry) ? '' : aCls) + '"></span>' + aVal + ' <small>%</small>';
    }
    var respEl = document.getElementById('probe-response');
    if (respEl && p.response != null) {
      respEl.innerHTML = p.response + ' <small>ms</small>';
    }
    var calEl = document.getElementById('probe-cal-age');
    if (calEl && p.calAge != null) {
      if (p.calAge < 0) {
        calEl.textContent = 'Never';
      } else {
        calEl.innerHTML = p.calAge + ' <small>days</small>';
      }
    }

    // Efficiency trend sparkline (acid slope over calibrations)
    var trendEl = document.getElementById('slope-trend');
    if (p.effHist && p.effHist.length > 1 && trendEl) {
      renderSparkline(trendEl, p.effHist);
    } else if (trendEl) {
      trendEl.innerHTML = '';
    }
  }

  function renderSparkline(container, data) {
    var W = 200, H = 40, PAD = 4;
    var vals = data.map(function(e) { return e[1]; });
    var min = Math.min.apply(null, vals) - 2;
    var max = Math.max.apply(null, vals) + 2;
    if (max - min < 5) { var mid = (max + min) / 2; min = mid - 5; max = mid + 5; }
    var n = vals.length;
    var pts = [];
    for (var i = 0; i < n; i++) {
      var x = PAD + (n > 1 ? i / (n - 1) : 0.5) * (W - 2 * PAD);
      var y = PAD + (1 - (vals[i] - min) / (max - min)) * (H - 2 * PAD);
      pts.push(x.toFixed(1) + ',' + y.toFixed(1));
    }
    var last = vals[n - 1];
    var color = (last >= 95) ? '#30d158' : (last >= 85) ? '#ff9f0a' : '#ff453a';
    container.innerHTML =
      '<svg viewBox="0 0 ' + W + ' ' + H + '" class="sparkline-svg">' +
      '<polyline points="' + pts.join(' ') + '" fill="none" stroke="' + color + '" stroke-width="2" stroke-linecap="round" stroke-linejoin="round"/>' +
      '<circle cx="' + pts[n - 1].split(',')[0] + '" cy="' + pts[n - 1].split(',')[1] + '" r="3" fill="' + color + '"/>' +
      '</svg>';
  }

  function updateLivePH(d) {
    if (liveChart) {
      liveChart.data.labels.push(d.ml.toFixed(2));
      liveChart.data.datasets[0].data.push(d.ph);
      liveChart.update('none');
    }
  }

  function updateGranChart(d) {
    if (!granChart || !d.points || d.points.length === 0) return;

    // Scatter points
    var pts = d.points.map(function(p) { return { x: p[0], y: p[1] }; });
    granChart.data.datasets[0].data = pts;

    // Linear regression for fit line
    var n = pts.length;
    var sx = 0, sy = 0, sxx = 0, sxy = 0;
    for (var i = 0; i < n; i++) {
      sx += pts[i].x; sy += pts[i].y;
      sxx += pts[i].x * pts[i].x; sxy += pts[i].x * pts[i].y;
    }
    var denom = n * sxx - sx * sx;
    if (Math.abs(denom) > 1e-12) {
      var slope = (n * sxy - sx * sy) / denom;
      var intercept = (sy - slope * sx) / n;
      var x0 = pts[0].x;
      var x1 = d.eqML > 0 ? d.eqML : -intercept / slope;
      granChart.data.datasets[1].data = [
        { x: x0, y: slope * x0 + intercept },
        { x: x1, y: 0 }
      ];
    }

    granChart.update();

    // Update info text
    var info = document.getElementById('gran-info');
    if (info) {
      var method = d.used ? 'Gran' : 'Interpolation';
      var r2Text = d.r2 > 0 ? d.r2.toFixed(4) : '--';
      var txt = 'R\u00b2 = ' + r2Text + '  \u2502  Method: ' + method;
      if (d.eqML > 0) txt += '  \u2502  Eq: ' + d.eqML.toFixed(2) + ' mL';
      info.textContent = txt;
      info.style.display = '';
    }
  }

  function updateHistory(d) {
    if (!d.data || !d.sensor) return;
    if (d.sensor === 'gran') { updateGranHistory(d.data); return; }
    var chart = (d.sensor === 'kh') ? khChart : phChart;
    if (!chart) return;
    chart.data.labels = d.data.map(function(p) { return fmtDate(p[0]); });
    chart.data.datasets[0].data = d.data.map(function(p) { return p[1]; });
    chart.update();
  }

  function updateGranHistory(data) {
    if (!granHistChart || !data || data.length === 0) return;
    // data: [[ts, r2, eqML, endpointPH, method], ...]
    granHistChart.data.labels = data.map(function(p) { return fmtDate(p[0]); });
    granHistChart.data.datasets[0].data = data.map(function(p) { return p[1]; }); // R2
    granHistChart.data.datasets[1].data = data.map(function(p) { return p[3]; }); // endpointPH
    // Color interpolation points red
    var r2Colors = data.map(function(p) { return p[4] === 1 ? '#0a84ff' : '#ff453a'; });
    var phColors = data.map(function(p) { return p[4] === 1 ? '#ff9f0a' : '#ff453a'; });
    granHistChart.data.datasets[0].pointBackgroundColor = r2Colors;
    granHistChart.data.datasets[1].pointBackgroundColor = phColors;
    granHistChart.update();
  }

  // --- Gauge arc math ---
  var ARC_LEN = 157;
  function setGaugeArc(id, val, min, max) {
    var el = document.getElementById(id);
    if (!el) return;
    var pct = Math.max(0, Math.min(1, (val - min) / (max - min)));
    el.setAttribute('stroke-dasharray', (pct * ARC_LEN) + ' ' + ARC_LEN);
  }

  // --- Charts ---
  var khChart, phChart, liveChart, granChart, granHistChart;
  var granView = 'last'; // 'last' or 'history'
  var chartOpts = {
    responsive: true,
    maintainAspectRatio: false,
    animation: false,
    plugins: { legend: { display: false } },
    scales: {
      x: { ticks: { color: '#8e8e93', maxTicksLimit: 6, font: { size: 10 } }, grid: { color: '#38383a' } },
      y: { ticks: { color: '#8e8e93', font: { size: 10 } }, grid: { color: '#38383a' } }
    }
  };

  function initCharts() {
    khChart = new Chart(document.getElementById('chart-kh'), {
      type: 'line',
      data: { labels: [], datasets: [{ data: [], borderColor: '#0a84ff', borderWidth: 2, pointRadius: 3, tension: 0.1 }] },
      options: chartOpts
    });
    phChart = new Chart(document.getElementById('chart-ph'), {
      type: 'line',
      data: { labels: [], datasets: [{ data: [], borderColor: '#30d158', borderWidth: 2, pointRadius: 3, tension: 0.1 }] },
      options: chartOpts
    });
    liveChart = new Chart(document.getElementById('chart-live'), {
      type: 'line',
      data: { labels: [], datasets: [{ data: [], borderColor: '#ff9f0a', borderWidth: 2, pointRadius: 0, tension: 0 }] },
      options: Object.assign({}, chartOpts, {
        scales: {
          x: { title: { display: true, text: 'Volume (mL)', color: '#8e8e93' }, ticks: { color: '#8e8e93', font: { size: 10 } }, grid: { color: '#38383a' } },
          y: { title: { display: true, text: 'pH', color: '#8e8e93' }, ticks: { color: '#8e8e93', font: { size: 10 } }, grid: { color: '#38383a' } }
        }
      })
    });
    granChart = new Chart(document.getElementById('chart-gran'), {
      type: 'scatter',
      data: {
        datasets: [
          { label: 'Gran F', data: [], backgroundColor: '#0a84ff', borderColor: '#0a84ff', pointRadius: 5 },
          { label: 'Fit', data: [], borderColor: '#ff9f0a', borderWidth: 2, pointRadius: 0, showLine: true }
        ]
      },
      options: {
        responsive: true, maintainAspectRatio: false, animation: false,
        plugins: { legend: { display: true, labels: { color: '#8e8e93', font: { size: 10 }, boxWidth: 12 } } },
        scales: {
          x: { title: { display: true, text: 'Volume (mL)', color: '#8e8e93' }, ticks: { color: '#8e8e93', font: { size: 10 } }, grid: { color: '#38383a' } },
          y: { title: { display: true, text: 'Gran F', color: '#8e8e93' }, ticks: { color: '#8e8e93', font: { size: 10 } }, grid: { color: '#38383a' } }
        }
      }
    });
    granHistChart = new Chart(document.getElementById('chart-gran-hist'), {
      type: 'line',
      data: {
        labels: [],
        datasets: [
          { label: 'R\u00b2', data: [], borderColor: '#0a84ff', backgroundColor: 'rgba(10,132,255,0.15)', borderWidth: 2, pointRadius: 4, yAxisID: 'yR2', tension: 0.1 },
          { label: 'End pH', data: [], borderColor: '#ff9f0a', backgroundColor: 'rgba(255,159,10,0.15)', borderWidth: 2, pointRadius: 4, yAxisID: 'yRight', tension: 0.1 }
        ]
      },
      options: {
        responsive: true, maintainAspectRatio: false, animation: false,
        plugins: { legend: { display: true, labels: { color: '#8e8e93', font: { size: 10 }, boxWidth: 12 } } },
        scales: {
          x: { ticks: { color: '#8e8e93', maxTicksLimit: 6, font: { size: 10 } }, grid: { color: '#38383a' } },
          yR2: { type: 'linear', position: 'left', min: 0.9, max: 1.0, ticks: { color: '#0a84ff', font: { size: 9 } }, grid: { color: '#38383a' }, title: { display: true, text: 'R\u00b2', color: '#0a84ff', font: { size: 10 } } },
          yRight: { type: 'linear', position: 'right', ticks: { color: '#ff9f0a', font: { size: 9 } }, grid: { drawOnChartArea: false }, title: { display: true, text: 'pH', color: '#ff9f0a', font: { size: 10 } } }
        }
      }
    });
  }

  // --- Tabs ---
  function initTabs() {
    var tabs = document.querySelectorAll('.tab');
    tabs.forEach(function(t) {
      t.addEventListener('click', function() {
        tabs.forEach(function(tt) { tt.classList.remove('active'); });
        t.classList.add('active');
        var sel = t.getAttribute('data-tab');
        // Hide all chart canvases
        ['kh','ph','live','gran','gran-hist'].forEach(function(id) {
          document.getElementById('chart-' + id).style.display = 'none';
        });
        // Show selected
        if (sel === 'gran') {
          showGranView();
        } else {
          document.getElementById('chart-' + sel).style.display = 'block';
        }
        // Gran sub-tabs visibility
        var granSub = document.getElementById('gran-subtabs');
        if (granSub) granSub.style.display = (sel === 'gran') ? 'flex' : 'none';
        // Gran info only in scatter view
        var granInfo = document.getElementById('gran-info');
        if (granInfo) granInfo.style.display = (sel === 'gran' && granView === 'last' && granInfo.textContent) ? '' : 'none';
        // Resize active chart
        if (sel === 'live' && liveChart) liveChart.resize();
        else if (sel === 'kh' && khChart) khChart.resize();
        else if (sel === 'ph' && phChart) phChart.resize();
      });
    });

    // Gran sub-tab toggle
    var btnLast = document.getElementById('gran-tab-last');
    var btnHist = document.getElementById('gran-tab-hist');
    if (btnLast) btnLast.addEventListener('click', function() { switchGranView('last'); });
    if (btnHist) btnHist.addEventListener('click', function() { switchGranView('history'); });
  }

  function switchGranView(view) {
    granView = view;
    var btnLast = document.getElementById('gran-tab-last');
    var btnHist = document.getElementById('gran-tab-hist');
    if (btnLast) btnLast.classList.toggle('active', view === 'last');
    if (btnHist) btnHist.classList.toggle('active', view === 'history');
    showGranView();
    var granInfo = document.getElementById('gran-info');
    if (granInfo) granInfo.style.display = (view === 'last' && granInfo.textContent) ? '' : 'none';
  }

  function showGranView() {
    var scatter = document.getElementById('chart-gran');
    var hist = document.getElementById('chart-gran-hist');
    if (granView === 'last') {
      scatter.style.display = 'block';
      hist.style.display = 'none';
      if (granChart) granChart.resize();
    } else {
      scatter.style.display = 'none';
      hist.style.display = 'block';
      if (granHistChart) granHistChart.resize();
    }
  }

  // --- Collapsible sections ---
  function initCollapsible() {
    document.querySelectorAll('.section-header').forEach(function(header) {
      header.addEventListener('click', function() {
        var section = header.parentElement;
        section.classList.toggle('collapsed');
      });
    });
  }

  // --- Buttons ---
  function initButtons() {
    document.querySelectorAll('.cmd-btn').forEach(function(btn) {
      btn.addEventListener('click', function() {
        var cmd = btn.getAttribute('data-cmd');
        if (cmd === 'o' && !confirm('Restart the device?')) return;
        send({ type: 'cmd', cmd: cmd });
      });
    });
  }

  // --- Config inputs ---
  function initConfigInputs() {
    document.querySelectorAll('.config-grid input').forEach(function(inp) {
      inp.addEventListener('change', function() {
        var key = inp.id.replace('cfg-', '');
        send({ type: 'config', key: key, value: parseFloat(inp.value) });
      });
    });
  }

  // --- Schedule (dynamic add/remove) ---
  var currentSlots = [];
  var currentSchedMode = 0; // 0=custom, 1=interval
  var currentIntervalHours = 6;
  var currentAnchorTime = 360; // minutes from midnight
  var schedLocalUntil = 0; // suppress server updates briefly after local changes

  function renderScheduleInputs() {
    var grid = document.getElementById('sched-grid');
    grid.innerHTML = '';
    for (var i = 0; i < currentSlots.length; i++) {
      var row = document.createElement('div');
      row.className = 'sched-row';

      var inp = document.createElement('input');
      inp.type = 'time';
      inp.value = minsToTime(currentSlots[i]);
      inp.dataset.idx = i;
      inp.addEventListener('change', onSchedChange);

      var rmBtn = document.createElement('button');
      rmBtn.className = 'sched-rm';
      rmBtn.textContent = '\u00d7';
      rmBtn.title = 'Remove';
      rmBtn.dataset.idx = i;
      rmBtn.addEventListener('click', onSchedRemove);

      row.appendChild(inp);
      row.appendChild(rmBtn);
      grid.appendChild(row);
    }

    // Add button (if under max)
    var addBtn = document.getElementById('sched-add');
    if (addBtn) addBtn.style.display = (currentSlots.length < MAX_SCHEDULES) ? '' : 'none';
  }

  function updateScheduleInputs(slots) {
    // Same slot count: update values in-place (preserves DOM, keeps pickers open)
    var inputs = document.querySelectorAll('#sched-grid input[type=time]');
    if (inputs.length === slots.length) {
      for (var i = 0; i < slots.length; i++) {
        currentSlots[i] = slots[i];
        if (document.activeElement !== inputs[i]) {
          inputs[i].value = minsToTime(slots[i]);
        }
      }
      return;
    }
    // Slot count changed — suppress stale broadcasts briefly after local changes
    if (Date.now() < schedLocalUntil) return;
    currentSlots = slots.slice();
    renderScheduleInputs();
  }

  function onSchedChange(e) {
    var idx = parseInt(e.target.dataset.idx);
    currentSlots[idx] = timeToMins(e.target.value);
    sendSchedule();
  }

  function onSchedRemove(e) {
    var idx = parseInt(e.target.dataset.idx);
    currentSlots.splice(idx, 1);
    renderScheduleInputs();
    sendSchedule();
  }

  function onSchedAdd() {
    if (currentSlots.length >= MAX_SCHEDULES) return;
    currentSlots.push(0);
    renderScheduleInputs();
    sendSchedule();
    // Focus the new input
    var inputs = document.querySelectorAll('#sched-grid input[type=time]');
    if (inputs.length > 0) inputs[inputs.length - 1].focus();
  }

  function sendSchedule() {
    schedLocalUntil = Date.now() + 3000;
    send({ type: 'schedule', mode: currentSchedMode, slots: currentSlots });
  }

  function sendScheduleMode() {
    schedLocalUntil = Date.now() + 3000;
    send({
      type: 'schedule',
      mode: currentSchedMode,
      intervalHours: currentIntervalHours,
      anchorTime: currentAnchorTime,
      slots: currentSlots
    });
  }

  function setSchedModeUI(mode) {
    var btns = document.querySelectorAll('.sched-mode-btn');
    btns.forEach(function(b) {
      b.classList.toggle('active', parseInt(b.dataset.mode) === mode);
    });
    var customPanel = document.getElementById('sched-custom-panel');
    var intervalPanel = document.getElementById('sched-interval-panel');
    if (customPanel) customPanel.style.display = (mode === 0) ? '' : 'none';
    if (intervalPanel) intervalPanel.style.display = (mode === 1) ? '' : 'none';
  }

  function updateIntervalPreview() {
    var el = document.getElementById('interval-preview');
    if (!el) return;
    if (currentSchedMode !== 1) { el.textContent = ''; return; }
    var intervalMins = currentIntervalHours * 60;
    var first = currentAnchorTime % intervalMins;
    var times = [];
    for (var t = first; t < 1440; t += intervalMins) {
      times.push(minsToTime(t));
    }
    el.textContent = 'Measurements at: ' + times.join(', ');
  }

  function initSchedule() {
    var addBtn = document.getElementById('sched-add');
    if (addBtn) addBtn.addEventListener('click', onSchedAdd);

    // Mode toggle
    document.querySelectorAll('.sched-mode-btn').forEach(function(btn) {
      btn.addEventListener('click', function() {
        currentSchedMode = parseInt(btn.dataset.mode);
        setSchedModeUI(currentSchedMode);
        updateIntervalPreview();
        sendScheduleMode();
      });
    });

    // Interval hours
    var intSel = document.getElementById('sched-interval-hours');
    if (intSel) intSel.addEventListener('change', function() {
      currentIntervalHours = parseInt(intSel.value);
      updateIntervalPreview();
      sendScheduleMode();
    });

    // Anchor time
    var anchInp = document.getElementById('sched-anchor-time');
    if (anchInp) anchInp.addEventListener('change', function() {
      currentAnchorTime = timeToMins(anchInp.value);
      updateIntervalPreview();
      sendScheduleMode();
    });
  }

  // --- Helpers ---
  function setText(id, val) {
    var el = document.getElementById(id);
    if (el) el.textContent = val;
  }

  function setInput(id, val) {
    var el = document.getElementById(id);
    if (el && document.activeElement !== el && val !== undefined) {
      el.value = val;
    }
  }

  function setDot(name, on) {
    var el = document.getElementById('dot-' + name);
    if (el) { el.className = 'dot ' + (on ? 'on' : 'off'); }
  }

  function fmtUptime(sec) {
    var d = Math.floor(sec / 86400);
    var h = Math.floor((sec % 86400) / 3600);
    var m = Math.floor((sec % 3600) / 60);
    if (d > 0) return d + 'd ' + h + 'h';
    if (h > 0) return h + 'h ' + m + 'm';
    return m + 'm';
  }

  function fmtDate(ts) {
    var d = new Date(ts * 1000);
    return (d.getMonth() + 1) + '/' + d.getDate() + ' ' + pad(d.getHours()) + ':' + pad(d.getMinutes());
  }

  function minsToTime(mins) {
    return pad(Math.floor(mins / 60)) + ':' + pad(mins % 60);
  }

  function timeToMins(t) {
    if (!t) return 0;
    var parts = t.split(':');
    return parseInt(parts[0]) * 60 + parseInt(parts[1]);
  }

  function pad(n) { return n < 10 ? '0' + n : '' + n; }

  // --- Init ---
  function init() {
    initCharts();
    initTabs();
    initCollapsible();
    initButtons();
    initConfigInputs();
    initSchedule();
    connect();
  }

  if (document.readyState === 'loading') {
    document.addEventListener('DOMContentLoaded', init);
  } else {
    init();
  }
})();
