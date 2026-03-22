(function() {
  'use strict';

  var MAX_SCHEDULES = 8;

  // (Volume conversion now done firmware-side)

  // --- WebSocket ---
  var ws, wsOk = false, reconnTimer, reconnDelay = 3000;
  var chartDays = 7;

  function connect() {
    var host = location.hostname;
    ws = new WebSocket('ws://' + host + '/ws');
    ws.onopen = function() {
      if (reconnTimer) { clearTimeout(reconnTimer); reconnTimer = null; }
      reconnDelay = 3000;  // Reset backoff on successful connection
      wsOk = true;
      setDot('ws', true);
      ws.send(JSON.stringify({type:'getHistory', sensor:'kh', days: chartDays}));
      ws.send(JSON.stringify({type:'getHistory', sensor:'ph', days: chartDays}));
      ws.send(JSON.stringify({type:'getHistory', sensor:'gran', days: chartDays}));
      ws.send(JSON.stringify({type:'getHistory', sensor:'motor'}));
      ws.send(JSON.stringify({type:'getHistory', sensor:'precision'}));
    };
    ws.onclose = function() {
      wsOk = false;
      setDot('ws', false);
      reconnTimer = setTimeout(connect, reconnDelay);
      reconnDelay = Math.min(reconnDelay * 2, 60000);  // Exponential backoff, max 60s
    };
    ws.onerror = function() {
      wsOk = false;
      setDot('ws', false);
    };
    ws.onmessage = function(e) {
      var msg;
      try { msg = JSON.parse(e.data); } catch(ex) { console.error('WS JSON parse error:', ex, e.data); return; }
      try { handleMsg(msg); } catch(ex) { console.error('WS handler error:', ex); }
    };
  }

  function send(obj) {
    if (ws && ws.readyState === 1) {
      ws.send(JSON.stringify(obj));
    } else {
      addLogEntry('error', 'Not connected \u2014 command not sent');
    }
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
    else if (d.type === 'mesStart') { clearLiveChart(); setMeasuringMode(true); }
    else if (d.type === 'mesData') { loadMesData(d); setMeasuringMode(false); }
    else if (d.type === 'history') updateHistory(d);
    else if (d.type === 'msg') { addLogEntry('msg', d.text); checkPrecisionResult(d.text); }
    else if (d.type === 'error') { addLogEntry('error', d.text); updateProgress(100); setMeasuringMode(false); }
    else if (d.type === 'logData') loadLogData(d.entries);
    else if (d.type === 'granData') updateGranChart(d);
    else if (d.type === 'progress') updateProgress(d.pct);
    else if (d.type === 'motorDiag') showMotorDiag(d);
    else if (d.type === 'hwDiagDone') showHWDiagDone();
    else if (d.type === 'configResult') {
      var inp = document.getElementById('cfg-' + d.key);
      if (inp) showConfigFeedback(inp, d.ok);
    }
  }

  function clearLiveChart() {
    if (liveChart) {
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
      liveChart.data.datasets[0].data = [];
    }
    for (var i = 0; i < d.data.length; i++) {
      liveChart.data.datasets[0].data.push({x: d.data[i][0], y: d.data[i][1]});
    }
    if (d.chunk === d.total - 1) liveChart.update();
  }

  function updateState(d) {
    // Device name and version
    if (d.deviceName) {
      var title = d.deviceName + (d.fwVersion ? ' v' + d.fwVersion : '');
      setText('device-title', title);
      document.title = title;
      // Update HW diagnostics download filename
      var dlLink = document.getElementById('hw-diag-dl');
      if (dlLink) dlLink.download = d.deviceName + '_hw_diagnostics.json';
    }

    // KH gauge
    var khVal = (d.kh > 0) ? d.kh : 0;
    setGaugeArc('gauge-kh-arc', khVal, 0, 15);
    setText('val-kh', (d.kh > 0) ? d.kh.toFixed(1) : '--');

    // KH slope, intercept, and trend line parameters from server
    if (d.khSlope != null) {
      serverSlope = parseFloat(d.khSlope);
      if (d.khIntercept != null) serverIntercept = parseFloat(d.khIntercept);
      if (d.khSlopeDay0 != null) serverSlopeDay0 = d.khSlopeDay0;
      if (d.slopeNDays != null) serverSlopeNDays = d.slopeNDays;
      if (d.slopeCI != null) lastSlopeCI = parseFloat(d.slopeCI);
      var st = isNaN(serverSlope) ? '--' : (serverSlope >= 0 ? '+' : '') + serverSlope.toFixed(2);
      if (!isNaN(serverSlope) && lastSlopeCI > 0) st += ' \u00b1' + lastSlopeCI.toFixed(2);
      setText('val-kh-slope', st);
      renderKHChart();  // re-render trend line with updated server params
    }
    if (d.confidence != null) {
      setText('val-confidence', (d.confidence != null && !isNaN(d.confidence)) ? (d.confidence * 100).toFixed(0) + '%' : '--');
    }
    if (d.khCI != null && !isNaN(parseFloat(d.khCI))) {
      var ciEl = document.getElementById('val-kh-ci');
      if (ciEl) ciEl.textContent = '\u00b1' + parseFloat(d.khCI).toFixed(2) + ' dKH';
    }

    // pH gauge (start pH from last KH measurement)
    var phVal = (d.lastStartPh > 0) ? d.lastStartPh : 0;
    setGaugeArc('gauge-ph-arc', phVal, 6, 9);
    setText('val-ph', (d.lastStartPh > 0) ? d.lastStartPh.toFixed(2) : '--');

    // Measured pH gauge (latest pH reading from any source)
    var mesPhVal = (d.ph > 0) ? d.ph : 0;
    setGaugeArc('gauge-mesph-arc', mesPhVal, 6, 9);
    setText('val-mesph', mesPhVal > 0 ? mesPhVal.toFixed(2) : '--');

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

    // Measuring state sync — shows/hides Abort button for all clients
    if (d.measuring != null) setMeasuringMode(!!d.measuring);

    // Status bar
    setText('water-temp', d.temp_sensor ? d.water_temp.toFixed(1) + ' \u00B0C' : '--');
    setText('rssi', d.rssi || '--');
    setText('uptime', fmtUptime(d.uptime || 0));

    // Next measurement (server sends formatted string)
    if (d.nextMeas) setText('next-meas', d.nextMeas);

    // Tube health header indicator + section status
    var th = d.tubeHealth || '';
    var tubeInd = document.getElementById('ind-tube');
    if (tubeInd) {
      tubeInd.className = 'si' + ((th === 'Good') ? ' on' : (th === 'Aging') ? ' warn' : (th === 'Replace') ? ' err' : '');
    }
    var tubeStatus = document.getElementById('tube-health-status');
    if (tubeStatus) {
      var dot = tubeStatus.querySelector('.health-dot');
      var cls = (th === 'Good') ? 'good' : (th === 'Aging') ? 'warn' : (th === 'Replace') ? 'replace' : '';
      if (dot) dot.className = 'health-dot' + (cls ? ' ' + cls : '');
      if (tubeStatus.lastChild) tubeStatus.lastChild.textContent = th || '--';
    }

    // Config values
    if (d.config) {
      setInput('cfg-device_name', d.config.device_name);
      setInput('cfg-titration_vol', d.config.titration_vol);
      setInput('cfg-correction_factor', d.config.correction_factor);
      setInput('cfg-hcl_molarity', d.config.hcl_molarity);
      setInput('cfg-hcl_volume', d.config.hcl_volume);
      setInput('cfg-cal_drops', d.config.cal_drops / 100);
      if (_initCalDropsRevs === null) _initCalDropsRevs = d.config.cal_drops / 100;
      setInput('cfg-sample_cal_revs', d.config.sample_cal_revs);
      if (_initSampleCalRevs === null) _initSampleCalRevs = d.config.sample_cal_revs;
      setInput('cfg-fast_ph', d.config.fast_ph);
      setInput('cfg-endpoint_method', d.config.endpoint_method);
      setInput('cfg-min_start_ph', d.config.min_start_ph);
      setInput('cfg-stab_timeout', d.config.stab_timeout);
      setInput('cfg-gran_mix_delay', d.config.gran_mix_delay);
      setInput('cfg-gran_min_r2', d.config.gran_min_r2);
      setInput('cfg-drop_ul', d.config.drop_ul);
      setInput('cfg-titration_rpm', d.config.titration_rpm);
      setInput('cfg-gran_burst_rpm', d.config.gran_burst_rpm);
      setInput('cfg-gran_burst_accel', d.config.gran_burst_accel);
      setInput('cfg-fast_phase_rpm', d.config.fast_phase_rpm);
      setInput('cfg-sample_pump_rpm', d.config.sample_pump_rpm);
      setText('sample-cal-current', d.config.sample_cal_vol > 0 ? '(current: ' + d.config.sample_cal_vol.toFixed(1) + ' mL)' : '');
      setInput('cfg-prefill_ul', d.config.prefill_ul);
      setInput('cfg-max_acid_ml', d.config.max_acid_ml);
      setInput('cfg-fast_step_ul', d.config.fast_step_ul);
      setInput('cfg-meas_temp_c', d.config.meas_temp_c);
      setInput('cfg-buf_ph4', d.config.buf_ph4);
      setInput('cfg-buf_ph7', d.config.buf_ph7);
      setInput('cfg-buf_ph10', d.config.buf_ph10);
      setInput('cfg-slope_hours', d.config.slope_hours);
      setInput('cfg-num_washes', d.config.num_washes);
      setInput('cfg-mqtt_server', d.config.mqtt_server);
      setInput('cfg-mqtt_port', d.config.mqtt_port);
      setInput('cfg-mqtt_user', d.config.mqtt_user);
      setInput('cfg-mqtt_pass', d.config.mqtt_pass);
      if (d.config.ph_sensor != null) {
        var phSel = document.getElementById('cfg-ph_sensor');
        if (phSel) phSel.value = d.config.ph_sensor;
      }
      updateEZOUI(d.config);

      // Sample pump calibration info
      var calInfo = document.getElementById('sample-cal-info');
      if (calInfo && d.config.sample_cal_revs_per_ml) {
        calInfo.textContent = 'Cal factor: ' + d.config.sample_cal_revs_per_ml.toFixed(2) + ' revs/mL';
      }

      // Pump calibration age badges
      var sampAge = document.getElementById('sample-cal-age');
      if (sampAge && d.config.sample_cal_age != null) {
        sampAge.textContent = d.config.sample_cal_age < 0 ? '(never calibrated)' : '(' + d.config.sample_cal_age + 'd ago)';
      }
      var titAge = document.getElementById('titration-cal-age');
      if (titAge && d.config.titration_cal_age != null) {
        titAge.textContent = d.config.titration_cal_age < 0 ? '(never calibrated)' : '(' + d.config.titration_cal_age + 'd ago)';
      }


      // Tube baseline info
      var sbl = d.config.sample_sg_baseline || 0;
      var tbl = d.config.titrate_sg_baseline || 0;
      motorBaselines.sample = sbl;
      motorBaselines.titrate = tbl;
      var blInfo = document.getElementById('tube-baseline-info');
      if (blInfo) {
        if (sbl > 0 || tbl > 0) {
          var parts = [];
          if (sbl > 0) parts.push('S:' + sbl);
          if (tbl > 0) parts.push('T:' + tbl);
          blInfo.textContent = parts.join(' / ');
        } else {
          blInfo.textContent = 'Not set';
        }
      }
      // Update SG values in tube health grid
      var thSample = document.getElementById('tube-health-sample');
      if (thSample && lastSGValues.sample > 0) thSample.textContent = 'SG ' + lastSGValues.sample;
      var thTitrate = document.getElementById('tube-health-titrate');
      if (thTitrate && lastSGValues.titrate > 0) thTitrate.textContent = 'SG ' + lastSGValues.titrate;
      renderMotorCharts();
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

  function effClass(v, ezo) {
    if (ezo) return (v >= 92 && v <= 103) ? 'good' : (v >= 85 && v <= 110) ? 'fair' : 'replace';
    return (v >= 95) ? 'good' : (v >= 85) ? 'fair' : 'replace';
  }

  function updateProbeHealth(p) {
    var isEzo = !!p.ezo;

    // Toggle analog-only / ezo-only items
    document.querySelectorAll('.probe-grid .analog-only').forEach(function(el) { el.style.display = isEzo ? 'none' : ''; });
    document.querySelectorAll('.probe-grid .ezo-only').forEach(function(el) { el.style.display = isEzo ? '' : 'none'; });

    // Hide noise chart when EZO (no ADC noise data)
    var noiseChartContainer = document.getElementById('chart-noise');
    if (noiseChartContainer) noiseChartContainer.parentElement.style.display = isEzo ? 'none' : '';

    var healthEl = document.getElementById('probe-health');
    if (healthEl) {
      var h = p.health || '--';
      var cls = (h === 'Good') ? 'good' : (h === 'Fair') ? 'fair' : (h === 'Replace') ? 'replace' : '';
      healthEl.innerHTML = '<span class="health-dot ' + cls + '"></span>' + h;
    }
    // Header probe indicator
    var probeInd = document.getElementById('ind-probe');
    if (probeInd) {
      var h = p.health || '';
      probeInd.className = 'si' + ((h === 'Good') ? ' on' : (h === 'Fair') ? ' warn' : (h === 'Replace') ? ' err' : '');
    }

    // Acid slope
    var acidEl = document.getElementById('probe-acid-eff');
    if (acidEl && p.acidEff != null) {
      var av = isNaN(p.acidEff) ? '--' : p.acidEff.toFixed(1);
      acidEl.innerHTML = '<span class="health-dot ' + (isNaN(p.acidEff) ? '' : effClass(p.acidEff, isEzo)) + '"></span>' + av + ' <small>%</small>';
    }

    // Base/alkaline slope
    var alkEl = document.getElementById('probe-alk-eff');
    if (alkEl && p.alkEff != null) {
      var bv = isNaN(p.alkEff) ? '--' : p.alkEff.toFixed(1);
      alkEl.innerHTML = '<span class="health-dot ' + (isNaN(p.alkEff) ? '' : effClass(p.alkEff, isEzo)) + '"></span>' + bv + ' <small>%</small>';
    }

    // EZO cal points
    if (isEzo) {
      var cpEl = document.getElementById('probe-cal-pts');
      if (cpEl) cpEl.textContent = (p.calPoints != null) ? p.calPoints : '--';
    }

    var asymEl = document.getElementById('probe-asymmetry');
    if (asymEl && p.asymmetry != null) {
      var aVal = isNaN(p.asymmetry) ? '--' : p.asymmetry.toFixed(1);
      var aCls = (p.asymmetry < 15) ? 'good' : (p.asymmetry < 25) ? 'fair' : 'replace';
      asymEl.innerHTML = '<span class="health-dot ' + (isNaN(p.asymmetry) ? '' : aCls) + '"></span>' + aVal + ' <small>%</small>';
    }
    // Noise — use last gran history entry (same source as noise chart)
    var noiseEl = document.getElementById('probe-noise');
    if (noiseEl) {
      var nv = NaN;
      if (granHistoryData && granHistoryData.length > 0) {
        for (var i = granHistoryData.length - 1; i >= 0; i--) {
          if (granHistoryData[i][7] > 0) { nv = granHistoryData[i][7]; break; }
        }
      }
      if (isNaN(nv) || nv <= 0) {
        noiseEl.innerHTML = '<span class="health-dot"></span>--';
      } else {
        var nCls = (nv < 5) ? 'good' : (nv < 8) ? 'fair' : 'replace';
        noiseEl.innerHTML = '<span class="health-dot ' + nCls + '"></span>' + nv.toFixed(1) + ' <small>mV</small>';
      }
    }

    var calEl = document.getElementById('probe-cal-age');
    if (calEl && p.calAge != null) {
      if (p.calAge < 0) {
        calEl.textContent = 'Never';
      } else {
        calEl.innerHTML = p.calAge + ' <small>days</small>';
      }
    }

    var mvEl = document.getElementById('probe-mv');
    if (mvEl && p.mV != null) {
      mvEl.innerHTML = (isNaN(p.mV) || p.mV === 0) ? '--' : p.mV.toFixed(1) + ' <small>mV</small>';
    }

    var calVEl = document.getElementById('probe-cal-v');
    if (calVEl && p.v4 != null) {
      var fmt = function(v) { return (isNaN(v) || v === 0) ? '--' : v.toFixed(0); };
      calVEl.innerHTML = fmt(p.v4) + ' / ' + fmt(p.v7) + ' / ' + fmt(p.v10) + ' <small>mV</small>';
    }

    // pH calibration mV per column
    var fmv = function(v) { return (isNaN(v) || v === 0) ? '--' : v.toFixed(0) + ' mV'; };
    var mv4 = document.getElementById('cal-mv-4');
    var mv7 = document.getElementById('cal-mv-7');
    var mv10 = document.getElementById('cal-mv-10');
    if (mv4 && p.v4 != null) mv4.textContent = fmv(p.v4);
    if (mv7 && p.v7 != null) mv7.textContent = fmv(p.v7);
    if (mv10 && p.v10 != null) mv10.textContent = fmv(p.v10);

    // pH calibration age badge
    var phAge = document.getElementById('ph-cal-age');
    if (phAge && p.calAge != null) {
      phAge.textContent = p.calAge < 0 ? '(never calibrated)' : '(' + p.calAge + 'd ago)';
    }

    // Asymmetry trend chart (over calibrations)
    if (effChart && p.effHist && p.effHist.length > 0) {
      effChart.data.datasets[0].data = p.effHist.map(function(e) { return {x: e[0], y: e[1]}; });
      var lastAsym = p.effHist[p.effHist.length - 1][1];
      var asymColor = (lastAsym < 15) ? '#30d158' : (lastAsym < 25) ? '#ff9f0a' : '#ff453a';
      effChart.data.datasets[0].borderColor = asymColor;
      effChart.data.datasets[0].pointBackgroundColor = asymColor;
      effChart.options.scales.x.min = p.effHist[0][0];
      effChart.options.scales.x.max = p.effHist[p.effHist.length - 1][0];
      effChart.update();
    }

    // Noise trend chart (from gran history data)
    renderNoiseTrend();
  }

  function updateEZOUI(cfg) {
    var badge = document.getElementById('ph-sensor-badge');
    if (badge && cfg.ph_sensor_name) badge.textContent = cfg.ph_sensor_name;

    var ezoInfo = document.getElementById('ezo-cal-info');
    var ezoActive = cfg.ezo_active;
    if (ezoInfo) ezoInfo.style.display = ezoActive ? '' : 'none';

    if (ezoActive) {
      var pts = document.getElementById('ezo-cal-pts');
      if (pts) pts.textContent = cfg.ezo_cal_points || 0;
      var acid = document.getElementById('ezo-acid-slope');
      if (acid) acid.textContent = isNaN(cfg.ezo_acid_slope) ? '--' : cfg.ezo_acid_slope.toFixed(1);
      var base = document.getElementById('ezo-base-slope');
      if (base) base.textContent = isNaN(cfg.ezo_base_slope) ? '--' : cfg.ezo_base_slope.toFixed(1);
    }

    // Hide cal mV values when EZO active (no voltage data)
    var calMvEls = document.querySelectorAll('.cal-mv');
    calMvEls.forEach(function(el) { el.style.display = ezoActive ? 'none' : ''; });

    // Enforce EZO calibration order: mid (7) first, then low (4), then high (10)
    var btn4 = document.querySelector('[data-cmd="4"]');
    var btn10 = document.querySelector('[data-cmd="10"]');
    if (ezoActive && btn4 && btn10) {
      var calPts = cfg.ezo_cal_points || 0;
      btn4.disabled = (calPts < 1);   // Need mid-point first
      btn10.disabled = (calPts < 2);  // Need mid + low first
      btn4.title = calPts < 1 ? 'Calibrate pH 7 first' : '';
      btn10.title = calPts < 2 ? 'Calibrate pH 7 and pH 4 first' : '';
    } else if (btn4 && btn10) {
      btn4.disabled = false;
      btn10.disabled = false;
      btn4.title = '';
      btn10.title = '';
    }
  }

  function renderNoiseTrend() {
    if (!noiseChart) return;
    if (!granHistoryData || granHistoryData.length < 1) return;
    var data = [];
    for (var i = 0; i < granHistoryData.length; i++) {
      var nv = granHistoryData[i][7];
      if (nv > 0) data.push([granHistoryData[i][0], nv]);
    }
    if (data.length === 0) return;
    noiseChart.data.datasets[0].data = data.map(function(e) { return {x: e[0], y: e[1]}; });
    var last = data[data.length - 1][1];
    var nColor = (last < 5) ? '#30d158' : (last < 8) ? '#ff9f0a' : '#ff453a';
    noiseChart.data.datasets[0].borderColor = nColor;
    noiseChart.data.datasets[0].pointBackgroundColor = nColor;
    noiseChart.options.scales.x.min = data[0][0];
    noiseChart.options.scales.x.max = data[data.length - 1][0];
    noiseChart.update();
  }

  function updateLivePH(d) {
    var mesPhVal = (d.ph > 0) ? d.ph : 0;
    setGaugeArc('gauge-mesph-arc', mesPhVal, 6, 9);
    setText('val-mesph', mesPhVal > 0 ? mesPhVal.toFixed(2) : '--');

    if (liveChart) {
      liveChart.data.datasets[0].data.push({x: d.ml, y: d.ph});
      liveChart.update('none');
    }
  }

  function updateGranChart(d) {
    if (!granChart) return;
    if (!d.points || d.points.length === 0) {
      for (var c = 0; c < 5; c++) granChart.data.datasets[c].data = [];
      granChart.update();
      return;
    }

    // Scatter points
    var pts = d.points.map(function(p) { return { x: p[0], y: p[1] }; });
    granChart.data.datasets[0].data = pts;

    // Fit line from firmware's weighted regression — extend across full data range
    if (d.slope && d.intercept !== undefined) {
      var xMin = Math.min.apply(null, pts.map(function(p) { return p.x; }));
      var xMax = Math.max.apply(null, pts.map(function(p) { return p.x; }));
      var xStart = d.eqML > 0 ? Math.min(d.eqML, xMin) : xMin;
      granChart.data.datasets[1].data = [
        { x: xStart, y: d.slope * xStart + d.intercept },
        { x: xMax, y: d.slope * xMax + d.intercept }
      ];
    } else {
      granChart.data.datasets[1].data = [];
    }

    // Equivalence point marker (red X at F=0)
    granChart.data.datasets[4].data = (d.eqML > 0) ? [{x: d.eqML, y: 0}] : [];

    // Window boundary vertical lines
    if (d.winLowML > 0 && d.winHighML > 0) {
      var yMax = Math.max.apply(null, pts.map(function(p) { return p.y; }));
      granChart.data.datasets[2].data = [{x: d.winLowML, y: 0}, {x: d.winLowML, y: yMax}];
      granChart.data.datasets[3].data = [{x: d.winHighML, y: 0}, {x: d.winHighML, y: yMax}];
    } else {
      granChart.data.datasets[2].data = [];
      granChart.data.datasets[3].data = [];
    }

    granChart.update();

    // Update info text
    var info = document.getElementById('gran-info');
    if (info) {
      var method = d.used ? 'Gran' : 'Interpolation';
      var r2Text = d.r2 > 0 ? d.r2.toFixed(5) : '--';
      var txt = 'R\u00b2 = ' + r2Text + '  \u2502  Method: ' + method;
      if (d.eqML > 0) txt += '  \u2502  Eq: ' + d.eqML.toFixed(2) + ' mL';
      if (d.winLowML > 0 && d.winHighML > 0)
        txt += '  \u2502  Window: ' + d.winLowML.toFixed(2) + '-' + d.winHighML.toFixed(2) + ' mL';
      info.textContent = txt;
      info.style.display = '';
    }

    updateGranWindows(d);
  }

  function updateGranWindows(d) {
    if (!granWinChart || !d.windows || d.windows.length === 0) return;
    // Find best window (highest R² among valid)
    var bestLow = 0, bestHigh = 0, bestR2 = 0;
    for (var i = 0; i < d.windows.length; i++) {
      var w = d.windows[i]; // [low, high, r2, valid, kh]
      if (w[3] && w[2] > bestR2) { bestR2 = w[2]; bestLow = w[0]; bestHigh = w[1]; }
    }
    // One horizontal line per window: from lower bound to upper bound at R² height
    // Blue lines semi-transparent so overlaps appear darker; best (red) on top
    var datasets = [];
    for (var i = 0; i < d.windows.length; i++) {
      var w = d.windows[i];
      if (!w[3]) continue; // skip invalid
      var isBest = (w[0] === bestLow && w[1] === bestHigh);
      var kh = w[4] || 0;
      if (!isBest) {
        datasets.push({
          data: [{ x: w[0], y: w[2] }, { x: w[1], y: w[2] }],
          borderColor: 'rgba(10,132,255,0.5)',
          borderWidth: 2,
          pointRadius: 3,
          pointBackgroundColor: 'rgba(10,132,255,0.5)',
          showLine: true,
          khLabel: kh > 0 ? kh.toFixed(1) : ''
        });
      }
    }
    // Best window last so it renders on top
    var bestKh = 0;
    for (var i = 0; i < d.windows.length; i++) {
      var w = d.windows[i];
      if (w[3] && w[0] === bestLow && w[1] === bestHigh) bestKh = w[4] || 0;
    }
    datasets.push({
      data: [{ x: bestLow, y: bestR2 }, { x: bestHigh, y: bestR2 }],
      borderColor: '#ff453a',
      borderWidth: 3,
      pointRadius: 4,
      pointBackgroundColor: '#ff453a',
      showLine: true,
      khLabel: bestKh > 0 ? bestKh.toFixed(1) : ''
    });
    granWinChart.data.datasets = datasets;
    granWinChart.update();
  }

  function updateHistory(d) {
    if (!d.data || !d.sensor) return;
    if (d.sensor === 'gran') {
      granHistoryData = d.data;
      updateGranHistChart();
      renderKHChart(); // gran data may affect KH chart when method filter is active
      renderNoiseTrend();
      return;
    }
    if (d.sensor === 'precision') {
      renderPrecisionHistory(d.data);
      return;
    }
    if (d.sensor === 'motor') {
      motorHistoryData = d.data;
      if (d.data && d.data.length > 0) {
        var last = d.data[d.data.length - 1];
        lastSGValues.sample = last[1];
        lastSGValues.titrate = last[3];
      }
      renderMotorCharts();
      return;
    }
    if (d.sensor === 'kh') {
      khHistoryData = d.data;
      renderKHChart();
      return;
    }
    // pH chart
    if (!phChart) return;
    phChart.data.datasets[0].data = d.data.map(function(p) { return {x: p[0], y: p[1]}; });
    if (d.data.length > 0) {
      phChart.options.scales.x.min = d.data[0][0];
      phChart.options.scales.x.max = d.data[d.data.length - 1][0];
    }
    phChart.update();
  }

  function renderKHChart() {
    if (!khChart) return;
    // Build data series based on selected method
    var data;
    if (khMethod === 'combined' || !granHistoryData) {
      data = khHistoryData || [];
    } else {
      // Extract from gran history: [ts, r2, eqML, eph, mth, khG, khE]
      var idx = (khMethod === 'gran') ? 5 : 6;
      data = [];
      for (var i = 0; i < granHistoryData.length; i++) {
        var val = granHistoryData[i][idx];
        if (val > 0) data.push([granHistoryData[i][0], val]);
      }
    }
    if (!data || data.length === 0) {
      khChart.data.labels = [];
      khChart.data.datasets[0].data = [];
      khChart.data.datasets[1].data = [];
      khChart.data.datasets[2].data = [];
      khChart.data.datasets[3].data = [];
      khChart.data.datasets[4].data = [];
      khChart.data.datasets[5].data = [];
      khChart.update();
      setText('val-kh-slope', '--');
      return;
    }
    // Dataset 0: scatter points
    khChart.data.datasets[0].data = data.map(function(p) { return {x: p[0], y: p[1]}; });

    // Dataset 1: Bidirectional EMA smooth line (adaptive half-life, time-aware)
    var hlInp = document.getElementById('ui-smooth-halflife');
    var hlVal = hlInp ? parseFloat(hlInp.value) : 0;
    var spanH = (data[data.length - 1][0] - data[0][0]) / 3600;
    var halfLife = (hlVal > 0) ? hlVal : Math.max(2, Math.min(24, spanH / 8));
    var fwd = [data[0][1]];
    for (var i = 1; i < data.length; i++) {
      var dtH = (data[i][0] - data[i-1][0]) / 3600;
      var alpha = 1 - Math.pow(0.5, dtH / halfLife);
      fwd.push(alpha * data[i][1] + (1 - alpha) * fwd[i-1]);
    }
    var bwd = new Array(data.length);
    bwd[data.length - 1] = data[data.length - 1][1];
    for (var i = data.length - 2; i >= 0; i--) {
      var dtH = (data[i+1][0] - data[i][0]) / 3600;
      var alpha = 1 - Math.pow(0.5, dtH / halfLife);
      bwd[i] = alpha * data[i][1] + (1 - alpha) * bwd[i+1];
    }
    var smooth = [];
    for (var i = 0; i < data.length; i++) {
      smooth.push({x: data[i][0], y: Math.round((fwd[i] + bwd[i]) / 2 * 1000) / 1000});
    }
    khChart.data.datasets[1].data = smooth;

    // Per-point score coloring on Dataset 0 (when Score checkbox is checked)
    var showScore = document.getElementById('kh-show-score');
    if (showScore && showScore.checked && granHistoryData) {
      var confByTs = {};
      for (var i = 0; i < granHistoryData.length; i++) confByTs[granHistoryData[i][0]] = granHistoryData[i][9];
      var ptBg = [];
      var ptBorder = [];
      for (var i = 0; i < data.length; i++) {
        var v = confByTs[data[i][0]];
        if (v != null) {
          ptBg.push(v < 0.8 ? '#ff3b30' : '#30d158');
          ptBorder.push(v < 0.8 ? '#ff3b30' : '#30d158');
        } else {
          ptBg.push('#0a84ff');
          ptBorder.push('#0a84ff');
        }
      }
      khChart.data.datasets[0].pointBackgroundColor = ptBg;
      khChart.data.datasets[0].pointBorderColor = ptBorder;
    } else {
      khChart.data.datasets[0].pointBackgroundColor = '#0a84ff';
      khChart.data.datasets[0].pointBorderColor = '#0a84ff';
    }
    khChart.data.datasets[3].data = [];

    // Per-point error bars from Gran regression CI (index 10 in gran history)
    khErrorBars = [];
    if (granHistoryData) {
      var ciByTs = {};
      for (var i = 0; i < granHistoryData.length; i++) {
        var ciVal = granHistoryData[i][10];
        if (ciVal > 0) ciByTs[granHistoryData[i][0]] = ciVal;
      }
      for (var i = 0; i < data.length; i++) {
        var ci = ciByTs[data[i][0]];
        if (ci > 0) khErrorBars.push({idx: i, val: data[i][1], ci: ci});
      }
    }

    // Dataset 2: trend line from server regression (slope, intercept, day0)
    // Uses exactly the same slope the device reports — guaranteed match, no knee.
    var slopeHoursEl = document.getElementById('cfg-slope_hours');
    var slopeHours = slopeHoursEl ? (parseInt(slopeHoursEl.value) || 72) : 72;
    var now = data[data.length - 1][0];
    var cutoff = now - slopeHours * 3600;
    var trendOk = false;
    if (!isNaN(serverSlope) && !isNaN(serverIntercept) && serverSlopeDay0 > 0) {
      // Find first and last data points within the slope window
      var firstIdx = -1, lastIdx = -1;
      for (var i = 0; i < data.length; i++) {
        if (data[i][0] >= cutoff) {
          if (firstIdx < 0) firstIdx = i;
          lastIdx = i;
        }
      }
      if (firstIdx >= 0 && lastIdx > firstIdx) {
        // 2-point straight line using {x,y} points
        var xFirst = data[firstIdx][0] / 86400 - serverSlopeDay0;
        var xLast = data[lastIdx][0] / 86400 - serverSlopeDay0;
        khChart.data.datasets[2].data = [
          {x: data[firstIdx][0], y: serverSlope * xFirst + serverIntercept},
          {x: data[lastIdx][0], y: serverSlope * xLast + serverIntercept}
        ];

        // Datasets 4+5: slope CI band (parallel lines offset by ±slopeCI × timespan)
        if (lastSlopeCI > 0) {
          var xMid = (xFirst + xLast) / 2;
          var upperData = [];
          var lowerData = [];
          // CI band widens with distance from center
          [firstIdx, lastIdx].forEach(function(idx) {
            var x = data[idx][0] / 86400 - serverSlopeDay0;
            var yHat = serverSlope * x + serverIntercept;
            var dx = x - xMid;
            var band = lastSlopeCI * Math.abs(dx);
            upperData.push({x: data[idx][0], y: yHat + band});
            lowerData.push({x: data[idx][0], y: yHat - band});
          });
          khChart.data.datasets[4].data = upperData;
          khChart.data.datasets[5].data = lowerData;
        } else {
          khChart.data.datasets[4].data = [];
          khChart.data.datasets[5].data = [];
        }
        trendOk = true;
      }
    }
    if (!trendOk) {
      khChart.data.datasets[2].data = [];
      khChart.data.datasets[4].data = [];
      khChart.data.datasets[5].data = [];
    }
    // Enforce minimum 1.5 dKH span on y-axis
    var vals = khChart.data.datasets[0].data.map(function(p) { return p.y; }).filter(function(v) { return v != null; });
    if (vals.length > 0) {
      var mn = Math.min.apply(null, vals);
      var mx = Math.max.apply(null, vals);
      if (mx - mn < 1.5) {
        var mid = (mn + mx) / 2;
        khChart.options.scales.y.min = mid - 0.75;
        khChart.options.scales.y.max = mid + 0.75;
      } else {
        delete khChart.options.scales.y.min;
        delete khChart.options.scales.y.max;
      }
    }
    khChart.options.scales.x.min = data[0][0];
    khChart.options.scales.x.max = data[data.length - 1][0];
    khChart.update();
  }

  function updateGranHistChart() {
    if (!granHistChart || !granHistoryData || granHistoryData.length === 0) return;
    // data: [[ts, r2, eqML, endpointPH, method, khGran, khEndpoint, noiseMv, reversals, conf, khCI], ...]
    granHistChart.data.datasets[0].data = granHistoryData.map(function(p) { return {x: p[0], y: p[1]}; }); // R2
    granHistChart.data.datasets[1].data = granHistoryData.map(function(p) { return {x: p[0], y: p[3]}; }); // endpointPH
    granHistChart.data.datasets[2].data = granHistoryData.map(function(p) { return p[10] > 0 ? {x: p[0], y: p[10]} : null; }).filter(function(v) { return v !== null; }); // CI
    // Color interpolation points red
    var r2Colors = granHistoryData.map(function(p) { return p[4] === 1 ? '#0a84ff' : '#ff453a'; });
    var phColors = granHistoryData.map(function(p) { return p[4] === 1 ? '#ff9f0a' : '#ff453a'; });
    granHistChart.data.datasets[0].pointBackgroundColor = r2Colors;
    granHistChart.data.datasets[1].pointBackgroundColor = phColors;
    granHistChart.options.scales.x.min = granHistoryData[0][0];
    granHistChart.options.scales.x.max = granHistoryData[granHistoryData.length - 1][0];
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
  var khChart, phChart, liveChart, granChart, granHistChart, granWinChart, effChart, noiseChart, precisionChart;
  var khErrorBars = [];  // [{x: index, ci: ±dKH}, ...] for per-point error bars
  var khErrorBarsVisible = false;
  var lastSlopeCI = 0;
  var serverSlope = NaN, serverIntercept = NaN, serverSlopeDay0 = 0, serverSlopeNDays = 0;

  // Custom Chart.js plugin: draws vertical error bars on dataset 0 (KH points)
  var errorBarPlugin = {
    id: 'khErrorBars',
    afterDatasetsDraw: function(chart) {
      if (!khErrorBarsVisible || khErrorBars.length === 0) return;
      if (chart !== khChart) return;
      var meta = chart.getDatasetMeta(0);
      if (meta.hidden) return;
      var ctx = chart.ctx;
      var yScale = chart.scales.y;
      ctx.save();
      ctx.strokeStyle = 'rgba(10,132,255,0.5)';
      ctx.lineWidth = 1.5;
      for (var i = 0; i < khErrorBars.length; i++) {
        var eb = khErrorBars[i];
        var pt = meta.data[eb.idx];
        if (!pt) continue;
        var yTop = yScale.getPixelForValue(eb.val + eb.ci);
        var yBot = yScale.getPixelForValue(eb.val - eb.ci);
        var x = pt.x;
        ctx.beginPath();
        ctx.moveTo(x, yTop); ctx.lineTo(x, yBot);
        ctx.stroke();
        // Caps
        ctx.beginPath();
        ctx.moveTo(x - 3, yTop); ctx.lineTo(x + 3, yTop);
        ctx.moveTo(x - 3, yBot); ctx.lineTo(x + 3, yBot);
        ctx.stroke();
      }
      ctx.restore();
    }
  };
  var sgSampleChart, sgTitrateChart;
  var granView = 'last'; // 'last' or 'history'
  var khMethod = 'combined'; // 'combined', 'gran', 'endpoint'
  var khHistoryData = null;  // raw kh history [[ts, val], ...]
  var granHistoryData = null; // raw gran history [[ts, r2, eqML, eph, mth, khG, khE, noiseMv, reversals, conf, khCI], ...]
  var motorHistoryData = null; // raw motor history [[ts, sAvg, sMin, tAvg, tMin], ...]
  var lastSGValues = { sample: 0, titrate: 0 };
  // Time-proportional X-axis: uses linear scale with Unix timestamps (seconds)
  // Returns a fresh object each call so charts don't share scale state
  function timeXScale() {
    return {
      type: 'linear',
      offset: false,
      ticks: { color: '#8e8e93', maxTicksLimit: 6, font: { size: 10 },
        callback: function(val) { return fmtDate(val); }
      },
      grid: { color: '#38383a' }
    };
  }
  var chartOpts = {
    responsive: true,
    maintainAspectRatio: false,
    animation: false,
    plugins: { legend: { display: false }, tooltip: { callbacks: { title: function(items) { if (!items.length) return ''; return fmtDate(items[0].parsed.x); } } } },
    scales: {
      x: timeXScale(),
      y: { ticks: { color: '#8e8e93', font: { size: 10 } }, grid: { color: '#38383a' } }
    }
  };

  function initCharts() {
    khChart = new Chart(document.getElementById('chart-kh'), {
      type: 'scatter',
      plugins: [errorBarPlugin],
      data: { datasets: [
        { label: 'KH', data: [], backgroundColor: '#0a84ff', borderColor: '#0a84ff', borderWidth: 0, pointRadius: 3, pointBackgroundColor: '#0a84ff', pointBorderColor: '#0a84ff', showLine: false, yAxisID: 'y', order: 1 },
        { label: 'Smooth', data: [], borderColor: '#0a84ff', borderWidth: 3, pointRadius: 0, showLine: true, cubicInterpolationMode: 'monotone', tension: 0.4, yAxisID: 'y', order: 2 },
        { label: 'Trend', data: [], borderColor: 'rgba(255,159,10,0.6)', borderWidth: 2, borderDash: [6,3], pointRadius: 0, showLine: true, tension: 0, spanGaps: true, yAxisID: 'y', order: 0 },
        { label: 'Conf', data: [], backgroundColor: 'rgba(48,209,88,0.35)', borderColor: 'rgba(48,209,88,0.8)', borderWidth: 2, pointRadius: 8, pointStyle: 'rect', yAxisID: 'y', order: 3 },
        { label: '_trendUpper', data: [], borderColor: 'rgba(255,159,10,0.2)', borderWidth: 1, pointRadius: 0, showLine: true, fill: 5, backgroundColor: 'rgba(255,159,10,0.08)', spanGaps: true, yAxisID: 'y', order: 6 },
        { label: '_trendLower', data: [], borderColor: 'rgba(255,159,10,0.2)', borderWidth: 1, pointRadius: 0, showLine: true, fill: false, spanGaps: true, yAxisID: 'y', order: 6 }
      ] },
      options: {
        responsive: true, maintainAspectRatio: false, animation: false,
        plugins: { legend: { display: false }, tooltip: { callbacks: { title: function(items) { if (!items.length) return ''; return fmtDate(items[0].parsed.x); } } } },
        scales: {
          x: timeXScale(),
          y: { ticks: { color: '#8e8e93', font: { size: 10 } }, grid: { color: '#38383a' } }
        }
      }
    });
    phChart = new Chart(document.getElementById('chart-ph'), {
      type: 'scatter',
      data: { datasets: [{ data: [], borderColor: '#30d158', borderWidth: 2, pointRadius: 3, pointBackgroundColor: '#30d158', showLine: true, cubicInterpolationMode: 'monotone', tension: 0.4 }] },
      options: chartOpts
    });
    liveChart = new Chart(document.getElementById('chart-live'), {
      type: 'scatter',
      data: { datasets: [{ data: [], showLine: true, borderColor: '#ff9f0a', borderWidth: 2, pointRadius: 0, tension: 0 }] },
      options: {
        responsive: true, maintainAspectRatio: false, animation: false,
        plugins: { legend: { display: false } },
        scales: {
          x: { type: 'linear', offset: false, title: { display: true, text: 'Volume (mL)', color: '#8e8e93' }, ticks: { color: '#8e8e93', font: { size: 10 } }, grid: { color: '#38383a' } },
          y: { title: { display: true, text: 'pH', color: '#8e8e93' }, ticks: { color: '#8e8e93', font: { size: 10 } }, grid: { color: '#38383a' } }
        }
      }
    });
    granChart = new Chart(document.getElementById('chart-gran'), {
      type: 'scatter',
      data: {
        datasets: [
          { label: 'Gran F', data: [], backgroundColor: '#0a84ff', borderColor: '#0a84ff', pointRadius: 3 },
          { label: 'Fit', data: [], borderColor: '#ff9f0a', borderWidth: 2, pointRadius: 0, showLine: true },
          { label: 'Window', data: [], borderColor: 'rgba(48,209,88,0.4)', borderWidth: 1, borderDash: [4,4], pointRadius: 0, showLine: true },
          { label: '_winHigh', data: [], borderColor: 'rgba(48,209,88,0.4)', borderWidth: 1, borderDash: [4,4], pointRadius: 0, showLine: true },
          { label: 'Eq', data: [], backgroundColor: '#ff3b30', borderColor: '#ff3b30', pointRadius: 8, pointStyle: 'crossRot', showLine: false }
        ]
      },
      options: {
        responsive: true, maintainAspectRatio: false, animation: false,
        plugins: { legend: { display: true, labels: { color: '#8e8e93', font: { size: 10 }, boxWidth: 12, filter: function(item) { return item.text && item.text.charAt(0) !== '_'; } } } },
        scales: {
          x: { offset: false, title: { display: true, text: 'Volume (mL)', color: '#8e8e93' }, ticks: { color: '#8e8e93', font: { size: 10 } }, grid: { color: '#38383a' } },
          y: { title: { display: true, text: 'Gran F', color: '#8e8e93' }, ticks: { color: '#8e8e93', font: { size: 10 } }, grid: { color: '#38383a' } }
        }
      }
    });
    granHistChart = new Chart(document.getElementById('chart-gran-hist'), {
      type: 'scatter',
      data: {
        datasets: [
          { label: 'R\u00b2', data: [], borderColor: '#0a84ff', backgroundColor: 'rgba(10,132,255,0.15)', borderWidth: 2, pointRadius: 2, showLine: true, yAxisID: 'yR2', tension: 0.1 },
          { label: 'End pH', data: [], borderColor: '#ff9f0a', backgroundColor: 'rgba(255,159,10,0.15)', borderWidth: 2, pointRadius: 2, showLine: true, yAxisID: 'yRight', tension: 0.1 },
          { label: '\u00b1CI', data: [], borderColor: '#30d158', backgroundColor: 'rgba(48,209,88,0.15)', borderWidth: 2, pointRadius: 2, showLine: true, yAxisID: 'yCI', tension: 0.1 }
        ]
      },
      options: {
        responsive: true, maintainAspectRatio: false, animation: false,
        plugins: { legend: { display: true, labels: { color: '#8e8e93', font: { size: 10 }, boxWidth: 12 } }, tooltip: { callbacks: { title: function(items) { if (!items.length) return ''; return fmtDate(items[0].parsed.x); } } } },
        scales: {
          x: timeXScale(),
          yR2: { type: 'linear', position: 'left', min: 0.995, max: 1.0, ticks: { color: '#0a84ff', font: { size: 9 }, maxTicksLimit: 4 }, grid: { color: '#38383a' }, title: { display: true, text: 'R\u00b2', color: '#0a84ff', font: { size: 9 }, padding: { top: 0, bottom: 0 } } },
          yRight: { type: 'linear', position: 'right', ticks: { color: '#ff9f0a', font: { size: 9 } }, grid: { drawOnChartArea: false }, title: { display: true, text: 'pH', color: '#ff9f0a', font: { size: 10 } } },
          yCI: { type: 'linear', position: 'right', ticks: { color: '#30d158', font: { size: 9 } }, grid: { drawOnChartArea: false }, title: { display: true, text: '\u00b1dKH', color: '#30d158', font: { size: 10 } } }
        }
      }
    });
    granWinChart = new Chart(document.getElementById('chart-gran-windows'), {
      type: 'line',
      data: { labels: [], datasets: [] },
      options: {
        responsive: true, maintainAspectRatio: false, animation: false,
        plugins: { legend: { display: false } },
        scales: {
          x: { type: 'linear', offset: false, title: { display: true, text: 'pH', color: '#8e8e93' }, ticks: { color: '#8e8e93', font: { size: 10 } }, grid: { color: '#38383a' } },
          y: { title: { display: true, text: 'R\u00b2', color: '#8e8e93' }, min: 0.995, max: 1.0, ticks: { color: '#8e8e93', font: { size: 9 } }, grid: { color: '#38383a' } }
        }
      },
      plugins: [{
        id: 'khLabels',
        afterDatasetsDraw: function(chart) {
          var ctx = chart.ctx;
          ctx.font = '9px sans-serif';
          ctx.textAlign = 'center';
          chart.data.datasets.forEach(function(ds, i) {
            if (!ds.khLabel) return;
            var meta = chart.getDatasetMeta(i);
            if (meta.data.length < 2) return;
            var x0 = meta.data[0].x, x1 = meta.data[1].x;
            var y = meta.data[0].y;
            ctx.fillStyle = ds.borderColor;
            ctx.fillText(ds.khLabel, (x0 + x1) / 2, y - 4);
          });
        }
      }]
    });
    var probeChartOpts = function(title, yLabel) {
      return {
        responsive: true, maintainAspectRatio: false, animation: false,
        plugins: {
          legend: { display: false },
          title: { display: true, text: title, color: '#8e8e93', font: { size: 11, weight: '500' }, padding: { bottom: 4 } },
          tooltip: { callbacks: { title: function(items) { if (!items.length) return ''; return fmtDate(items[0].parsed.x); } } }
        },
        scales: {
          x: { type: 'linear', ticks: { color: '#8e8e93', maxTicksLimit: 4, font: { size: 9 }, callback: function(val) { return fmtDate(val); } }, grid: { color: '#38383a' } },
          y: { title: { display: true, text: yLabel, color: '#8e8e93', font: { size: 9 } }, ticks: { color: '#8e8e93', font: { size: 9 } }, grid: { color: '#38383a' } }
        }
      };
    };
    effChart = new Chart(document.getElementById('chart-efficiency'), {
      type: 'scatter',
      data: { datasets: [{ data: [], borderColor: '#30d158', borderWidth: 2, pointRadius: 3, pointBackgroundColor: '#30d158', showLine: true, tension: 0.1 }] },
      options: probeChartOpts('Probe Asymmetry', '%')
    });
    noiseChart = new Chart(document.getElementById('chart-noise'), {
      type: 'scatter',
      data: { datasets: [{ data: [], borderColor: '#30d158', borderWidth: 2, pointRadius: 3, pointBackgroundColor: '#30d158', showLine: true, tension: 0.1 }] },
      options: probeChartOpts('Probe Noise', 'mV')
    });

    var precCanvas = document.getElementById('precision-chart');
    if (precCanvas) {
      precisionChart = new Chart(precCanvas, {
        type: 'bar',
        data: {
          labels: [],
          datasets: [
            { label: 'SD', data: [], backgroundColor: 'rgba(50,130,240,0.7)', borderRadius: 4, barPercentage: 0.6 },
            { label: 'Range', data: [], backgroundColor: 'rgba(50,130,240,0.2)', borderColor: 'rgba(50,130,240,0.5)', borderWidth: 1, borderRadius: 4, barPercentage: 0.6 }
          ]
        },
        options: {
          responsive: true, maintainAspectRatio: false,
          plugins: {
            legend: { display: true, labels: { boxWidth: 12, font: { size: 11 } } },
            title: { display: true, text: 'Precision Test History', font: { size: 13 } },
            tooltip: {
              callbacks: {
                label: function(ctx) {
                  if (ctx.datasetIndex === 0) return 'SD: \u00B1' + ctx.raw.toFixed(3) + ' dKH';
                  return 'Range: ' + ctx.raw.toFixed(2) + ' dKH';
                }
              }
            }
          },
          scales: {
            y: { beginAtZero: true, title: { display: true, text: 'dKH' }, ticks: { font: { size: 10 } } },
            x: { ticks: { font: { size: 10 }, maxRotation: 45 } }
          }
        }
      });
    }
  }

  // --- KH Chart Layer Toggles ---
  function initKHLayers() {
    // Datasets: 0=Points, 1=Smooth, 2=Trend, 3=Conf, 4=TrendUpper, 5=TrendLower
    var map = [
      ['kh-show-points', [0]],
      ['kh-show-smooth', [1]],
      ['kh-show-trend', [2]],
      ['kh-show-ci', [4, 5]]
    ];
    // Restore saved state from localStorage
    var saved = {};
    try { saved = JSON.parse(localStorage.getItem('khLayers') || '{}'); } catch(e) {}
    map.forEach(function(entry) {
      var el = document.getElementById(entry[0]);
      if (!el) return;
      // Apply saved state if available
      if (saved.hasOwnProperty(entry[0])) el.checked = saved[entry[0]];
      // Sync chart datasets to checkbox state
      if (khChart) {
        entry[1].forEach(function(idx) {
          khChart.data.datasets[idx].hidden = !el.checked;
        });
      }
      if (entry[0] === 'kh-show-ci') khErrorBarsVisible = el.checked;
      el.addEventListener('change', function() {
        if (!khChart) return;
        entry[1].forEach(function(idx) {
          khChart.data.datasets[idx].hidden = !el.checked;
        });
        if (entry[0] === 'kh-show-ci') khErrorBarsVisible = el.checked;
        // Persist to localStorage
        var state = {};
        try { state = JSON.parse(localStorage.getItem('khLayers') || '{}'); } catch(e) {}
        state[entry[0]] = el.checked;
        localStorage.setItem('khLayers', JSON.stringify(state));
        khChart.update();
      });
    });
    // Score toggle: re-apply point colors instead of hiding a dataset
    var scoreEl = document.getElementById('kh-show-score');
    if (scoreEl) {
      if (saved.hasOwnProperty('kh-show-score')) scoreEl.checked = saved['kh-show-score'];
      scoreEl.addEventListener('change', function() {
        if (!khChart) return;
        renderKHChart();
        // Persist to localStorage
        var state = {};
        try { state = JSON.parse(localStorage.getItem('khLayers') || '{}'); } catch(e) {}
        state['kh-show-score'] = scoreEl.checked;
        localStorage.setItem('khLayers', JSON.stringify(state));
      });
    }
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
        ['kh','ph','live','gran','gran-hist','gran-windows'].forEach(function(id) {
          var el = document.getElementById('chart-' + id);
          if (el) el.style.display = 'none';
        });
        // Show selected
        if (sel === 'gran') {
          showGranView();
        } else {
          var chartEl = document.getElementById('chart-' + sel);
          if (chartEl) chartEl.style.display = 'block';
        }
        // KH trend info and method toggle
        var khTrend = document.getElementById('kh-trend');
        if (khTrend) khTrend.style.display = (sel === 'kh') ? '' : 'none';
        var khLayers = document.getElementById('kh-layers');
        if (khLayers) khLayers.style.display = (sel === 'kh') ? 'flex' : 'none';
        var khToggle = document.getElementById('kh-method-toggle');
        if (khToggle) khToggle.style.display = (sel === 'kh') ? 'flex' : 'none';
        // Chart range selector visibility (history tabs only)
        var rangeEl = document.getElementById('chart-range');
        if (rangeEl) rangeEl.style.display = (sel === 'kh' || sel === 'ph' || sel === 'gran') ? 'flex' : 'none';
        // Gran sub-tabs visibility
        var granSub = document.getElementById('gran-subtabs');
        if (granSub) granSub.style.display = (sel === 'gran') ? 'flex' : 'none';
        // Gran info only in scatter view
        var granInfo = document.getElementById('gran-info');
        if (granInfo) granInfo.style.display = (sel === 'gran' && granView === 'last' && granInfo.textContent) ? '' : 'none';
        var granQual = document.getElementById('gran-quality');
        if (granQual) granQual.style.display = (sel === 'gran' && granView === 'last') ? '' : 'none';
        // Resize active chart
        if (sel === 'live' && liveChart) liveChart.resize();
        else if (sel === 'kh' && khChart) khChart.resize();
        else if (sel === 'ph' && phChart) phChart.resize();
      });
    });

    // Gran sub-tab toggle
    var btnLast = document.getElementById('gran-tab-last');
    var btnHist = document.getElementById('gran-tab-hist');
    var btnWin = document.getElementById('gran-tab-windows');
    if (btnLast) btnLast.addEventListener('click', function() { switchGranView('last'); });
    if (btnHist) btnHist.addEventListener('click', function() { switchGranView('history'); });
    if (btnWin) btnWin.addEventListener('click', function() { switchGranView('windows'); });

    // Chart range selector
    document.querySelectorAll('.range-btn').forEach(function(btn) {
      btn.addEventListener('click', function() {
        document.querySelectorAll('.range-btn').forEach(function(b) { b.classList.remove('active'); });
        btn.classList.add('active');
        chartDays = parseInt(btn.getAttribute('data-days'));
        if (wsOk) {
          ws.send(JSON.stringify({type:'getHistory', sensor:'kh', days: chartDays}));
          ws.send(JSON.stringify({type:'getHistory', sensor:'ph', days: chartDays}));
          ws.send(JSON.stringify({type:'getHistory', sensor:'gran', days: chartDays}));
        }
      });
    });
  }

  function switchGranView(view) {
    granView = view;
    var btnLast = document.getElementById('gran-tab-last');
    var btnHist = document.getElementById('gran-tab-hist');
    var btnWin = document.getElementById('gran-tab-windows');
    if (btnLast) btnLast.classList.toggle('active', view === 'last');
    if (btnHist) btnHist.classList.toggle('active', view === 'history');
    if (btnWin) btnWin.classList.toggle('active', view === 'windows');
    showGranView();
    var granInfo = document.getElementById('gran-info');
    if (granInfo) granInfo.style.display = (view === 'last' && granInfo.textContent) ? '' : 'none';
    var granQual = document.getElementById('gran-quality');
    if (granQual) granQual.style.display = (view === 'last') ? '' : 'none';
  }

  function showGranView() {
    var scatter = document.getElementById('chart-gran');
    var hist = document.getElementById('chart-gran-hist');
    var winEl = document.getElementById('chart-gran-windows');
    scatter.style.display = 'none';
    hist.style.display = 'none';
    winEl.style.display = 'none';
    if (granView === 'last') {
      scatter.style.display = 'block';
      if (granChart) granChart.resize();
    } else if (granView === 'history') {
      hist.style.display = 'block';
      if (granHistChart) granHistChart.resize();
    } else {
      winEl.style.display = 'block';
      if (granWinChart) granWinChart.resize();
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

  function setMeasuringMode(active) {
    var btnKH = document.querySelector('[data-original-cmd="k"]') || document.querySelector('[data-cmd="k"]');
    var btnPH = document.querySelector('[data-original-cmd="p"]') || document.querySelector('[data-cmd="p"]');
    [btnKH, btnPH].forEach(function(btn) {
      if (!btn) return;
      if (active) {
        btn.setAttribute('data-original-cmd', btn.getAttribute('data-cmd'));
        btn.setAttribute('data-cmd', 'abort');
        btn.textContent = 'Abort';
        btn.classList.remove('primary');
        btn.style.background = 'var(--red)';
        btn.style.color = '#fff';
      } else {
        var orig = btn.getAttribute('data-original-cmd');
        if (orig) {
          btn.setAttribute('data-cmd', orig);
          btn.removeAttribute('data-original-cmd');
        }
        var cmd = btn.getAttribute('data-cmd');
        btn.textContent = (cmd === 'k') ? 'Measure KH' : 'Measure pH';
        btn.classList.add('primary');
        btn.style.background = '';
        btn.style.color = '';
      }
    });
  }

  // --- Precision test result ---
  function checkPrecisionResult(text) {
    if (!text) return;
    var el = document.getElementById('precision-result');
    if (!el) return;
    if (text.indexOf('Precision: ') === 0 || text.indexOf('Precision test') === 0) {
      // Live update — append to status area
      var status = document.getElementById('precision-status');
      if (status) { status.textContent = text; status.style.display = 'block'; }
      // Refresh history after final result
      if (text.indexOf('Precision: ') === 0 && ws && ws.readyState === 1) {
        ws.send(JSON.stringify({type:'getHistory', sensor:'precision'}));
      }
    }
  }

  function renderPrecisionHistory(data) {
    var el = document.getElementById('precision-result');
    var chartEl = document.getElementById('precision-chart');
    if (!el) return;
    if (!data || data.length === 0) {
      el.style.display = 'none';
      if (chartEl) chartEl.style.display = 'none';
      return;
    }
    // data: [[ts, n, mean, sd, min, max, elapsedSec], ...]

    // Update chart
    if (precisionChart) {
      precisionChart.data.labels = data.map(function(r) {
        var d = new Date(r[0] * 1000);
        return d.toLocaleDateString(undefined, {month:'short', day:'numeric'});
      });
      precisionChart.data.datasets[0].data = data.map(function(r) { return r[3]; }); // SD
      precisionChart.data.datasets[1].data = data.map(function(r) { return r[5] - r[4]; }); // range (max-min)
      precisionChart.update();
      if (chartEl) chartEl.style.display = 'block';
    }

    // Update table
    var html = '<table style="width:100%;border-collapse:collapse;font-size:0.85em">';
    html += '<tr style="border-bottom:1px solid var(--border)"><th>Date</th><th>n</th><th>Mean</th><th>SD</th><th>Range</th><th>Duration</th></tr>';
    for (var i = data.length - 1; i >= 0; i--) {
      var r = data[i];
      var d = new Date(r[0] * 1000);
      var date = d.toLocaleDateString(undefined, {month:'short', day:'numeric'}) + ' ' + d.toLocaleTimeString(undefined, {hour:'2-digit', minute:'2-digit'});
      var mins = Math.floor(r[6] / 60);
      var secs = r[6] % 60;
      html += '<tr style="border-bottom:1px solid var(--border)">';
      html += '<td>' + date + '</td>';
      html += '<td style="text-align:center">' + r[1] + '</td>';
      html += '<td style="text-align:right">' + r[2].toFixed(2) + '</td>';
      html += '<td style="text-align:right;font-weight:600">\u00B1' + r[3].toFixed(3) + '</td>';
      html += '<td style="text-align:right">' + r[4].toFixed(2) + '\u2013' + r[5].toFixed(2) + '</td>';
      html += '<td style="text-align:right">' + mins + 'm' + secs + 's</td>';
      html += '</tr>';
    }
    html += '</table>';
    el.innerHTML = html;
    el.style.display = 'block';
  }

  // --- Buttons ---
  function initButtons() {
    document.querySelectorAll('.cmd-btn').forEach(function(btn) {
      btn.addEventListener('click', function() {
        var cmd = btn.getAttribute('data-cmd');
        if (cmd === 'o' && !confirm('Restart the device?')) return;
        if (cmd === 'p') addLogEntry('msg', 'Measuring pH...');
        send({ type: 'cmd', cmd: cmd });
      });
    });
  }

  // --- Config inputs ---
  function showConfigFeedback(el, ok) {
    var fb = el.parentElement && el.parentElement.querySelector('.config-fb');
    if (!fb) {
      fb = document.createElement('span');
      fb.className = 'config-fb';
      fb.style.cssText = 'margin-left:6px;font-size:0.8em;opacity:0;transition:opacity 0.3s';
      el.parentElement.appendChild(fb);
    }
    fb.textContent = ok ? 'Saved' : 'Error';
    fb.style.color = ok ? '#4a4' : '#c44';
    fb.style.opacity = '1';
    setTimeout(function() { fb.style.opacity = '0'; }, 1500);
  }

  // Track initial values for recalibration warnings
  var _initCalDropsRevs = null;
  var _initSampleCalRevs = null;

  function initConfigInputs() {
    document.querySelectorAll('.config-grid input, .cal-ph-col input').forEach(function(inp) {
      if (inp.classList.contains('ui-only')) return;
      inp.addEventListener('change', function() {
        var key = inp.id.replace('cfg-', '');
        if (key === 'mqtt_pass' && inp.value === '********') return;
        var val = (inp.type === 'text' || inp.type === 'password') ? inp.value : parseFloat(inp.value);
        if (typeof val === 'number' && isNaN(val)) return;
        // Convert revolutions back to internal units for titration calibration
        if (key === 'cal_drops') {
          val = Math.round(val * 100);
          if (_initCalDropsRevs !== null && parseFloat(inp.value) !== _initCalDropsRevs) {
            alert('Titration calibration revolutions changed \u2014 please recalibrate the titration pump.');
            _initCalDropsRevs = parseFloat(inp.value);
          }
        }
        if (key === 'sample_cal_revs') {
          if (_initSampleCalRevs !== null && val !== _initSampleCalRevs) {
            alert('Sample calibration revolutions changed \u2014 please recalibrate the sample pump.');
            _initSampleCalRevs = val;
          }
        }
        send({ type: 'config', key: key, value: val });
      });
    });
    document.querySelectorAll('.config-grid select').forEach(function(sel) {
      sel.addEventListener('change', function() {
        var key = sel.id.replace('cfg-', '');
        var val = parseInt(sel.value);
        if (isNaN(val)) return;
        send({ type: 'config', key: key, value: val });
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
    if (intervalMins < 60) intervalMins = 60;
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
    var el = document.getElementById('ind-' + name);
    if (el) { el.className = 'si' + (on ? ' on' : ''); }
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
    if (!ts || ts < 946684800) return '--';
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
  function initKHMethodToggle() {
    document.querySelectorAll('.kh-method-btn').forEach(function(btn) {
      btn.addEventListener('click', function() {
        document.querySelectorAll('.kh-method-btn').forEach(function(b) { b.classList.remove('active'); });
        btn.classList.add('active');
        khMethod = btn.getAttribute('data-method');
        renderKHChart();
      });
    });
  }

  // --- Motor Diagnostics ---
  var lastDiagResult = null;

  function fmtSG(obj) {
    if (!obj) return '--';
    return 'SG ' + obj.sgMin + '-' + obj.sgMax + ' (avg ' + obj.sgAvg + ')';
  }

  function showMotorDiag(d) {
    lastDiagResult = d;
    var prog = document.getElementById('motor-diag-progress');
    var results = document.getElementById('motor-diag-results');
    if (prog) prog.style.display = 'none';
    ['btn-motor-diag', 'btn-motor-diag-sample', 'btn-motor-diag-titrate'].forEach(function(id) {
      var b = document.getElementById(id);
      if (b) b.disabled = false;
    });
    if (!results) return;

    var m = d.mode || 'd';
    var sampleRow = document.getElementById('diag-row-sample');
    var titrateRow = document.getElementById('diag-row-titrate');
    var stallEl = document.getElementById('diag-stall-result');

    if (sampleRow) sampleRow.style.display = (m === 'd' || m === 'B') ? '' : 'none';
    if (titrateRow) titrateRow.style.display = (m === 'd' || m === 'C') ? '' : 'none';

    if ((m === 'd' || m === 'B') && d.sample) {
      setText('diag-sample-sc', fmtSG(d.sample.stealthchop));
      setText('diag-sample-sp', fmtSG(d.sample.spreadcycle));
      setText('diag-sample-rec', d.sample.recommended + ' (SG\u2265' + d.sample.suggestedThreshold + ')');
    }
    if ((m === 'd' || m === 'C') && d.titrate) {
      setText('diag-titrate-sc', fmtSG(d.titrate.stealthchop));
      setText('diag-titrate-sp', fmtSG(d.titrate.spreadcycle));
      setText('diag-titrate-rec', d.titrate.recommended + ' (SG\u2265' + d.titrate.suggestedThreshold + ')');
    }

    if (stallEl) {
      var parts = [];
      if ((m === 'd' || m === 'B') && d.sample) {
        if (d.sample.stallRPM > 0) {
          parts.push('Sample: stall ' + d.sample.stallRPM + ' RPM \u2192 max ' + Math.round(d.sample.maxRPM) + ' RPM');
        } else {
          parts.push('Sample: no stall (30\u2013500 RPM)');
        }
      }
      if ((m === 'd' || m === 'C') && d.titrate) {
        if (d.titrate.stallRPM > 0) {
          parts.push('Titration: stall ' + d.titrate.stallRPM + ' RPM \u2192 max ' + Math.round(d.titrate.maxRPM) + ' RPM');
        } else {
          parts.push('Titration: no stall (30\u2013300 RPM)');
        }
      }
      stallEl.textContent = '';
      parts.forEach(function(p, i) {
        if (i > 0) stallEl.appendChild(document.createElement('br'));
        stallEl.appendChild(document.createTextNode(p));
      });
      stallEl.style.display = parts.length > 0 ? '' : 'none';
    }

    // SG Profile charts
    var profileEl = document.getElementById('diag-sg-profile');
    var profileStats = document.getElementById('sg-profile-stats');
    if (profileEl && d.sgProfile) {
      profileEl.style.display = '';
      var renderProfile = function(canvasId, prof, label) {
        var canvas = document.getElementById(canvasId);
        if (!canvas || !prof || !prof.values || prof.values.length === 0) return;
        var ctx = canvas.getContext('2d');
        // Destroy existing chart if any
        if (canvas._chart) canvas._chart.destroy();
        var data = prof.values.map(function(v, i) { return {x: i + 1, y: v}; });
        canvas._chart = new Chart(ctx, {
          type: 'line',
          data: {
            datasets: [{
              data: data,
              borderColor: '#30d158',
              borderWidth: 1.5,
              pointRadius: 2,
              pointBackgroundColor: '#30d158',
              fill: false
            }]
          },
          options: {
            responsive: true, maintainAspectRatio: false, animation: false,
            plugins: {
              legend: { display: false },
              title: { display: true, text: label, color: '#8e8e93', font: { size: 10 }, padding: { bottom: 2 } }
            },
            scales: {
              x: { type: 'linear', title: { display: true, text: 'Revolution', color: '#8e8e93', font: { size: 8 } }, ticks: { color: '#8e8e93', font: { size: 8 } }, grid: { color: '#38383a' } },
              y: { title: { display: true, text: 'SG', color: '#8e8e93', font: { size: 8 } }, ticks: { color: '#8e8e93', font: { size: 8 } }, grid: { color: '#38383a' },
                min: 0 }
            }
          }
        });
      };
      if (d.sgProfile.sample) renderProfile('sg-profile-sample', d.sgProfile.sample, 'Sample Pump SG Profile');
      if (d.sgProfile.titrate) renderProfile('sg-profile-titrate', d.sgProfile.titrate, 'Titration Pump SG Profile');

      // Stats text
      var stats = [];
      if (d.sgProfile.sample && d.sgProfile.sample.values && d.sgProfile.sample.values.length > 0) {
        stats.push('Sample: min=' + d.sgProfile.sample.min + ' rec. stallSG=' + d.sgProfile.sample.recSG);
      }
      if (d.sgProfile.titrate && d.sgProfile.titrate.values && d.sgProfile.titrate.values.length > 0) {
        stats.push('Titrate: min=' + d.sgProfile.titrate.min + ' rec. stallSG=' + d.sgProfile.titrate.recSG);
      }
      if (profileStats) profileStats.textContent = stats.join(' | ');
    } else if (profileEl) {
      profileEl.style.display = 'none';
    }

    results.style.display = 'block';
    try { localStorage.setItem('motorDiagResult', JSON.stringify(d)); } catch(e) {}
  }

  function initMotorCharts() {
    var sgChartOpts = function(title) {
      return {
        responsive: true, maintainAspectRatio: false, animation: false,
        plugins: {
          legend: { display: false },
          title: { display: true, text: title, color: '#8e8e93', font: { size: 10, weight: '500' }, padding: { bottom: 4 } }
        },
        scales: {
          x: { type: 'linear', ticks: { color: '#8e8e93', maxTicksLimit: 4, font: { size: 9 }, callback: function(val) { return fmtDate(val); } }, grid: { color: '#38383a' } },
          y: { ticks: { color: '#8e8e93', font: { size: 9 } }, grid: { color: '#38383a' }, title: { display: true, text: 'SG', color: '#8e8e93', font: { size: 9 } } }
        }
      };
    };
    var el1 = document.getElementById('chart-sg-sample');
    var el2 = document.getElementById('chart-sg-titrate');
    if (el1) sgSampleChart = new Chart(el1, {
      type: 'scatter',
      data: { datasets: [
        { data: [], borderColor: '#0a84ff', borderWidth: 2, pointRadius: 2, pointBackgroundColor: '#0a84ff', showLine: true, tension: 0.1 },
        { data: [], borderColor: 'rgba(255,159,10,0.5)', borderWidth: 1, borderDash: [4,4], pointRadius: 0, showLine: true }
      ] },
      options: sgChartOpts('Sample Pump SG')
    });
    if (el2) sgTitrateChart = new Chart(el2, {
      type: 'scatter',
      data: { datasets: [
        { data: [], borderColor: '#30d158', borderWidth: 2, pointRadius: 2, pointBackgroundColor: '#30d158', showLine: true, tension: 0.1 },
        { data: [], borderColor: 'rgba(255,159,10,0.5)', borderWidth: 1, borderDash: [4,4], pointRadius: 0, showLine: true }
      ] },
      options: sgChartOpts('Titration Pump SG')
    });
  }

  function renderMotorCharts() {
    if (!motorHistoryData || motorHistoryData.length === 0) return;
    // data: [[ts, sAvg, sMin, tAvg, tMin], ...]
    if (sgSampleChart) {
      sgSampleChart.data.datasets[0].data = motorHistoryData.map(function(p) { return {x: p[0], y: p[1]}; });
      if (motorBaselines.sample > 0) {
        sgSampleChart.data.datasets[1].data = motorHistoryData.map(function(p) { return {x: p[0], y: motorBaselines.sample}; });
      }
      sgSampleChart.options.scales.x.min = motorHistoryData[0][0];
      sgSampleChart.options.scales.x.max = motorHistoryData[motorHistoryData.length - 1][0];
      sgSampleChart.update();
    }
    if (sgTitrateChart) {
      sgTitrateChart.data.datasets[0].data = motorHistoryData.map(function(p) { return {x: p[0], y: p[3]}; });
      if (motorBaselines.titrate > 0) {
        sgTitrateChart.data.datasets[1].data = motorHistoryData.map(function(p) { return {x: p[0], y: motorBaselines.titrate}; });
      }
      sgTitrateChart.options.scales.x.min = motorHistoryData[0][0];
      sgTitrateChart.options.scales.x.max = motorHistoryData[motorHistoryData.length - 1][0];
      sgTitrateChart.update();
    }
  }

  var motorBaselines = { sample: 0, titrate: 0 };

  function initMotorDiag() {
    var diagBtns = ['btn-motor-diag', 'btn-motor-diag-sample', 'btn-motor-diag-titrate'];
    diagBtns.forEach(function(id) {
      var btn = document.getElementById(id);
      if (btn) {
        btn.addEventListener('click', function() {
          var prog = document.getElementById('motor-diag-progress');
          var results = document.getElementById('motor-diag-results');
          if (prog) prog.style.display = 'block';
          if (results) results.style.display = 'none';
          diagBtns.forEach(function(bid) {
            var b = document.getElementById(bid);
            if (b) b.disabled = true;
          });
        });
      }
    });
    var applyBtn = document.getElementById('btn-apply-diag');
    if (applyBtn) {
      applyBtn.addEventListener('click', function() {
        if (!lastDiagResult) return;
        var m = lastDiagResult.mode || 'd';
        var s = lastDiagResult.sample;
        var t = lastDiagResult.titrate;
        if ((m === 'd' || m === 'B') && s) {
          send({ type: 'config', key: 'sample_spreadcycle', value: s.recommended === 'spreadcycle' ? 1 : 0 });
          send({ type: 'config', key: 'sample_stall_sg', value: s.suggestedThreshold });
          var sMode = s.recommended === 'spreadcycle' ? s.spreadcycle : s.stealthchop;
          if (sMode) send({ type: 'config', key: 'sample_sg_baseline', value: sMode.sgAvg });
        }
        if ((m === 'd' || m === 'C') && t) {
          send({ type: 'config', key: 'titrate_spreadcycle', value: t.recommended === 'spreadcycle' ? 1 : 0 });
          send({ type: 'config', key: 'titrate_stall_sg', value: t.suggestedThreshold });
          var tMode = t.recommended === 'spreadcycle' ? t.spreadcycle : t.stealthchop;
          if (tMode) send({ type: 'config', key: 'titrate_sg_baseline', value: tMode.sgAvg });
        }
        addLogEntry('msg', 'Motor diagnostic settings applied');
      });
    }
  }

  function showHWDiagDone() {
    var status = document.getElementById('hw-diag-status');
    var results = document.getElementById('hw-diag-results');
    var btn = document.getElementById('btn-hw-diag');
    if (status) status.style.display = 'none';
    if (results) results.style.display = 'block';
    if (btn) btn.disabled = false;

    // Fetch the report to show a summary
    fetch('/api/hwdiag').then(function(r) { return r.json(); }).then(function(d) {
      var el = document.getElementById('hw-diag-summary');
      if (!el) return;
      var rows = [];
      rows.push('<table style="width:100%;font-size:0.85em;border-collapse:collapse">');
      rows.push('<thead><tr><th style="text-align:left">Test</th><th>Result</th></tr></thead><tbody>');

      var ok = function(v) { return '<span style="color:#30d158">OK</span>'; };
      var warn = function(t) { return '<span style="color:#ff9f0a">' + t + '</span>'; };
      var fail = function(t) { return '<span style="color:#ff453a">' + t + '</span>'; };
      var skip = function() { return '<span style="color:var(--text-secondary)">Skipped</span>'; };

      // System
      rows.push('<tr><td>System</td><td>' + ok() + ' (heap ' + Math.round(d.system.free_heap/1024) + 'kB)</td></tr>');

      // I2C
      if (d.i2c && !d.i2c.skipped) {
        var i2cOk = d.i2c.config_readback_ok && d.i2c.conversions_ok === d.i2c.conversions_total;
        rows.push('<tr><td>I2C / ADS1115</td><td>' + (i2cOk ? ok() : fail('Errors: ' + d.i2c.nak_count + ' NAK')) + '</td></tr>');
      } else {
        rows.push('<tr><td>I2C / ADS1115</td><td>' + skip() + '</td></tr>');
      }

      // EZO pH
      if (d.probe && d.probe.ezo) {
        var ezo = d.probe.ezo;
        var ezoStatus = ezo.test_ok ? ok() + ' (pH ' + ezo.test_ph.toFixed(1) + ', FW ' + ezo.firmware + ')' : fail('test read failed');
        rows.push('<tr><td>EZO pH</td><td>' + ezoStatus + '</td></tr>');
        rows.push('<tr><td>EZO Slope</td><td>Acid ' + ezo.acid_slope_pct.toFixed(1) + '% / Base ' + ezo.base_slope_pct.toFixed(1) + '% (' + ezo.cal_points + ' cal pts)</td></tr>');
      }

      // ADC Noise
      var an = d.adc_noise;
      if (an && an.ph_channel) {
        var noiseMv = an.ph_channel.stddev_mv;
        var nStatus = noiseMv < 0.5 ? ok() : noiseMv < 2.0 ? warn(noiseMv.toFixed(2) + ' mV') : fail(noiseMv.toFixed(2) + ' mV');
        rows.push('<tr><td>ADC Noise (pH)</td><td>' + nStatus + ' stddev</td></tr>');
      }
      if (an && an.noise_ph && an.noise_ph.valid) {
        var np = an.noise_ph;
        var impactColor = np.impact === 'Negligible' || np.impact === 'Low' ? '#30d158' :
                          np.impact === 'Moderate' ? '#ff9f0a' : '#ff453a';
        rows.push('<tr><td>pH Noise</td><td>&plusmn;' + np.stddev_ph.toFixed(2) + ' pH (PTP ' + np.ptp_ph.toFixed(2) + ') — <span style="color:' + impactColor + '">' + np.impact + '</span></td></tr>');
        rows.push('<tr><td>Est. KH Error</td><td>&plusmn;' + np.est_kh_pct.toFixed(1) + '% (Gran avg. over ~15 pts)</td></tr>');
      }
      if (an && an.baseline && !an.baseline.skipped) {
        rows.push('<tr><td>Noise Ratio</td><td>' + (an.noise_ratio ? an.noise_ratio.toFixed(1) + 'x' : '--') + '</td></tr>');
      }

      // Board noise (ESP32 ADC on GPIO36)
      if (d.internal_adc && !d.internal_adc.skipped && d.internal_adc.n_samples > 0) {
        rows.push('<tr><td>Board Noise (GPIO36)</td><td>stddev ' + d.internal_adc.stddev_mv.toFixed(1) + ' mV</td></tr>');
      }

      // Temperature
      if (d.temperature && !d.temperature.skipped) {
        var tErr = d.temperature.crc_errors || d.temperature.power_on_reset;
        rows.push('<tr><td>Temperature</td><td>' + (tErr ? fail('sensor error') : ok() + ' (' + d.temperature.mean_c.toFixed(1) + '&deg;C)') + '</td></tr>');
      } else {
        rows.push('<tr><td>Temperature</td><td>' + skip() + '</td></tr>');
      }

      // TMC
      if (d.tmc_drivers && !d.tmc_drivers.skipped) {
        var tmcOk = d.tmc_drivers.sample.ioin_ok && d.tmc_drivers.titrate.ioin_ok;
        var tmcOT = d.tmc_drivers.sample.overtemp || d.tmc_drivers.titrate.overtemp;
        rows.push('<tr><td>TMC2209</td><td>' + (tmcOT ? fail('overtemp') : tmcOk ? ok() : fail('UART error')) + '</td></tr>');
      } else {
        rows.push('<tr><td>TMC2209</td><td>' + skip() + '</td></tr>');
      }

      // Motors
      if (d.motors && !d.motors.skipped) {
        rows.push('<tr><td>Motors</td><td>' + ok() + '</td></tr>');
      } else {
        rows.push('<tr><td>Motors</td><td>' + skip() + '</td></tr>');
      }

      // GPIO
      if (d.gpio) {
        var gpioOk = true;
        for (var k in d.gpio) { if (!d.gpio[k].ok) gpioOk = false; }
        rows.push('<tr><td>GPIO</td><td>' + (gpioOk ? ok() : warn('unexpected pin states')) + '</td></tr>');
      }

      // Probe
      if (d.probe) {
        if (d.probe.calibrated) {
          rows.push('<tr><td>pH Probe</td><td>' + (d.probe.health === 'Good' ? ok() : d.probe.health === 'Fair' ? warn('Fair') : fail('Replace')) + '</td></tr>');
        } else {
          rows.push('<tr><td>pH Probe</td><td>' + warn('uncalibrated') + '</td></tr>');
        }
      }

      rows.push('</tbody></table>');
      el.innerHTML = rows.join('');
      try { localStorage.setItem('hwDiagDone', '1'); } catch(e) {}
    }).catch(function() {
      var el = document.getElementById('hw-diag-summary');
      if (el) el.innerHTML = '<p style="color:var(--red)">Failed to load diagnostics report</p>';
    });
  }

  function initHWDiag() {
    var btn = document.getElementById('btn-hw-diag');
    if (btn) {
      btn.addEventListener('click', function() {
        var status = document.getElementById('hw-diag-status');
        var results = document.getElementById('hw-diag-results');
        if (status) status.style.display = 'block';
        if (results) results.style.display = 'none';
        btn.disabled = true;
        try { localStorage.removeItem('hwDiagDone'); } catch(e) {}
      });
    }
  }

  function restoreDiagResults() {
    // Restore motor diagnostics
    try {
      var md = localStorage.getItem('motorDiagResult');
      if (md) showMotorDiag(JSON.parse(md));
    } catch(e) {}
    // Restore HW diagnostics
    try {
      if (localStorage.getItem('hwDiagDone')) showHWDiagDone();
    } catch(e) {}
  }

  // --- OTA Firmware Upload ---
  function initOTAUpload() {
    function uploadFile(inputId, type) {
      var input = document.getElementById(inputId);
      if (!input || !input.files.length) return;
      var file = input.files[0];
      var xhr = new XMLHttpRequest();
      var progSection = document.getElementById('ota-progress-section');
      var fillEl = document.getElementById('ota-progress-fill');
      var labelEl = document.getElementById('ota-progress-label');
      var statusEl = document.getElementById('ota-status');

      progSection.style.display = 'block';
      statusEl.textContent = 'Uploading ' + type + '...';
      statusEl.style.color = 'var(--text-secondary)';

      xhr.upload.onprogress = function(e) {
        if (e.lengthComputable) {
          var pct = Math.round(e.loaded / e.total * 100);
          fillEl.style.width = pct + '%';
          labelEl.textContent = pct + '%';
        }
      };
      xhr.onload = function() {
        if (xhr.status === 200) {
          try {
            var resp = JSON.parse(xhr.responseText);
            if (resp.ok) {
              statusEl.textContent = 'Success! Restarting device...';
              statusEl.style.color = 'var(--green)';
              setTimeout(function() { location.reload(); }, 15000);
            } else {
              statusEl.textContent = 'Error: ' + (resp.error || 'Unknown');
              statusEl.style.color = 'var(--red)';
            }
          } catch(e) {
            statusEl.textContent = 'Upload complete';
          }
        } else {
          statusEl.textContent = 'Upload failed (HTTP ' + xhr.status + ')';
          statusEl.style.color = 'var(--red)';
        }
      };
      xhr.onerror = function() {
        statusEl.textContent = 'Upload failed (connection error)';
        statusEl.style.color = 'var(--red)';
      };

      var formData = new FormData();
      formData.append('file', file);
      xhr.open('POST', '/api/update?type=' + type);
      xhr.send(formData);
    }

    var btnFw = document.getElementById('btn-upload-fw');
    if (btnFw) btnFw.addEventListener('click', function() {
      uploadFile('ota-firmware', 'firmware');
    });

    var btnFs = document.getElementById('btn-upload-fs');
    if (btnFs) btnFs.addEventListener('click', function() {
      if (!confirm('Upload filesystem image? This will overwrite the existing filesystem including history data. Export history first!')) return;
      uploadFile('ota-fs', 'filesystem');
    });
  }

  function init() {
    initCharts();
    initKHLayers();
    initTabs();
    initCollapsible();
    initButtons();
    initConfigInputs();
    // Smooth half-life UI-only setting (localStorage)
    var hlInp = document.getElementById('ui-smooth-halflife');
    if (hlInp) {
      var stored = localStorage.getItem('smoothHalfLife');
      if (stored !== null) hlInp.value = stored;
      hlInp.addEventListener('change', function() {
        localStorage.setItem('smoothHalfLife', hlInp.value);
        renderKHChart();
      });
    }
    initSchedule();
    initKHMethodToggle();
    initMotorDiag();
    initMotorCharts();
    initHWDiag();
    restoreDiagResults();
    initOTAUpload();
    connect();
  }

  if (document.readyState === 'loading') {
    document.addEventListener('DOMContentLoaded', init);
  } else {
    init();
  }
})();
