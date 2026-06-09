(function() {
  'use strict';

  var MAX_SCHEDULES = 8;

  // --- WebSocket ---
  var ws, wsOk = false, reconnTimer, reconnDelay = 3000;
  var chartDays = 7;
  var lastMsgTime = 0;

  function connect() {
    var host = location.hostname;
    ws = new WebSocket('ws://' + host + '/ws');
    ws.onopen = function() {
      if (reconnTimer) { clearTimeout(reconnTimer); reconnTimer = null; }
      reconnDelay = 3000;  // Reset backoff on successful connection
      wsOk = true;
      lastMsgTime = Date.now();
      setDot('ws', true);
      // Spark fetches first (always 7d). Order is preserved over WS so the
      // first kh/ph history responses fill sparkKhData/sparkPhData.
      sparkFetchPending.kh = true;
      sparkFetchPending.ph = true;
      ws.send(JSON.stringify({type:'getHistory', sensor:'kh', days: 7}));
      ws.send(JSON.stringify({type:'getHistory', sensor:'ph', days: 7}));
      // Chart fetches at user-selected range (skip if the spark fetch already covers it)
      if (chartDays !== 7) {
        ws.send(JSON.stringify({type:'getHistory', sensor:'kh', days: chartDays}));
        ws.send(JSON.stringify({type:'getHistory', sensor:'ph', days: chartDays}));
      }
      ws.send(JSON.stringify({type:'getHistory', sensor:'gran', days: chartDays}));
      ws.send(JSON.stringify({type:'getHistory', sensor:'precision'}));
    };
    ws.onclose = function() {
      wsOk = false;
      setDot('ws', false);
      if (!reconnTimer) {
        reconnTimer = setTimeout(connect, reconnDelay);
        reconnDelay = Math.min(reconnDelay * 2, 10000);  // Exponential backoff, max 10s (was 60s)
      }
    };
    ws.onerror = function() {
      wsOk = false;
      setDot('ws', false);
      // Force close to trigger onclose -> reconnect (onerror alone may not fire onclose)
      if (ws.readyState !== 3) ws.close();
    };
    ws.onmessage = function(e) {
      lastMsgTime = Date.now();
      var msg;
      try { msg = JSON.parse(e.data); } catch(ex) { console.error('WS JSON parse error:', ex, e.data); return; }
      try { handleMsg(msg); } catch(ex) { console.error('WS handler error:', ex); }
    };
  }

  // Receive watchdog: if no message for 15s (broadcastState fires every 2s), connection is stale
  setInterval(function() {
    if (wsOk && lastMsgTime > 0 && Date.now() - lastMsgTime > 15000) {
      console.log('WS receive watchdog: no message for 15s, reconnecting');
      wsOk = false;
      ws.close();
    }
  }, 5000);

  function send(obj) {
    if (ws && ws.readyState === 1) {
      ws.send(JSON.stringify(obj));
    } else {
      addLogEntry('error', 'Not connected \u2014 command not sent');
    }
  }

  // --- Event log ---
  var LOG_MAX = 40;

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

  // --- Measurement phase + progress ---
  // Mirrors the server-side currentMeasPhase: 0=idle, 1=wash, 2=titrate, 3=cleanup.
  // `isMeasuring` is set as soon as `mesStart` arrives, so updateProgress can
  // route to the live-banner from the very first progress event — without
  // waiting for the next state broadcast that carries `measPhase`.
  var measPhase = 0;
  var measPct = 0;
  var isMeasuring = false;

  // Non-measurement op tracking (calibration / cleaning / precision test).
  // The label comes from the most recent `msg` broadcast.
  var lastOpMessage = '';
  var opActive = false;
  var _bannerHideTimer = null;
  // Phase weights sum to 100; sub-phase pct contributes proportionally
  var PHASE_WEIGHTS = [0, 30, 65, 5];
  var PHASE_LABELS  = ['', 'Washing', 'Titrating', 'Cleaning up'];

  function overallMeasPct() {
    if (measPhase < 1 || measPhase > 3) return 0;
    var cumulative = 0;
    for (var i = 1; i < measPhase; i++) cumulative += PHASE_WEIGHTS[i];
    return Math.round(cumulative + PHASE_WEIGHTS[measPhase] * measPct / 100);
  }

  function refreshLiveBannerProgress() {
    var label = document.getElementById('live-banner-label');
    var fill  = document.getElementById('live-banner-fill');
    var value = document.getElementById('live-banner-value');
    if (label) label.textContent = (PHASE_LABELS[measPhase] || 'Measuring') + '…';
    var overall = overallMeasPct();
    if (fill)  fill.style.width = overall + '%';
    if (value) value.textContent = measPct + '% · ' + overall + '% total';
  }

  // Render non-measurement op progress (calibration, cleaning, etc.) into
  // the same live-banner so the UX is consistent across all operations.
  function showOpProgress(pct) {
    var banner = document.getElementById('live-banner');
    if (!banner) return;
    banner.classList.add('active');
    var label = document.getElementById('live-banner-label');
    var fill  = document.getElementById('live-banner-fill');
    var clamped = Math.max(0, Math.min(100, pct));
    if (label) label.textContent = (lastOpMessage || 'Working') + '…';
    if (fill)  fill.style.width = clamped + '%';
    setText('live-banner-value', clamped + '%');
  }

  // Hide the live-banner after a brief grace period — but only if no other
  // operation has taken over by the time the timer fires.
  function scheduleBannerHide() {
    if (_bannerHideTimer) clearTimeout(_bannerHideTimer);
    _bannerHideTimer = setTimeout(function() {
      _bannerHideTimer = null;
      if (isMeasuring || opActive) return;
      var banner = document.getElementById('live-banner');
      if (!banner) return;
      banner.classList.remove('active');
      var fill = document.getElementById('live-banner-fill');
      if (fill) fill.style.width = '0';
      setText('live-banner-value', '--');
    }, 1200);
  }

  // --- Progress bar (single dispatch — always the live-banner) ---
  function updateProgress(pct) {
    // Cancel any pending hide; we're getting fresh activity.
    if (_bannerHideTimer) { clearTimeout(_bannerHideTimer); _bannerHideTimer = null; }

    if (isMeasuring) {
      // Measurement-phase rendering owns the banner content.
      measPct = pct;
      refreshLiveBannerProgress();
      return;
    }

    if (pct < 100) {
      opActive = true;
      showOpProgress(pct);
    } else {
      opActive = false;
      showOpProgress(100);  // show the final 100% briefly
      scheduleBannerHide();
    }
  }

  // --- State handling ---
  function handleMsg(d) {
    if (d.type === 'state') updateState(d);
    else if (d.type === 'mesPh') updateLivePH(d);
    else if (d.type === 'mesStart') { clearLiveChart(); setMeasuringMode(true); showLiveBanner(true, 'KH'); }
    else if (d.type === 'mesData') { loadMesData(d); setMeasuringMode(false); showLiveBanner(false); }
    else if (d.type === 'history') updateHistory(d);
    else if (d.type === 'msg') {
      addLogEntry('msg', d.text);
      // Use the most recent operation message as the live-banner label
      // for any non-measurement progress events that follow.
      if (d.text) lastOpMessage = d.text.replace(/[…\.]+$/, '').trim();
      checkPrecisionResult(d.text);
      if (motorDiagRunning && d.text.indexOf('Ramp:') === 0) {
        var prog = document.getElementById('motor-diag-progress');
        if (prog) prog.textContent = d.text;
      }
      if (d.text.indexOf('already running') !== -1) resetHWDiagUI();
    }
    else if (d.type === 'error') { addLogEntry('error', d.text); updateProgress(100); setMeasuringMode(false); showLiveBanner(false); }
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
      // Header shows just the device name; footer carries the version.
      setText('device-title', d.deviceName);
      setText('footer-info', d.deviceName + (d.fwVersion ? ' v' + d.fwVersion : ''));
      document.title = d.deviceName;
      // Update HW diagnostics download filename
      var dlLink = document.getElementById('hw-diag-dl');
      if (dlLink) dlLink.download = d.deviceName + '_hw_diagnostics.json';
    }

    // KH gauge (only update when field present — light broadcasts omit NVS-backed values)
    if (d.kh !== undefined) {
      var khVal = (d.kh > 0) ? d.kh : 0;
      setText('val-kh', (d.kh > 0) ? d.kh.toFixed(1) : '--');
      setKhStatusPill(d.kh);
    }

    // KH slope, intercept, and trend line parameters from server
    if (d.khSlope != null) {
      serverSlope = parseFloat(d.khSlope);
      if (d.khIntercept != null) serverIntercept = parseFloat(d.khIntercept);
      if (d.khSlopeT0 != null) serverSlopeT0 = d.khSlopeT0;
      if (d.slopeNPts != null) serverSlopeNPts = d.slopeNPts;
      var st = isNaN(serverSlope) ? '--' : (serverSlope >= 0 ? '+' : '') + serverSlope.toFixed(2);
      setText('val-kh-slope', st);
      setText('val-kh-slope-h', st);
      setKhTrendArrow(serverSlope);
      renderKHChart();      // re-render trend line on main chart
      renderHeroKhSpark();  // ...and on the hero spark
    }
    if (d.predCurve != null) {
      predCurveData = d.predCurve.map(function(p) { return [p[0], parseFloat(p[1])]; });
    }
    if (d.predPoints != null) {
      predPointsData = d.predPoints.map(function(p) { return [p[0], parseFloat(p[1])]; });
    }
    if (d.confidence != null) {
      setText('val-confidence', (d.confidence != null && !isNaN(d.confidence)) ? (d.confidence * 100).toFixed(0) + '%' : '--');
    }

    // pH (start pH from last KH measurement) — primary hero pH value
    if (d.lastStartPh !== undefined) {
      setText('val-ph', (d.lastStartPh > 0) ? d.lastStartPh.toFixed(2) : '--');
      setPhStatusPill(d.lastStartPh);
    }

    // Measured pH (latest pH reading from any source) — secondary metric
    if (d.ph !== undefined) {
      var mesPhVal = (d.ph > 0) ? d.ph : 0;
      setText('val-mesph', mesPhVal > 0 ? mesPhVal.toFixed(2) : '--');
    }

    // HCl tank fill (now horizontal bar; width-based)
    if (d.hclVol !== undefined) {
      var hclMax = 5000;
      var hclPct = Math.max(0, Math.min(100, ((d.hclVol || 0) / hclMax) * 100));
      var fill = document.getElementById('hcl-fill');
      if (fill) fill.style.width = hclPct + '%';
      setText('val-hcl', d.hclVol ? Math.round(d.hclVol) : '--');
      updateAlerts({ hclVol: d.hclVol });
    }

    // Status dots (only update when present)
    if (d.wifiOk !== undefined) setDot('wifi', d.wifiOk);
    if (d.mqttOk !== undefined) setDot('mqtt', d.mqttOk);
    if (d.ntpOk !== undefined) setDot('ntp', d.ntpOk);
    if (d.wifiOk !== undefined || d.mqttOk !== undefined) updateNetworkStatus(d);

    // Measuring state sync — shows/hides Abort button for all clients
    if (d.measuring != null) {
      setMeasuringMode(!!d.measuring);
      if (!!d.measuring) showLiveBanner(true, 'KH');
      else showLiveBanner(false);
    }

    // Phase tracker drives the live-banner progress display
    if (d.measPhase != null && d.measPhase !== measPhase) {
      measPhase = d.measPhase;
      if (measPhase === 0) {
        // Measurement ended — reset sub-progress
        measPct = 0;
      } else {
        // Phase boundary — reset sub-pct (each phase starts at 0%)
        measPct = 0;
        refreshLiveBannerProgress();
      }
    }

    // Status bar
    if (d.temp_sensor !== undefined) {
      var wt = (d.temp_sensor && d.water_temp != null) ? d.water_temp.toFixed(1) + ' \u00B0C' : '--';
      setText('meas-temp-info', wt + ' water');
    }
    if (d.rssi !== undefined) setText('rssi', d.rssi || '--');
    if (d.uptime !== undefined) setText('uptime', fmtUptime(d.uptime || 0));

    // Last crash info (persistent until clean boot)
    if (d.lastCrash !== undefined) {
      var crashEl = document.getElementById('last-crash');
      if (crashEl) {
        if (d.lastCrash) { crashEl.textContent = 'Last crash: ' + d.lastCrash; crashEl.style.display = 'block'; }
        else { crashEl.style.display = 'none'; }
      }
    }

    // Next measurement (server sends formatted string)
    if (d.nextMeas) {
      setText('next-meas', d.nextMeas);
      setText('next-meas-setup', d.nextMeas);
    }

    // Config values
    if (d.config) {
      setInput('cfg-device_name', d.config.device_name);
      setInput('cfg-correction_factor', d.config.correction_factor);
      setInput('cfg-hcl_molarity', d.config.hcl_molarity);
      setInput('cfg-cal_drops', d.config.cal_drops / 100);
      if (_initCalDropsRevs === null) _initCalDropsRevs = d.config.cal_drops / 100;
      setInput('cfg-sample_cal_revs', d.config.sample_cal_revs);
      if (_initSampleCalRevs === null) _initSampleCalRevs = d.config.sample_cal_revs;
      setInput('cfg-fast_ph', d.config.fast_ph);
      setInput('cfg-endpoint_method', d.config.endpoint_method);
      setInput('cfg-min_start_ph', d.config.min_start_ph);
      setInput('cfg-stab_timeout', d.config.stab_timeout);
      setInput('cfg-mix_delay', d.config.mix_delay);
      setInput('cfg-gran_min_r2', d.config.gran_min_r2);
      setInput('cfg-gran_readings', d.config.gran_readings);
      setInput('cfg-kh_ema_alpha', d.config.kh_ema_alpha);
      setInput('cfg-drop_ul', d.config.drop_ul);
      setInput('cfg-gran_burst_rpm', d.config.gran_burst_rpm);
      setInput('cfg-gran_burst_accel', d.config.gran_burst_accel);
      setInput('cfg-fast_phase_rpm', d.config.fast_phase_rpm);
      setInput('cfg-sample_pump_rpm', d.config.sample_pump_rpm);
      setText('sample-cal-current', d.config.sample_cal_vol > 0 ? '(current: ' + d.config.sample_cal_vol.toFixed(1) + ' mL)' : '');
      setText('titration-cal-current', d.config.titration_vol > 0 ? '(current: ' + d.config.titration_vol.toFixed(2) + ' mL)' : '');
      setInput('cfg-fill_brst_n', d.config.fill_brst_n);
      setInput('cfg-fill_pls_n', d.config.fill_pls_n);
      setInput('cfg-max_acid_ml', d.config.max_acid_ml);
      setInput('cfg-fast_step_ul', d.config.fast_step_ul);
      setInput('cfg-meas_temp_c', d.config.meas_temp_c);
      setInput('cfg-buf_ph4', d.config.buf_ph4);
      setInput('cfg-buf_ph7', d.config.buf_ph7);
      setInput('cfg-buf_ph10', d.config.buf_ph10);
      setInput('cfg-slope_hours', d.config.slope_hours);
      setInput('cfg-num_washes', d.config.num_washes);
      setInput('cfg-scavenge', d.config.scavenge);
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

      // Titration pump calibration info
      var titCalInfo = document.getElementById('titration-cal-info');
      if (titCalInfo && d.config.cal_drops && d.config.titration_vol > 0) {
        titCalInfo.textContent = 'Cal factor: ' + ((d.config.cal_drops / 100) / d.config.titration_vol).toFixed(2) + ' revs/mL';
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
    // System tab probe overview cell
    var probeStat = document.getElementById('sys-stat-probe');
    if (probeStat) {
      var h2 = p.health || '--';
      var sCls = (h2 === 'Good') ? 'ok' : (h2 === 'Fair') ? 'warn' : (h2 === 'Replace') ? 'alert' : '';
      probeStat.className = 'ss-state' + (sCls ? ' ' + sCls : '');
      var stText = probeStat.querySelector('span:last-child');
      if (stText) stText.textContent = h2;
    }
    // Surface probe-related issues in alerts strip
    updateAlerts({ probeHealth: p.health, calAge: p.calAge });

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
    // Mirror cal age into the Calibration accordion summary
    var calMeta = document.getElementById('cal-acc-meta');
    if (calMeta && p.calAge != null) {
      calMeta.textContent = p.calAge < 0 ? 'never calibrated' : 'pH cal: ' + p.calAge + 'd ago';
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
    setText('val-mesph', mesPhVal > 0 ? mesPhVal.toFixed(2) : '--');
    // The live-banner shows phase progress during a measurement (refreshLiveBannerProgress
    // owns lb-value); only fall back to pH text outside of an active measurement.
    if (measPhase === 0) {
      setText('live-banner-value', mesPhVal > 0 ? 'pH ' + mesPhVal.toFixed(2) : '--');
    }

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
    if (d.slope !== undefined && d.intercept !== undefined) {
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

  }

  function updateHistory(d) {
    if (!d.data || !d.sensor) return;
    if (d.sensor === 'gran') {
      granHistoryData = d.data;
      updateGranHistChart();
      renderKHChart(); // gran data used for score coloring
      renderNoiseTrend();
      return;
    }
    if (d.sensor === 'precision') {
      renderPrecisionHistory(d.data);
      return;
    }
    if (d.sensor === 'kh') {
      // First kh response after WS open is the dedicated 7d spark fetch.
      if (sparkFetchPending.kh) {
        sparkKhData = d.data;
        sparkFetchPending.kh = false;
        renderHeroKhSpark();
        // If the user's chart range is also 7d, this same data feeds the chart.
        if (chartDays === 7) {
          khHistoryData = d.data;
          renderKHChart();
        }
      } else {
        khHistoryData = d.data;
        renderKHChart();
      }
      return;
    }
    // pH history
    if (d.sensor === 'ph') {
      var phData = d.data;
      if (sparkFetchPending.ph) {
        sparkPhData = phData;
        sparkFetchPending.ph = false;
        renderHeroPhSpark();
        if (chartDays !== 7) return;  // chart will be filled by a later response
      }
      phHistoryData = phData;
      if (phChart) {
        phChart.data.datasets[0].data = phData.map(function(p) { return {x: p[0], y: p[1]}; });
        if (phData.length > 0) {
          phChart.options.scales.x.min = phData[0][0];
          phChart.options.scales.x.max = phData[phData.length - 1][0];
        }
        phChart.update();
      }
      if (phData.length > 0) {
        var lastTs = phData[phData.length - 1][0];
        var ago = Math.max(0, Math.floor((Date.now()/1000 - lastTs)));
        setText('ph-meta', 'last reading ' + fmtAgo(ago));
      }
    }
  }

  function renderKHChart() {
    if (!khChart) return;
    var data = khHistoryData || [];
    if (!data || data.length === 0) {
      khChart.data.labels = [];
      khChart.data.datasets[0].data = [];
      khChart.data.datasets[1].data = [];
      khChart.data.datasets[2].data = [];
      khChart.data.datasets[3].data = [];
      khChart.data.datasets[4].data = [];
      khChart.update();
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

    // Dataset 2: trend line from server regression (slope, intercept, day0)
    // Uses exactly the same slope the device reports — guaranteed match, no knee.
    var slopeHoursEl = document.getElementById('cfg-slope_hours');
    var slopeHours = slopeHoursEl ? (parseInt(slopeHoursEl.value) || 72) : 72;
    var now = data[data.length - 1][0];
    var cutoff = now - slopeHours * 3600;
    var trendOk = false;
    if (!isNaN(serverSlope) && !isNaN(serverIntercept) && serverSlopeT0 > 0) {
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
        var xFirst = (data[firstIdx][0] - serverSlopeT0) / 86400;
        var xLast = (data[lastIdx][0] - serverSlopeT0) / 86400;
        khChart.data.datasets[2].data = [
          {x: data[firstIdx][0], y: serverSlope * xFirst + serverIntercept},
          {x: data[lastIdx][0], y: serverSlope * xLast + serverIntercept}
        ];
        trendOk = true;
      }
    }
    if (!trendOk) {
      khChart.data.datasets[2].data = [];
    }

    // Dataset 3+4: prediction forecast curve + scheduled measurement points
    if (predCurveData.length > 0) {
      khChart.data.datasets[3].data = predCurveData.map(function(p) { return {x: p[0], y: p[1]}; });
    } else {
      khChart.data.datasets[3].data = [];
    }
    if (predPointsData.length > 0) {
      khChart.data.datasets[4].data = predPointsData.map(function(p) { return {x: p[0], y: p[1]}; });
    } else {
      khChart.data.datasets[4].data = [];
    }

    // Enforce minimum 1.5 dKH span on y-axis (include forecast points)
    var vals = khChart.data.datasets[0].data.map(function(p) { return p.y; }).filter(function(v) { return v != null; });
    var forecastVals = khChart.data.datasets[4].data.map(function(p) { return p.y; }).filter(function(v) { return v != null; });
    vals = vals.concat(forecastVals);
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
    var xMax = data[data.length - 1][0];
    if (predCurveData.length > 0) xMax = Math.max(xMax, predCurveData[predCurveData.length - 1][0]);
    khChart.options.scales.x.max = xMax;
    khChart.update();
  }

  function updateGranHistChart() {
    if (!granHistChart || !granHistoryData || granHistoryData.length === 0) return;
    // data: [[ts, r2, eqML, endpointPH, method, khGran, khEndpoint, noiseMv, reversals, conf, khCI], ...]
    granHistChart.data.datasets[0].data = granHistoryData.map(function(p) { return {x: p[0], y: p[1]}; }); // R2
    granHistChart.data.datasets[1].data = granHistoryData.map(function(p) { return {x: p[0], y: p[3]}; }); // endpointPH
    // Color interpolation points red
    var r2Colors = granHistoryData.map(function(p) { return p[4] === 1 ? '#0a84ff' : '#ff453a'; });
    var phColors = granHistoryData.map(function(p) { return p[4] === 1 ? '#ff9f0a' : '#ff453a'; });
    granHistChart.data.datasets[0].pointBackgroundColor = r2Colors;
    granHistChart.data.datasets[1].pointBackgroundColor = phColors;
    granHistChart.options.scales.x.min = granHistoryData[0][0];
    granHistChart.options.scales.x.max = granHistoryData[granHistoryData.length - 1][0];
    granHistChart.update();
  }

  // --- Charts ---
  var khChart, phChart, liveChart, granChart, granHistChart, effChart, noiseChart, precisionChart;
  var sparkKhChart, sparkPhChart;     // hero-card mini charts
  var phHistoryData = null;            // last fetched pH history (matches chart range)
  var sparkKhData = null;              // dedicated 7-day window for hero KH spark
  var sparkPhData = null;              // dedicated 7-day window for hero pH spark
  var sparkFetchPending = { kh: false, ph: false };
  var serverSlope = NaN, serverIntercept = NaN, serverSlopeT0 = 0, serverSlopeNPts = 0;
  var predCurveData = [];    // prediction curve [[ts, kh], ...]
  var predPointsData = [];   // predicted measurement points [[ts, kh], ...]
  var khHistoryData = null;  // raw kh history [[ts, val], ...]
  var granHistoryData = null; // raw gran history [[ts, r2, eqML, eph, mth, khG, khE, noiseMv, reversals, conf, khCI], ...]
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
      data: { datasets: [
        { label: 'KH', data: [], backgroundColor: '#0a84ff', borderColor: '#0a84ff', borderWidth: 0, pointRadius: 3, pointBackgroundColor: '#0a84ff', pointBorderColor: '#0a84ff', showLine: false, yAxisID: 'y', order: 1 },
        { label: 'Smooth', data: [], borderColor: '#0a84ff', borderWidth: 3, pointRadius: 0, showLine: true, cubicInterpolationMode: 'monotone', tension: 0.4, yAxisID: 'y', order: 2 },
        { label: 'Trend', data: [], borderColor: 'rgba(255,159,10,0.6)', borderWidth: 2, borderDash: [6,3], pointRadius: 0, showLine: true, tension: 0, spanGaps: true, yAxisID: 'y', order: 0 },
        { label: 'Forecast', data: [], borderColor: 'rgba(10,132,255,0.35)', borderWidth: 2, borderDash: [4,4], pointRadius: 0, showLine: true, cubicInterpolationMode: 'monotone', tension: 0.4, yAxisID: 'y', order: 2 },
        { label: 'ForecastPts', data: [], backgroundColor: 'rgba(10,132,255,0.3)', borderColor: 'rgba(10,132,255,0.3)', borderWidth: 0, pointRadius: 4, showLine: false, yAxisID: 'y', order: 1 }
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
          { label: 'End pH', data: [], borderColor: '#ff9f0a', backgroundColor: 'rgba(255,159,10,0.15)', borderWidth: 2, pointRadius: 2, showLine: true, yAxisID: 'yRight', tension: 0.1 }
        ]
      },
      options: {
        responsive: true, maintainAspectRatio: false, animation: false,
        plugins: { legend: { display: true, labels: { color: '#8e8e93', font: { size: 10 }, boxWidth: 12 } }, tooltip: { callbacks: { title: function(items) { if (!items.length) return ''; return fmtDate(items[0].parsed.x); } } } },
        scales: {
          x: timeXScale(),
          yR2: { type: 'linear', position: 'left', min: 0.995, max: 1.0, ticks: { color: '#0a84ff', font: { size: 9 }, maxTicksLimit: 4 }, grid: { color: '#38383a' }, title: { display: true, text: 'R\u00b2', color: '#0a84ff', font: { size: 9 }, padding: { top: 0, bottom: 0 } } },
          yRight: { type: 'linear', position: 'right', ticks: { color: '#ff9f0a', font: { size: 9 } }, grid: { drawOnChartArea: false }, title: { display: true, text: 'pH', color: '#ff9f0a', font: { size: 10 } } }
        }
      }
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
    // Datasets: 0=Points, 1=Smooth, 2=Trend, 3=Forecast line, 4=Forecast points
    var map = [
      ['kh-show-points', [0]],
      ['kh-show-smooth', [1]],
      ['kh-show-trend', [2]],
      ['kh-show-forecast', [3, 4]]
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
      el.addEventListener('change', function() {
        if (!khChart) return;
        entry[1].forEach(function(idx) {
          khChart.data.datasets[idx].hidden = !el.checked;
        });
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

  // --- History mode tabs (KH / pH / Gran / Live) ---
  function initTabs() {
    var tabs = document.querySelectorAll('.seg .tab[data-tab]');
    tabs.forEach(function(t) {
      t.addEventListener('click', function() {
        tabs.forEach(function(tt) { tt.classList.remove('active'); });
        t.classList.add('active');
        applyHistoryMode(t.getAttribute('data-tab'));
      });
    });

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

    // Gran "Last titration curve" details — resize chart when opened
    var granDetails = document.getElementById('gran-details');
    if (granDetails) {
      granDetails.addEventListener('toggle', function() {
        if (granDetails.open && granChart) setTimeout(function() { granChart.resize(); }, 0);
      });
    }
  }

  function applyHistoryMode(sel) {
    // Hide all chart canvases inside the main chart container
    ['kh','ph','live','gran-hist'].forEach(function(id) {
      var el = document.getElementById('chart-' + id);
      if (el) el.style.display = 'none';
    });
    // Show the canvas for the selected mode
    var canvasId = (sel === 'gran') ? 'chart-gran-hist' : 'chart-' + sel;
    var canvas = document.getElementById(canvasId);
    if (canvas) canvas.style.display = 'block';

    // Toolbar visibility
    var khTrend = document.getElementById('kh-trend');
    if (khTrend) khTrend.style.display = (sel === 'kh') ? '' : 'none';
    var khLayers = document.getElementById('kh-layers');
    if (khLayers) khLayers.style.display = (sel === 'kh') ? 'flex' : 'none';
    var rangeEl = document.getElementById('chart-range');
    if (rangeEl) rangeEl.style.display = (sel === 'live') ? 'none' : 'flex';

    // Gran-only widgets
    var granInfo = document.getElementById('gran-info');
    if (granInfo) granInfo.style.display = (sel === 'gran' && granInfo.textContent) ? '' : 'none';
    var granQual = document.getElementById('gran-quality');
    if (granQual) granQual.style.display = (sel === 'gran') ? '' : 'none';
    var granDetails = document.getElementById('gran-details');
    if (granDetails) granDetails.style.display = (sel === 'gran') ? '' : 'none';

    // Resize the active chart
    if (sel === 'live' && liveChart) liveChart.resize();
    else if (sel === 'kh' && khChart) khChart.resize();
    else if (sel === 'ph' && phChart) phChart.resize();
    else if (sel === 'gran' && granHistChart) granHistChart.resize();
  }

  // Only the KH button toggles to "Abort" during a measurement. pH measurements
  // are short and don't need an abort affordance, so the pH button is left alone.
  function setMeasuringMode(active) {
    isMeasuring = !!active;
    if (!active) measPhase = 0;  // reset so updateProgress falls back to non-measurement path
    var btnKH = document.querySelector('[data-original-cmd="k"]') || document.querySelector('[data-cmd="k"]');
    if (!btnKH) return;
    if (active) {
      if (btnKH.getAttribute('data-cmd') === 'abort') return;  // already in abort mode — don't re-clobber data-original-cmd on repeat broadcasts
      btnKH.setAttribute('data-original-cmd', btnKH.getAttribute('data-cmd'));
      btnKH.setAttribute('data-cmd', 'abort');
      btnKH.textContent = 'Abort';
      btnKH.style.background = 'var(--red)';
      btnKH.style.color = '#fff';
    } else {
      var orig = btnKH.getAttribute('data-original-cmd');
      if (orig) {
        btnKH.setAttribute('data-cmd', orig);
        btnKH.removeAttribute('data-original-cmd');
      }
      btnKH.textContent = 'Measure';
      btnKH.style.background = '';
      btnKH.style.color = '';
    }
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
      if (!btn.getAttribute('data-cmd')) return; // skip non-command buttons
      btn.addEventListener('click', function() {
        var cmd = btn.getAttribute('data-cmd');
        if (!cmd) return;
        if (cmd === 'o' && !confirm('Restart the device?')) return;
        if (cmd === 'p') addLogEntry('msg', 'Measuring pH...');
        send({ type: 'cmd', cmd: cmd });
        // Brief visual flash on press
        btn.style.opacity = '0.6';
        setTimeout(function() { btn.style.opacity = ''; }, 200);
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

  // --- Motor Diagnostics ---
  var motorDiagRunning = null; // which button ID is running, or null

  function showMotorDiag(d) {
    var prog = document.getElementById('motor-diag-progress');
    var results = document.getElementById('motor-diag-results');
    if (prog) prog.style.display = 'none';

    // Restore all buttons
    ['btn-motor-diag-sample', 'btn-motor-diag-titrate'].forEach(function(id) {
      var b = document.getElementById(id);
      if (b) { b.disabled = false; b.textContent = b.getAttribute('data-label'); b.style.background = ''; }
    });
    motorDiagRunning = null;

    if (!results) return;
    var lines = [];
    if (d.sample) {
      lines.push('Sample: ' + (d.sample.aborted ? 'aborted at ' : 'OK up to ') + Math.round(d.sample.maxRPM) + ' RPM');
    }
    if (d.titrate) {
      lines.push('Titration: ' + (d.titrate.aborted ? 'aborted at ' : 'OK up to ') + Math.round(d.titrate.maxRPM) + ' RPM');
    }
    results.textContent = lines.join(' | ');
    results.style.display = 'block';
    try { localStorage.setItem('motorDiagResult', JSON.stringify(d)); } catch(e) {}
  }

  function initMotorDiag() {
    var diagBtns = ['btn-motor-diag-sample', 'btn-motor-diag-titrate'];
    diagBtns.forEach(function(id) {
      var btn = document.getElementById(id);
      if (!btn) return;
      btn.setAttribute('data-label', btn.textContent);
      btn.addEventListener('click', function() {
        if (motorDiagRunning) {
          // Abort
          send({ type: 'cmd', cmd: 'abort' });
          return;
        }
        var prog = document.getElementById('motor-diag-progress');
        var results = document.getElementById('motor-diag-results');
        if (prog) { prog.textContent = 'Starting ramp test...'; prog.style.display = 'block'; }
        if (results) results.style.display = 'none';
        motorDiagRunning = id;
        // Disable other buttons, turn this one into Abort
        diagBtns.forEach(function(bid) {
          var b = document.getElementById(bid);
          if (!b) return;
          if (bid === id) {
            b.textContent = 'Abort';
            b.style.background = 'var(--red, #ff453a)';
          } else {
            b.disabled = true;
          }
        });
      });
    });
  }

  function showHWDiagDone() {
    if (hwDiagTimer) { clearTimeout(hwDiagTimer); hwDiagTimer = null; }
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
        var ezoStatus = ezo.test_ok ? ok() + ' (pH ' + ezo.test_ph.toFixed(1) + ', FW ' + escHtml(ezo.firmware) + ')' : fail('test read failed');
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

  var hwDiagTimer = null;

  function resetHWDiagUI() {
    var btn = document.getElementById('btn-hw-diag');
    var status = document.getElementById('hw-diag-status');
    if (btn) btn.disabled = false;
    if (status) status.style.display = 'none';
    if (hwDiagTimer) { clearTimeout(hwDiagTimer); hwDiagTimer = null; }
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
        // Safety timeout: re-enable after 3 minutes if no response
        if (hwDiagTimer) clearTimeout(hwDiagTimer);
        hwDiagTimer = setTimeout(resetHWDiagUI, 180000);
      });
    }
  }

  function restoreDiagResults() {
    // Restore motor diagnostics
    try {
      var md = localStorage.getItem('motorDiagResult');
      if (md) {
        var parsed = JSON.parse(md);
        if (parsed.sample || parsed.titrate) showMotorDiag(parsed);
      }
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
      var statusEl = document.getElementById('ota-status');
      // Basic file validation
      var name = file.name.toLowerCase();
      if (type === 'firmware' && !name.endsWith('.bin')) {
        statusEl.textContent = 'Error: Please select a .bin firmware file';
        statusEl.style.color = 'var(--red)';
        return;
      }
      if (file.size > 1900000) {
        if (!confirm('File is ' + Math.round(file.size / 1024) + ' KB — larger than expected. Continue?')) return;
      }
      var xhr = new XMLHttpRequest();
      var progSection = document.getElementById('ota-progress-section');
      var fillEl = document.getElementById('ota-progress-fill');
      var labelEl = document.getElementById('ota-progress-label');

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

  // ============================================================
  // ===== Page navigation (Overview / History / Setup / System) =====
  // ============================================================
  function initPageNav() {
    var tabs = document.querySelectorAll('.page-tab[data-page]');
    function activate(page) {
      tabs.forEach(function(t) { t.classList.toggle('active', t.getAttribute('data-page') === page); });
      document.querySelectorAll('.page-section').forEach(function(s) {
        s.classList.toggle('active', s.getAttribute('data-page') === page);
      });
      // Resize charts on the active page (Chart.js needs this when canvases were display:none)
      if (page === 'history') {
        var activeTab = document.querySelector('.seg .tab.active');
        if (activeTab) applyHistoryMode(activeTab.getAttribute('data-tab'));
      } else if (page === 'system') {
        if (effChart) effChart.resize();
        if (noiseChart) noiseChart.resize();
        if (precisionChart) precisionChart.resize();
      } else if (page === 'overview') {
        if (sparkKhChart) sparkKhChart.resize();
        if (sparkPhChart) sparkPhChart.resize();
      }
    }
    tabs.forEach(function(t) {
      t.addEventListener('click', function() {
        var page = t.getAttribute('data-page');
        activate(page);
        try { history.replaceState(null, '', '#' + page); } catch(e) {}
      });
    });
    // Initial page from hash, or default to overview
    var initial = (location.hash || '').replace('#', '');
    if (!initial || !document.querySelector('[data-page="' + initial + '"]')) initial = 'overview';
    activate(initial);
    window.addEventListener('hashchange', function() {
      var p = (location.hash || '').replace('#', '') || 'overview';
      if (document.querySelector('[data-page="' + p + '"]')) activate(p);
    });

    // Tap the time in the "Next measurement" card → Setup → Schedule accordion.
    // Card itself is no longer the link (it now hosts a Measure button at the bottom);
    // only the time text is clickable.
    var nextMeasLink = document.getElementById('next-meas-link');
    function openSchedule(e) {
      if (e) e.preventDefault();
      activate('setup');
      try { history.replaceState(null, '', '#setup'); } catch(err) {}
      var schedAcc = document.querySelector('[data-acc="sched"]');
      if (schedAcc) schedAcc.open = true;
      if (schedAcc && schedAcc.scrollIntoView) schedAcc.scrollIntoView({behavior: 'smooth', block: 'start'});
    }
    if (nextMeasLink) {
      nextMeasLink.addEventListener('click', openSchedule);
    }

    initHclEdit();
  }

  // Tap HCl card → inline edit the volume (replaces #val-hcl span with a number input).
  // Skips if the tap landed on a button (Fill) so the action button keeps its own behavior.
  function initHclEdit() {
    var card = document.getElementById('hcl-card');
    if (!card) return;
    var editing = false;

    function startEdit() {
      if (editing) return;
      var span = document.getElementById('val-hcl');
      if (!span) return;
      editing = true;
      var current = parseFloat(span.textContent.replace(/[^\d.\-]/g, '')) || 0;
      var input = document.createElement('input');
      input.type = 'number';
      input.inputMode = 'decimal';
      input.min = '0';
      input.max = '20000';
      input.step = '1';
      input.value = current;
      input.id = 'val-hcl';  // preserve id so subsequent state updates still find it (after restore)
      input.style.cssText = 'width:5.5em;font:inherit;color:inherit;background:var(--card-elevated);' +
        'border:1px solid var(--accent);border-radius:6px;padding:2px 6px;text-align:right;';
      span.parentNode.replaceChild(input, span);
      input.focus();
      input.select();

      function finish(save) {
        if (!editing) return;
        editing = false;
        var newSpan = document.createElement('span');
        newSpan.id = 'val-hcl';
        var val = parseFloat(input.value);
        if (save && !isNaN(val) && val >= 0 && val <= 20000) {
          newSpan.textContent = Math.round(val);
          send({ type: 'config', key: 'hcl_volume', value: val });
        } else {
          newSpan.textContent = Math.round(current);  // revert
        }
        input.parentNode.replaceChild(newSpan, input);
      }

      input.addEventListener('blur', function() { finish(true); });
      input.addEventListener('keydown', function(e) {
        if (e.key === 'Enter')   { e.preventDefault(); finish(true); }
        else if (e.key === 'Escape') { e.preventDefault(); finish(false); }
      });
    }

    card.addEventListener('click', function(e) {
      // Don't start edit when the user tapped the Fill button or its descendants.
      if (e.target.closest('.cmd-btn')) return;
      // Don't restart edit if we're already inside the input.
      if (e.target.tagName === 'INPUT') return;
      startEdit();
    });
    card.addEventListener('keydown', function(e) {
      if (editing) return;
      if (e.target !== card) return;  // only when card itself is focused, not the inner button
      if (e.key === 'Enter' || e.key === ' ') { e.preventDefault(); startEdit(); }
    });
  }

  // ============================================================
  // ===== Live measurement banner =====
  // ============================================================
  function showLiveBanner(active, kind) {
    var banner = document.getElementById('live-banner');
    if (!banner) return;
    if (active) {
      // Once a phase is known, refreshLiveBannerProgress owns the label so it
      // doesn't flip-flop with every state broadcast. Use the generic
      // "Measuring KH…" only as a placeholder before the first phase arrives.
      if (measPhase > 0) {
        refreshLiveBannerProgress();
      } else {
        var label = document.getElementById('live-banner-label');
        if (label) label.textContent = 'Measuring ' + (kind || 'KH') + '…';
      }
      banner.classList.add('active');
    } else {
      // Share the same hide-with-grace path as non-measurement ops, so an
      // operation that starts immediately after a measurement ends doesn't
      // get its banner yanked out from under it by the trailing timer.
      scheduleBannerHide();
    }
  }
  function initLiveBanner() {
    var banner = document.getElementById('live-banner');
    if (!banner) return;
    banner.addEventListener('click', function() {
      // Switch to History → Live
      var liveTab = document.querySelector('.seg .tab[data-tab="live"]');
      var pageBtn = document.querySelector('.page-tab[data-page="history"]');
      if (pageBtn) pageBtn.click();
      if (liveTab) liveTab.click();
    });
  }

  // ============================================================
  // ===== Hero sparks (small charts on Overview) =====
  // ============================================================
  function initHeroSparks() {
    var sparkOpts = {
      responsive: true, maintainAspectRatio: false, animation: false,
      plugins: { legend: { display: false }, tooltip: { enabled: false } },
      scales: {
        x: { type: 'linear', display: false, offset: false },
        y: { display: false, offset: false }
      },
      elements: { point: { radius: 0 } }
    };
    var khEl = document.getElementById('spark-kh');
    if (khEl) {
      sparkKhChart = new Chart(khEl, {
        type: 'line',
        data: { datasets: [
          { data: [], borderColor: 'rgba(10,132,255,0.95)', borderWidth: 2, showLine: true, cubicInterpolationMode: 'monotone', tension: 0.4, fill: false },
          { data: [], borderColor: 'rgba(255,159,10,0.6)', borderDash: [4,3], borderWidth: 1.5, showLine: true, fill: false }
        ] },
        options: sparkOpts
      });
    }
    var phEl = document.getElementById('spark-ph');
    if (phEl) {
      sparkPhChart = new Chart(phEl, {
        type: 'line',
        data: { datasets: [
          { data: [], borderColor: 'rgba(48,209,88,0.95)', borderWidth: 2, showLine: true, cubicInterpolationMode: 'monotone', tension: 0.4, fill: false }
        ] },
        options: sparkOpts
      });
    }
  }
  function smoothSeries(data) {
    if (!data || data.length < 2) return [];
    // Match the History KH chart: honor user's smooth-halflife override, else auto.
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
      var dtH2 = (data[i+1][0] - data[i][0]) / 3600;
      var alpha2 = 1 - Math.pow(0.5, dtH2 / halfLife);
      bwd[i] = alpha2 * data[i][1] + (1 - alpha2) * bwd[i+1];
    }
    var out = [];
    for (var i = 0; i < data.length; i++) {
      out.push({ x: data[i][0], y: (fwd[i] + bwd[i]) / 2 });
    }
    return out;
  }
  function lastNDays(data, days) {
    if (!data || data.length === 0) return data || [];
    var cutoff = data[data.length - 1][0] - days * 86400;
    var out = [];
    for (var i = 0; i < data.length; i++) {
      if (data[i][0] >= cutoff) out.push(data[i]);
    }
    return out;
  }
  function setSparkRange(minId, maxId, smoothed, decimals) {
    if (!smoothed || smoothed.length === 0) return;
    var ys = smoothed.map(function(p) { return p.y; });
    var mn = Math.min.apply(null, ys);
    var mx = Math.max.apply(null, ys);
    setText(minId, mn.toFixed(decimals));
    setText(maxId, mx.toFixed(decimals));
  }
  function renderHeroKhSpark() {
    if (!sparkKhChart || !sparkKhData || sparkKhData.length < 2) return;
    var window = lastNDays(sparkKhData, 7);
    if (window.length < 2) window = sparkKhData;
    var smoothed = smoothSeries(window);
    sparkKhChart.data.datasets[0].data = smoothed;
    // Trend line — same server slope/intercept as the main chart
    var trend = [];
    if (!isNaN(serverSlope) && !isNaN(serverIntercept) && serverSlopeT0 > 0) {
      var first = window[0][0];
      var last = window[window.length - 1][0];
      trend = [
        { x: first, y: serverSlope * ((first - serverSlopeT0) / 86400) + serverIntercept },
        { x: last,  y: serverSlope * ((last  - serverSlopeT0) / 86400) + serverIntercept }
      ];
    }
    sparkKhChart.data.datasets[1].data = trend;
    sparkKhChart.update('none');
    setSparkRange('spark-kh-min', 'spark-kh-max', smoothed, 1);
  }
  function renderHeroPhSpark() {
    if (!sparkPhChart || !sparkPhData || sparkPhData.length < 2) return;
    var window = lastNDays(sparkPhData, 7);
    if (window.length < 2) window = sparkPhData;
    var smoothed = smoothSeries(window);
    sparkPhChart.data.datasets[0].data = smoothed;
    sparkPhChart.update('none');
    setSparkRange('spark-ph-min', 'spark-ph-max', smoothed, 2);
  }

  // ============================================================
  // ===== Status pills, trend arrow, network status, alerts =====
  // ============================================================
  function setKhStatusPill(kh) {
    var p = document.getElementById('kh-status-pill');
    if (!p) return;
    p.classList.remove('ok','warn','alert');
    if (!kh || kh <= 0) { p.querySelector('span:not(.dot)') ? null : null; p.innerHTML = '<span class="dot"></span>--'; return; }
    var cls = (kh >= 7 && kh <= 11) ? 'ok' : (kh >= 6 && kh <= 13) ? 'warn' : 'alert';
    var label = (cls === 'ok') ? 'in range' : (cls === 'warn') ? 'check' : 'out of range';
    p.classList.add(cls);
    p.innerHTML = '<span class="dot"></span>' + label;
  }
  function setPhStatusPill(ph) {
    var p = document.getElementById('ph-status-pill');
    if (!p) return;
    p.classList.remove('ok','warn','alert');
    if (!ph || ph <= 0) { p.innerHTML = '<span class="dot"></span>--'; return; }
    var cls = (ph >= 7.8 && ph <= 8.5) ? 'ok' : (ph >= 7.4 && ph <= 8.7) ? 'warn' : 'alert';
    var label = (cls === 'ok') ? 'in range' : (cls === 'warn') ? 'check' : 'out of range';
    p.classList.add(cls);
    p.innerHTML = '<span class="dot"></span>' + label;
  }
  function setKhTrendArrow(slope) {
    var meta = document.getElementById('kh-trend-meta');
    if (!meta) return;
    var arrow = '—'; // em-dash
    if (!isNaN(slope) && Math.abs(slope) > 0.005) arrow = (slope > 0) ? '↑' : '↓';
    meta.firstChild && (meta.firstChild.nodeValue = arrow + ' ');
  }
  function updateNetworkStatus(d) {
    var el = document.getElementById('sys-stat-network');
    if (!el) return;
    el.classList.remove('ok','warn','alert');
    var stText = el.querySelector('span:last-child');
    var both = (d.wifiOk && d.mqttOk);
    var some = (d.wifiOk || d.mqttOk);
    var cls = both ? 'ok' : some ? 'warn' : 'alert';
    var label = both ? 'OK' : some ? (d.wifiOk ? 'WiFi only' : 'MQTT only') : 'Offline';
    el.classList.add(cls);
    if (stText) stText.textContent = label;
  }
  // Alerts strip — accumulate from various state updates, dedup by key
  var _alertState = { hcl: false, probe: false, calOld: false };
  function updateAlerts(d) {
    var alerts = document.getElementById('alerts');
    if (!alerts) return;
    if (d.hclVol !== undefined) _alertState.hcl = (d.hclVol >= 0 && d.hclVol < 200);
    if (d.probeHealth !== undefined) _alertState.probe = (d.probeHealth === 'Replace');
    if (d.calAge !== undefined) _alertState.calOld = (d.calAge >= 30);
    var html = '';
    if (_alertState.hcl) html += '<div class="alert alert-red"><span class="a-dot"></span><span class="a-text">HCl tank low — refill soon</span></div>';
    if (_alertState.probe) html += '<div class="alert alert-red"><span class="a-dot"></span><span class="a-text">pH probe needs replacement</span></div>';
    if (_alertState.calOld) html += '<div class="alert warn"><span class="a-dot"></span><span class="a-text">pH calibration is over 30 days old</span></div>';
    alerts.innerHTML = html;
  }
  function fmtAgo(sec) {
    if (sec < 60) return sec + 's ago';
    if (sec < 3600) return Math.floor(sec/60) + 'm ago';
    if (sec < 86400) return Math.floor(sec/3600) + 'h ago';
    return Math.floor(sec/86400) + 'd ago';
  }

  function init() {
    initCharts();
    initHeroSparks();
    initKHLayers();
    initTabs();
    initPageNav();
    initLiveBanner();
    initButtons();
    initConfigInputs();
    // Smooth half-life UI-only setting (localStorage)
    var hlInp = document.getElementById('ui-smooth-halflife');
    if (hlInp) {
      try { var stored = localStorage.getItem('smoothHalfLife'); if (stored !== null) hlInp.value = stored; } catch(e) {}
      hlInp.addEventListener('change', function() {
        try { localStorage.setItem('smoothHalfLife', hlInp.value); } catch(e) {}
        renderKHChart();
        // Brief save feedback
        var fb = hlInp.parentNode.querySelector('.save-fb');
        if (!fb) {
          fb = document.createElement('span');
          fb.className = 'save-fb';
          fb.style.cssText = 'font-size:.75em;color:#30d158;margin-left:6px;transition:opacity .4s';
          hlInp.parentNode.appendChild(fb);
        }
        fb.textContent = 'Saved';
        fb.style.opacity = '1';
        clearTimeout(fb._t);
        fb._t = setTimeout(function() { fb.style.opacity = '0'; }, 1500);
      });
    }
    initSchedule();
    initMotorDiag();
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
