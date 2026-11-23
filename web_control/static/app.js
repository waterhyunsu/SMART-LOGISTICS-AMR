// SMART-LOGISTICS 웹 관제 (E)
// 화면에 보이는 미션·Job·단계 상태는 모두 A가 준 값(a_state)이다. E는 요청과 발신 기록만 관리한다.

const $ = (s) => document.querySelector(s);
const esc = (s) => String(s ?? '').replace(/[&<>"']/g, (c) => ({ '&': '&amp;', '<': '&lt;', '>': '&gt;', '"': '&quot;', "'": '&#39;' }[c]));
// A 상태 값은 A 코드(mission_manager/node.py) 그대로 소문자로 온다.
const KO = {
  pending: '대기', executing: '이동 중', waiting_confirmation: '인계 대기', completed: '완료',
  run_on: '허용됨', run_off: '실행 요청 전',
  idle: '대기', active: '운행 중', returning_home: 'HOME 복귀 중', recovery_required: 'A 복구 확인 중',
  applied: '계획 적용', deferred: '인계 후 재계획', rejected: '계획 거부', timeout: '계획 시간 초과', failed: '계획 실패',
  PENDING_SEND: '전송 중', UNCONFIRMED: '응답 미확인', RECOVERY_PENDING: 'A 복구 대기', ACCEPTED: '수락', REJECTED: '거부', CONFLICT: '충돌',
};
const badge = (s) => `<span class="st st-${esc(s)}">${esc(KO[s] || s)}</span>`;
const hms = (iso) => (iso ? new Date(iso).toLocaleTimeString('ko-KR', { hour12: false }) : '');

let S = { config: null, wps: {}, snap: null };

async function api(path, opts = {}) {
  const r = await fetch(path, { headers: { 'Content-Type': 'application/json' }, ...opts });
  const body = await r.json().catch(() => ({}));
  if (!r.ok) throw new Error(typeof body.detail === 'string' ? body.detail : r.statusText);
  return body;
}
function msg(text, ok, keepFault) {
  const m = $('#form-msg'); m.className = `small ${ok ? 'ok' : 'err'}`; m.textContent = text;
  if (!keepFault) document.querySelectorAll('[data-fault]').forEach((x) => x.classList.remove('armed')); // 결함은 다음 요청 1회에만 적용
}

// ------------------------------------------------------------------ 실시간
function connect() {
  const ws = new WebSocket(`${location.protocol === 'https:' ? 'wss' : 'ws'}://${location.host}/ws`);
  ws.onopen = () => { $('#ws-badge').className = 'badge on'; $('#ws-badge').textContent = '웹 실시간'; };
  ws.onclose = () => { $('#ws-badge').className = 'badge off'; $('#ws-badge').textContent = '웹 연결 끊김'; setTimeout(connect, 2000); };
  ws.onmessage = (e) => { S.snap = JSON.parse(e.data); render(); };
}

function render() {
  const s = S.snap; if (!s) return;
  const a = s.a_state || {};
  $('#a-badge').className = `badge ${s.a_connected ? 'on' : 'off'}`;
  $('#a-badge').textContent = s.a_connected ? `A 연결됨${a.source ? ` (${a.source})` : ''}` : 'A 응답 없음';
  $('#recovery-banner').hidden = !a.recovery_required;
  const rb = $('#recovery-toggle'); if (rb) { rb.classList.toggle('armed', !!a.recovery_required); rb.textContent = a.recovery_required ? 'A 복구 완료 처리' : 'A 재시작(복구 대기)'; }
  renderMission(a, s.handoffs);
  renderJobs(a);
  renderHandoffs(s.handoffs);
  renderJobRequests(s.job_requests);
  draw(a);
}

// ------------------------------------------------------------------ 현재 미션 + 인계
function renderMission(a, handoffs) {
  // A가 실행을 허용하기 전(run_authorized=false)에는 '운행 중' 대신 '실행 요청 전'으로 표시
  $('#mission-status').innerHTML = a.mission_status
    ? badge(a.mission_status === 'active' && !a.run_authorized ? 'run_off' : a.mission_status) : '';
  $('#mission-summary').innerHTML = a.mission_id ? `
    <dl class="kv">
      <dt>미션</dt><dd>${esc(a.mission_id)}</dd>
      <dt>활성 계획 버전</dt><dd>v${a.active_version ?? 0} ${a.plan_status ? badge(a.plan_status) : ''}</dd>
      <dt>비용표</dt><dd>${esc(a.cost_revision || '-')}</dd>
      <dt>실행</dt><dd>${a.run_authorized ? badge('run_on') : badge('run_off')}</dd>
      <dt>현재 목표</dt><dd>${esc(a.current_goal ? (a.current_goal.step_id || 'HOME 복귀') : '-')}</dd>
    </dl>` : '<span class="muted">미션 없음</span>';
  const canRun = (a.steps || []).length > 0 && !a.run_authorized && a.mission_status !== 'completed';
  $('#run-btn').disabled = !canRun;
  $('#run-btn').textContent = a.run_authorized ? '미션 실행 중' : '미션 실행 요청';

  const box = $('#current-step');
  const goal = a.current_goal;
  const step = goal && goal.step_id ? (a.steps || []).find((x) => x.step_id === goal.step_id) : null;
  if (step && a.waiting_confirmation) {
    // 도착 → A가 인계 대기로 전환. 이 단계에 대한 E의 최근 발신 기록
    const last = (handoffs || []).find((h) => h.mission_id === a.mission_id && h.step_id === step.step_id);
    const open = last && ['PENDING_SEND', 'UNCONFIRMED', 'RECOVERY_PENDING'].includes(last.status);
    const what = step.kind === 'PICKUP' ? '물품을 바구니에 실은 뒤' : '물품을 내린 뒤';
    box.innerHTML = `<div class="box waiting">
      <p><b>${esc(step.waypoint_id)}</b> 도착 · <b>${esc(step.step_id)}</b> ${badge('waiting_confirmation')}</p>
      <p class="small">${what} 인계 완료를 눌러주세요. A가 수락해야 다음 작업으로 넘어갑니다.</p>
      ${open
        ? `<p class="small err">${last.status === 'RECOVERY_PENDING'
              ? `A가 재시작 후 복구 확인 중이라 요청 ${esc(last.event_id)} 을(를) 아직 처리하지 않았습니다. 복구가 끝나면 같은 요청으로 재전송하세요.`
              : `이전 요청 ${esc(last.event_id)} 의 결과를 모릅니다 (${esc(KO[last.status])}). 같은 요청으로 재전송하세요.`}</p>
           <button class="confirm" data-retry="${esc(last.event_id)}">재전송</button>`
        : `<button class="confirm" data-handoff="${esc(step.step_id)}">인계 완료</button>`}
      ${last && last.status === 'REJECTED' ? `<p class="small err">직전 요청이 거부됨 — 실제로 인계했다면 다시 누르세요 (새 요청)</p>` : ''}
    </div>`;
  } else if (step) {
    box.innerHTML = `<div class="box moving"><p><b>${esc(step.waypoint_id)}</b>(으)로 이동 중 · ${esc(step.step_id)} ${badge(step.status)}</p></div>`;
  } else if (goal) {
    box.innerHTML = `<div class="box moving"><p><b>HOME</b>(으)로 복귀 중 <span class="small muted">(물품 인계 단계 아님)</span></p></div>`;
  } else if ((a.steps || []).length && !a.run_authorized && a.mission_status !== 'completed') {
    box.innerHTML = `<div class="box"><p>작업이 등록되었습니다. <b>미션 실행 요청</b>을 누르면 A가 준비 상태를 확인한 뒤 출발합니다.</p></div>`;
  } else if (a.mission_status === 'completed') {
    box.innerHTML = `<div class="box"><p>모든 인계와 HOME 복귀가 끝났습니다</p></div>`;
  } else box.innerHTML = '';

  box.querySelectorAll('[data-handoff]').forEach((b) => b.onclick = async () => {
    b.disabled = true;
    try { const r = await api('/api/handoffs', { method: 'POST', body: JSON.stringify({ mission_id: a.mission_id, step_id: b.dataset.handoff }) });
          msg(`${r.event_id}: ${KO[r.status]}`, r.status === 'ACCEPTED'); }
    catch (e) { msg(e.message); b.disabled = false; }
  });
  box.querySelectorAll('[data-retry]').forEach((b) => b.onclick = () => retryHandoff(b.dataset.retry, b));

  // 남은 계획 (A가 승인한 순서)
  const plan = [goal && goal.step_id, ...(a.order || [])].filter(Boolean);
  const deferred = a.deferred_order || null;
  $('#plan-view').innerHTML = plan.length || deferred ? `
    <div><b>방문 계획 v${a.active_version}</b> <span class="muted small">A가 승인한 순서</span></div>
    <div class="route">${plan.map((id, i) => `<span class="wp ${i === 0 && goal && goal.step_id ? 'now' : ''}">${esc(id)}</span>`).join('<span class="muted">→</span>')}</div>
    ${deferred ? `<div class="small muted" style="margin-top:6px">재계획 보류 (현재 단계 인계 후 적용): ${deferred.map(esc).join(', ')}</div>` : ''}` : '';
}

async function retryHandoff(id, btn) {
  if (btn) btn.disabled = true;
  try { const r = await api(`/api/handoffs/${encodeURIComponent(id)}/retry`, { method: 'POST' });
        msg(`${r.event_id} 재전송: ${KO[r.status]}`, r.status === 'ACCEPTED'); }
  catch (e) { msg(e.message); }
  if (btn) btn.disabled = false;
}

// ------------------------------------------------------------------ Job 목록 (A 상태)
function renderJobs(a) {
  const steps = {}; (a.steps || []).forEach((x) => { (steps[x.job_id] ||= {})[x.kind] = x; });
  $('#job-table tbody').innerHTML = (a.jobs || []).map((j) => {
    const p = steps[j.job_id]?.PICKUP, d = steps[j.job_id]?.DROPOFF;
    return `<tr><td><b>${esc(j.job_id)}</b></td><td>${esc(p?.waypoint_id)} → ${esc(d?.waypoint_id)}</td>
      <td>${p ? badge(p.status) : ''}</td><td>${d ? badge(d.status) : ''}</td><td>${badge(j.status)}</td></tr>`;
  }).join('') || '<tr><td colspan="5" class="muted">A에 등록된 Job이 없습니다</td></tr>';
}

// ------------------------------------------------------------------ E 발신 기록
function renderHandoffs(rows) {
  $('#handoff-table tbody').innerHTML = (rows || []).slice(0, 12).map((h) => `<tr>
    <td class="mono">${esc(h.event_id)}</td><td>${esc(h.step_id)}</td>
    <td>${badge(h.status)}${h.last_error ? `<div class="small muted">${esc(h.last_error)}</div>` : ''}</td>
    <td>${h.attempts}</td><td class="small">${hms(h.updated_at)}</td>
    <td>${['PENDING_SEND', 'UNCONFIRMED', 'RECOVERY_PENDING'].includes(h.status) ? `<button class="retry small" data-hretry="${esc(h.event_id)}">재전송</button>` : ''}</td></tr>`).join('')
    || '<tr><td colspan="6" class="muted">기록 없음</td></tr>';
  document.querySelectorAll('[data-hretry]').forEach((b) => b.onclick = () => retryHandoff(b.dataset.hretry, b));
}

function renderJobRequests(rows) {
  $('#jobreq-table tbody').innerHTML = (rows || []).slice(0, 12).map((r) => `<tr>
    <td class="mono">${esc(r.request_id)}</td><td>${esc(r.pickup)} → ${esc(r.dropoff)}</td>
    <td>${badge(r.status)}${r.last_error || r.reason ? `<div class="small muted">${esc(r.last_error || r.reason)}</div>` : ''}</td>
    <td>${esc(r.job_id || '-')}</td><td>${r.attempts}</td>
    <td>${['PENDING_SEND', 'UNCONFIRMED'].includes(r.status) ? `<button class="retry small" data-jretry="${esc(r.request_id)}">재전송</button>` : ''}</td></tr>`).join('')
    || '<tr><td colspan="6" class="muted">기록 없음</td></tr>';
  document.querySelectorAll('[data-jretry]').forEach((b) => b.onclick = async () => {
    b.disabled = true;
    try { const r = await api(`/api/job-requests/${encodeURIComponent(b.dataset.jretry)}/retry`, { method: 'POST' }); msg(`${r.request_id} 재전송: ${KO[r.status]}`, r.status === 'ACCEPTED'); }
    catch (e) { msg(e.message); b.disabled = false; }
  });
}

// ------------------------------------------------------------------ 지도
const canvas = $('#map'), ctx = canvas.getContext('2d');
function view() {
  const pts = Object.values(S.wps).map((w) => [w.x, w.y]);
  const xs = pts.map((p) => p[0]), ys = pts.map((p) => p[1]);
  const b = { minX: Math.min(...xs) - 0.6, maxX: Math.max(...xs) + 0.6, minY: Math.min(...ys) - 0.6, maxY: Math.max(...ys) + 0.6 };
  const w = b.maxX - b.minX, h = b.maxY - b.minY, s = Math.min(canvas.width / w, canvas.height / h);
  return { b, s, ox: (canvas.width - w * s) / 2, oy: (canvas.height - h * s) / 2 };
}
const px = (x, y, v) => [v.ox + (x - v.b.minX) * v.s, canvas.height - (v.oy + (y - v.b.minY) * v.s)];

function draw(a) {
  if (!S.config) return;
  const v = view();
  ctx.clearRect(0, 0, canvas.width, canvas.height);
  ctx.strokeStyle = '#eceef1'; ctx.lineWidth = 1;
  for (let x = Math.ceil(v.b.minX); x <= v.b.maxX; x++) { const [x1, y1] = px(x, v.b.minY, v), [, y2] = px(x, v.b.maxY, v); ctx.beginPath(); ctx.moveTo(x1, y1); ctx.lineTo(x1, y2); ctx.stroke(); }
  for (let y = Math.ceil(v.b.minY); y <= v.b.maxY; y++) { const [x1, y1] = px(v.b.minX, y, v), [x2] = px(v.b.maxX, y, v); ctx.beginPath(); ctx.moveTo(x1, y1); ctx.lineTo(x2, y1); ctx.stroke(); }

  const stepWp = Object.fromEntries((a.steps || []).map((x) => [x.step_id, x.waypoint_id]));
  const target = a.current_goal ? a.current_goal.waypoint_id : null;
  // 남은 계획 경로
  const route = [];
  if (a.robot) route.push([a.robot.x, a.robot.y]);
  [a.current_goal && a.current_goal.step_id, ...(a.order || [])].filter(Boolean).forEach((id) => { const w = S.wps[stepWp[id]]; if (w) route.push([w.x, w.y]); });
  if (['active', 'returning_home'].includes(a.mission_status) && S.wps.HOME) route.push([S.wps.HOME.x, S.wps.HOME.y]);
  if (route.length > 1) {
    ctx.strokeStyle = '#d97706'; ctx.lineWidth = 3; ctx.setLineDash([10, 7]); ctx.beginPath();
    route.forEach(([x, y], i) => { const [X, Y] = px(x, y, v); i ? ctx.lineTo(X, Y) : ctx.moveTo(X, Y); });
    ctx.stroke(); ctx.setLineDash([]);
  }
  for (const w of Object.values(S.wps)) {
    const [X, Y] = px(w.x, w.y, v);
    ctx.beginPath(); ctx.arc(X, Y, 17, 0, Math.PI * 2);
    ctx.fillStyle = w.id === target ? '#f97316' : (w.id === 'HOME' ? '#4f46e5' : '#374151'); ctx.fill();
    ctx.fillStyle = '#fff'; ctx.font = 'bold 13px sans-serif'; ctx.textAlign = 'center'; ctx.textBaseline = 'middle';
    ctx.fillText(w.id === 'HOME' ? 'H' : w.id.replace('PICKUP_', 'P').replace('DROP_', 'D').slice(0, 3), X, Y);
    ctx.fillStyle = '#374151'; ctx.font = '12px sans-serif'; ctx.fillText(w.id, X, Y + 28);
  }
  if (a.robot) {
    const [X, Y] = px(a.robot.x, a.robot.y, v);
    ctx.beginPath(); ctx.arc(X, Y, 9, 0, Math.PI * 2); ctx.fillStyle = '#2563eb'; ctx.fill();
    ctx.strokeStyle = '#fff'; ctx.lineWidth = 3; ctx.stroke();
    $('#robot-text').textContent = `${a.source === 'MOCK' ? 'MOCK 위치' : '위치'} x ${a.robot.x.toFixed(2)} y ${a.robot.y.toFixed(2)}`;
  }
}

// ------------------------------------------------------------------ 요청
$('#job-form').onsubmit = async (e) => {
  e.preventDefault();
  const f = new FormData(e.target);
  try { const r = await api('/api/job-requests', { method: 'POST', body: JSON.stringify({ pickup: f.get('pickup'), dropoff: f.get('dropoff') }) });
        msg(`${r.request_id}: ${KO[r.status]}${r.job_id ? ` → ${r.job_id}` : ''}`, r.status === 'ACCEPTED'); }
  catch (err) { msg(err.message); }
};
document.querySelectorAll('[data-fault]').forEach((b) => b.onclick = async () => {
  const body = { drop_next_ack: false, timeout_next: false, not_ready: !(S.snap?.a_state?.ready ?? true) }; body[b.dataset.fault] = true;
  await api('/api/mock/faults', { method: 'POST', body: JSON.stringify(body) });
  document.querySelectorAll('[data-fault]').forEach((x) => x.classList.toggle('armed', x === b));
  msg(`다음 요청 1회에 '${b.textContent}' 적용됨`, true, true);
});

$('#run-btn').onclick = async () => {
  const a = S.snap && S.snap.a_state; if (!a || !a.mission_id) return;
  try { const r = await api('/api/mission/run', { method: 'POST', body: JSON.stringify({ mission_id: a.mission_id }) });
        msg(r.outcome === 'accepted' ? `미션 실행 허용됨${r.reason ? ` (${r.reason})` : ''}` : `미션 실행 거부: ${r.reason}`, r.outcome === 'accepted'); }
  catch (e) { msg(e.message); }
};

$('#recovery-toggle').onclick = async () => {
  const on = !(S.snap && S.snap.a_state && S.snap.a_state.recovery_required);
  await api('/api/mock/recovery', { method: 'POST', body: JSON.stringify({ required: on }) });
  msg(on ? 'MOCK: A가 복구 확인 중 상태 — 새 인계는 처리되지 않음' : 'MOCK: A 복구 완료 — 대기 중인 요청을 재전송하세요', true, true);
};

(async () => {
  S.config = await api('/api/config');
  S.config.waypoints.forEach((w) => { S.wps[w.id] = w; });
  $('#mode-badge').textContent = S.config.run_mode; $('#mode-badge').className = 'badge mode';
  $('#mock-tools').hidden = S.config.run_mode !== 'MOCK';
  const opts = S.config.waypoints.map((w) => `<option title="${esc(w.role)}">${esc(w.id)}</option>`).join('');
  $('#job-form [name=pickup]').innerHTML = opts; $('#job-form [name=dropoff]').innerHTML = opts;
  $('#job-form [name=pickup]').value = S.config.waypoints.find((w) => w.id !== 'HOME')?.id || S.config.waypoints[0].id;
  $('#job-form [name=dropoff]').selectedIndex = S.config.waypoints.length - 1;
  $('#plan-src').textContent = `시작 지점 ${S.config.start_waypoint} · 비용표 ${S.config.cost_revision} (설정값, A와 합의 전)`;
  connect();
})();
