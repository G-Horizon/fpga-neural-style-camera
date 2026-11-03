import {createRemoteApi, createStatusPoller, STYLE_NAMES, MODE_NAMES} from './remote_api.mjs';

const api = createRemoteApi();
const $ = id => document.getElementById(id);
const display = (id, value) => { $(id).textContent = value ?? '—'; };
const onOff = value => value === null || value === undefined ? '未知' : value ? '开启' : '关闭';
const markActive = (button, active) => { button.classList.toggle('active', active); button.setAttribute('aria-pressed', String(active)); };
const localRecords = new Map();
let lastStatus = null, lastReceivedAt = 0, pollingFailed = true, sending = 0;
const stateNames = {queued: '排队中', sent: 'A板已接收', failed: '失败', uncertain: '结果未知'};
const nameOfCommand = ({command, value}) => ({style: `风格 → ${STYLE_NAMES[value] || '?'}`, next: '下一种风格', mode: `布局 → ${MODE_NAMES[value] || '?'}`, mode_next: '下一显示布局', fps: `FPS → ${onOff(value)}`, effect: '触发约 1 秒特效'}[command] || command);

function updateControls() {
  const fresh = lastStatus && !pollingFailed && Date.now() - lastReceivedAt < 5000;
  const connected = fresh && lastStatus.a_state?.connected;
  for (const button of document.querySelectorAll('.control')) {
    const allowed = connected && lastStatus.capabilities.style && (!button.classList.contains('extended') || lastStatus.capabilities.extended);
    button.disabled = !allowed || sending > 0;
    button.title = !fresh ? '请先获取有效设备状态' : !connected ? 'A板控制连接尚未建立' : !allowed ? '当前位流不支持此功能' : '';
  }
}

function feedback(message, kind = '') { display('feedback', message); $('feedback').className = `command-feedback ${kind}`; }

function renderLog() {
  const records = new Map(localRecords);
  for (const record of lastStatus?.commands || []) {
    const local = localRecords.get(record.request_id);
    records.set(record.request_id, local?.state === 'uncertain' && record.state === 'queued' ? local : record);
  }
  const sorted = [...records.values()].sort((a, b) => b.created_at_ms - a.created_at_ms).slice(0, 16);
  if (!sorted.length) return;
  const rows = sorted.map(record => {
    const tr = document.createElement('tr');
    const time = document.createElement('td');
    // Device times are uptime milliseconds, not the PC wall clock.
    time.append(document.createTextNode(typeof record.created_at_ms === 'number' ? `运行 ${(record.created_at_ms / 1000).toFixed(1)} s` : '本次请求'));
    const id = document.createElement('small'); id.textContent = record.request_id; time.append(id);
    const operation = document.createElement('td'); operation.textContent = nameOfCommand(record);
    const state = document.createElement('td'); const tag = document.createElement('span'); tag.className = `command-state ${record.state}`; tag.textContent = stateNames[record.state] || '状态未知'; state.append(tag);
    const detail = document.createElement('td'); detail.textContent = record.message || record.error || '等待设备处理';
    tr.append(time, operation, state, detail); return tr;
  });
  $('command-log').replaceChildren(...rows);
}

function renderStatus(data) {
  lastStatus = data; lastReceivedAt = Date.now(); pollingFailed = false;
  const {bridge: b, capabilities: c, fpga: f} = data;
  $('connection-dot').className = `dot ${b.connected ? 'ok' : 'error'}`;
  display('connection-text', data.mock_source ? '模拟服务' : `A控制${data.a_state?.connected?'在线':'离线'} · B显示${b.connected?'在线':'离线'}`);
  display('status-age', `状态 ${b.status_age_ms === null ? '年龄未知' : `${b.status_age_ms} ms 前`}`);
  const legacy = b.connected && !c.extended;
  $('source-banner').hidden = !data.mock_source && !legacy;
  $('source-banner').textContent = [data.mock_source ? '模拟模式 · 当前数据来自本机 HTTP 测试服务，不代表 FPGA、ESP32 或无线链路已通过实物验证。' : '', legacy ? '旧 FPGA 位流 · 仅支持风格切换；布局、FPS、特效和实时 FPGA 状态需要无线调试版位流。' : ''].filter(Boolean).join(' ');
  $('error-banner').hidden = !!b.connected&&!!data.a_state?.connected;
  $('error-banner').textContent = !data.a_state?.connected?'A板控制连接离线，暂不能发送操作。':'B板画面状态离线；A板仍可接收命令，画面是否生效需等待B板状态恢复。';
  display('active-style', f.model_valid === true ? STYLE_NAMES[f.active_style] || '未知' : f.model_valid === false ? '未就绪' : '未知');
  display('target-style', f.target_style === null ? '未知' : `${STYLE_NAMES[f.target_style] || '未知'}${f.target_pending ? ' · 加载中' : ''}`);
  display('model-state', f.model_valid === null ? '未知' : f.model_valid ? '模型有效' : '模型未就绪');
  $('model-state').className = `pill ${f.model_valid === null ? 'neutral' : f.model_valid ? 'ok' : 'pending'}`;
  display('epoch', f.active_epoch === null ? '未知' : f.active_epoch);
  display('frame', f.frame_valid && f.latest_frame_id !== null ? f.latest_frame_id : '无完整帧');
  display('fps', f.source_fps === null ? '未知' : `${f.source_fps} FPS`);
  for (const type of ['requested', 'accepted', 'actual']) display(`mode-${type}`, MODE_NAMES[f[`${type}_mode`]] || '未知');
  for (const type of ['requested', 'actual']) { display(`fps-${type}`, onOff(f[`fps_${type}`])); display(`effect-${type}`, onOff(f[`effect_${type}`])); }
  display('spi', b.connected ? '近期有效响应' : '未连接');
  const a=data.a_state||{};display('a-link',a.connected?'控制在线':'离线');display('a-epoch',a.connected?a.requested_epoch:'未知');display('a-fids',a.connected?`${a.capture_fid} / ${a.computed_fid} / ${a.transmitted_fid}`:'未知');display('a-audio',a.connected?`${a.next_audio_sample} / ${a.pcm_free}`:'未知');
  display('transport', `${b.transport_ready ? '已初始化' : '未初始化'} / ${b.ap_clients ?? '?'} 台`);
  display('streams', `${b.stream_clients ?? '?'} / ${b.stream_capacity ?? 3}`);
  display('stream-capacity', `本页不占视频席位 · 已用 ${b.stream_clients ?? '?'} / ${b.stream_capacity ?? 3} 路`);
  display('uptime', b.uptime_ms === null ? '未知' : `${Math.floor(b.uptime_ms / 60000)} 分 ${Math.floor(b.uptime_ms / 1000) % 60} 秒`);
  display('heap', b.free_heap_bytes === null ? '未知' : `${(b.free_heap_bytes / 1024).toFixed(1)} KiB`);
  $('last-error').hidden = !b.last_error; display('last-error', b.last_error);
  for (const button of document.querySelectorAll('[data-command="style"]')) markActive(button, Number(button.dataset.value) === f.active_style && f.model_valid === true);
  for (const button of document.querySelectorAll('[data-command="mode"]')) markActive(button, Number(button.dataset.value) === f.actual_mode);
  for (const button of document.querySelectorAll('[data-command="fps"]')) markActive(button, (button.dataset.value === 'true') === f.fps_actual && typeof f.fps_actual === 'boolean');
  renderLog(); updateControls();
}

function statusError(error) {
  pollingFailed = true; display('connection-text', '状态不可用'); $('connection-dot').className = 'dot error';
  $('error-banner').hidden = false; $('error-banner').textContent = error.message;
  display('status-age', lastReceivedAt ? `显示上次快照 · ${Math.round((Date.now() - lastReceivedAt) / 1000)} 秒前` : '尚未获取状态');
  updateControls();
}

const poller = createStatusPoller({read: () => api.status(), onStatus: renderStatus, onError: statusError});
for (const button of document.querySelectorAll('.control')) button.addEventListener('click', async () => {
  let body;
  try { body = api.makeCommand(button.dataset.command, button.dataset.value === undefined ? undefined : button.dataset.command === 'fps' ? button.dataset.value === 'true' : Number(button.dataset.value)); }
  catch (error) { feedback(error.message, 'error'); return; }
  const local = {...body, value: body.value ?? null, state: 'queued', message: '正在提交；尚未确认排队', created_at_ms: lastStatus?.bridge.uptime_ms ?? null, completed_at_ms: null, error: null};
  localRecords.set(body.request_id, local);
  if (localRecords.size > 32) localRecords.delete(localRecords.keys().next().value);
  sending++; updateControls(); renderLog(); feedback(`正在提交：${nameOfCommand(body)}…`);
  try {
    const record = await api.control(body);
    localRecords.set(body.request_id, {...local, ...record, request_id: body.request_id});
    feedback(`${nameOfCommand(body)}：${stateNames[record.state] || '已提交'}。请查看实时状态确认实际呈现。`, 'success');
  } catch (error) {
    localRecords.set(body.request_id, {...local, ...(error.record || {}), state: error.uncertain ? 'uncertain' : 'failed', message: error.message, error: error.message});
    feedback(error.message, 'error');
  } finally { sending--; updateControls(); renderLog(); poller.refresh(); }
});
$('refresh').addEventListener('click', () => poller.refresh());
$('preview-link').addEventListener('click', event => {
  if (lastStatus && Number.isInteger(lastStatus.bridge.stream_clients) && Number.isInteger(lastStatus.bridge.stream_capacity) && lastStatus.bridge.stream_clients >= lastStatus.bridge.stream_capacity) {
    event.preventDefault(); feedback('3 路视频席位已满，请先关闭一台设备的观看页，再打开预览。', 'error');
  }
});
document.addEventListener('visibilitychange', () => { if (document.hidden) poller.stop(); else poller.start(); });
window.addEventListener('pagehide', () => poller.stop());
setInterval(updateControls, 1000);
updateControls(); poller.start();
