export const STYLE_NAMES = Object.freeze({1: '梵高', 2: '浮世绘', 3: '水墨'});
export const MODE_NAMES = Object.freeze({0: '全屏风格', 1: '并肩对比', 2: '中线对比'});

export function createRequestId(cryptoProvider = globalThis.crypto) {
  if (!cryptoProvider?.getRandomValues) throw new Error('当前浏览器不支持安全请求标识，请使用 Chrome。');
  const bytes = new Uint8Array(16);
  cryptoProvider.getRandomValues(bytes);
  return `pc-${Array.from(bytes, b => b.toString(16).padStart(2, '0')).join('')}`;
}

export function validateCommand(command, value) {
  if (command === 'style') return Number.isInteger(value) && value >= 1 && value <= 3;
  if (command === 'mode') return Number.isInteger(value) && value >= 0 && value <= 2;
  if (command === 'fps') return typeof value === 'boolean';
  return ['next', 'mode_next', 'effect'].includes(command) && value === undefined;
}

export class ApiError extends Error {
  constructor(message, {status = 0, uncertain = false, record = null} = {}) {
    super(message); this.name = 'ApiError'; this.status = status; this.uncertain = uncertain; this.record = record;
  }
}

export function unwrapCommandRecord(data) {
  const record = data?.command && typeof data.command === 'object' ? data.command : data?.record && typeof data.record === 'object' ? data.record : data;
  if (!record || typeof record.request_id !== 'string' || !['queued', 'sent', 'failed', 'uncertain'].includes(record.state)) throw new ApiError('命令响应格式未知：操作可能已接收，请检查实时状态后再操作。', {uncertain: true});
  return record;
}

export function createRemoteApi({fetchImpl = globalThis.fetch.bind(globalThis), timeoutMs = 4000, cryptoProvider = globalThis.crypto} = {}) {
  async function request(path, options = {}, mutating = false) {
    const abort = new AbortController();
    const timeout = setTimeout(() => abort.abort(), timeoutMs);
    try {
      const response = await fetchImpl(path, {...options, signal: abort.signal, cache: 'no-store'});
      let data;
      try { data = await response.json(); }
      catch { throw new ApiError('服务端响应不是有效 JSON；请检查连接和固件版本。', {status: response.status, uncertain: mutating}); }
      if (!response.ok) throw new ApiError(data.message || data.error || `HTTP ${response.status}`, {status: response.status, uncertain: mutating && ![400, 409, 422, 503].includes(response.status), record: data.command || data.record || null});
      return data;
    } catch (error) {
      if (error instanceof ApiError) throw error;
      throw new ApiError(mutating ? '请求结果未知：可能已被设备接收。请先检查实时状态和记录，再决定是否再次操作。' : '无法读取设备状态，请检查电脑是否连接 S3 热点。', {uncertain: mutating});
    } finally { clearTimeout(timeout); }
  }
  return {
    async status() {
      const data = await request('/api/status');
      if (data.version !== 1 || !data.bridge || !data.capabilities || !data.fpga || !Array.isArray(data.commands)) throw new ApiError('状态格式不兼容，请检查固件版本。');
      return data;
    },
    makeCommand(command, value) {
      if (!validateCommand(command, value)) throw new ApiError('无效的控制参数。');
      return {request_id: createRequestId(cryptoProvider), command, ...(value === undefined ? {} : {value})};
    },
    async control(body) {
      // Exactly one HTTP attempt. A timeout never silently repeats a cycle/effect action.
      const record = unwrapCommandRecord(await request('/api/control', {method: 'POST', headers: {'Content-Type': 'application/json'}, body: JSON.stringify(body)}, true));
      if (record.request_id !== body.request_id) throw new ApiError('命令响应 ID 不匹配：请检查实时状态后再操作。', {uncertain: true});
      return record;
    }
  };
}

export function createStatusPoller({read, onStatus, onError, intervalMs = 800}) {
  let timer = null, running = false, generation = 0, inFlight = false, refreshPending = false;
  async function poll() {
    if (!running || inFlight) { refreshPending = running; return; }
    inFlight = true;
    const current = generation;
    try { const data = await read(); if (running && current === generation) onStatus(data); }
    catch (error) { if (running && current === generation) onError(error); }
    finally {
      inFlight = false;
      if (running) { const delay = refreshPending ? 0 : intervalMs; refreshPending = false; timer = setTimeout(poll, delay); }
    }
  }
  return {
    start() { if (running) return; running = true; generation++; poll(); },
    stop() { running = false; generation++; refreshPending = false; clearTimeout(timer); },
    refresh() { if (!running) return; clearTimeout(timer); if (inFlight) refreshPending = true; else poll(); }
  };
}
