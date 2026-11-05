/**
 * @file practice.js
 * @brief 本地练习网页交互：种子生成只修改待部署配置，部署沿用完整配置。
 * 地图算法及HTTP控制在C++中实现；本文件只收集表单、显示状态和绘制观赛地图。
 * 数据流：读取表单 → 请求本地C++服务 → 展示返回状态；浏览器不直接替选手发送动作。
 * 绘图坐标与裁判配置保持一致，地图生成、部署与开始比赛是独立操作。
 */
'use strict';
/** @brief 按固定表单ID读取DOM元素；本页元素由practice.html统一声明。 */
const $ = (id) => document.getElementById(id);
let busy = false,
  initialized = false,
  session = '',
  dirty = false;
for (const seat of 'ABCD') {
  $('bots').insertAdjacentHTML(
    'beforeend',
    `<label><input type="checkbox" value="${seat}">${seat}</label>`,
  );
}

/** @brief 请求同源C++网页入口，保留官方API的错误信息。 */
async function api(path, body) {
  const response = await fetch(
    '/api/' + path,
    body === undefined
      ? {}
      : {
          method: 'POST',
          headers: { 'Content-Type': 'application/json' },
          body: JSON.stringify(body),
        },
  );
  const value = await response.json();
  if (!response.ok)
    throw Error(typeof value.error === 'string' ? value.error : JSON.stringify(value.error));
  return value;
}

/** @brief 串行执行用户操作并禁用表单，防止生成与部署交叉覆盖待部署配置。 */
async function act(action) {
  if (busy) return;
  busy = true;
  $('notice').textContent = '处理中…';
  for (const element of document.querySelectorAll('button,select,input,textarea'))
    element.disabled = true;
  try {
    $('notice').textContent = (await action()) || '操作完成';
    $('notice').className = '';
  } catch (error) {
    $('notice').textContent = error.message;
    $('notice').className = 'bad';
  } finally {
    busy = false;
    for (const element of document.querySelectorAll('button,select,input,textarea'))
      element.disabled = false;
  }
}

/** @brief 配置不含某席位时取消其陪练勾选，合法的现有选择保持不变。 */
function syncBots(config) {
  const seats = (config.robots || []).map((robot) => robot.team);
  for (const input of document.querySelectorAll('#bots input'))
    if (!seats.includes(input.value)) input.checked = false;
}

/** @brief 显示种子含义及实际随机种子；自定义配置不虚构为种子0。 */
function describe(metadata) {
  if (!metadata) return '自定义地图（种子未知）';
  const size = `${metadata.width}×${metadata.height}`;
  if (metadata.official_template) return `历史官方练习图 · ${size}（生成新地图可切换标准尺寸）`;
  return `种子 ${metadata.seed} · ${size}${metadata.randomized ? '（随机生成，可用此数值复现）' : ' · 固定地图'}`;
}

/** @brief 从实际部署状态初始化表单；轮询不覆盖用户正在编辑的配置或种子。 */
function loadSettings(state) {
  const settings = state.settings;
  if (!settings) return;
  $('stage').value = settings.stage;
  $('tick').value = settings.tick_ms;
  $('max').value = settings.max_ticks;
  $('config').value = JSON.stringify(settings.config, null, 2);
  $('seed').value = state.map_seed?.requested_seed ?? '0';
  for (const input of document.querySelectorAll('#bots input'))
    input.checked = settings.bots.includes(input.value);
  $('pending').textContent = '';
  dirty = false;
  session = state.session_id;
  initialized = true;
}

/** @brief 生成待部署地图，绝不调用start或reset；-1每次生成一次新的实际种子。 */
async function generateMap() {
  const seed = $('seed').value.trim();
  if (!/^-?\d+$/.test(seed)) throw Error('地图种子须为整数：-1随机，其余整数（含0）固定');
  const result = await api('map', { stage: Number($('stage').value), seed });
  $('config').value = JSON.stringify(result.config, null, 2);
  syncBots(result.config);
  dirty = true;
  $('pending').textContent = '待部署：' + describe(result.metadata);
  return '地图已生成；当前练习不变，点“部署新练习”后应用。';
}

// 表单事件只修改待部署配置；实际比赛状态由明确的部署/运行按钮控制。
$('generate').onclick = () => act(generateMap);
$('seed').oninput = () => {
  dirty = true;
  $('pending').textContent = '种子尚未生成；点“生成地图”准备新图，直接部署仍使用配置中的地图。';
};
for (const input of [$('tick'), $('max'), ...document.querySelectorAll('#bots input')]) {
  input.addEventListener('change', () => {
    dirty = true;
  });
}
$('stage').onchange = () =>
  act(async () => {
    $('max').value = $('stage').value === '1' ? 100 : 500;
    return generateMap();
  });
$('config').oninput = () => {
  dirty = true;
  $('pending').textContent = '配置已编辑，仅下一次部署生效。';
};
// 部署使用当前完整JSON，种子输入框本身不会在这里重新生成地图。
$('deploy').onclick = () =>
  act(async () => {
    const config = JSON.parse($('config').value);
    if (config.stage !== Number($('stage').value))
      throw Error('配置阶段与选择不一致，请先生成对应阶段地图。');
    if (!confirm('按当前配置重开练习？地图点位保持当前配置，不会重新随机。')) return '已取消';
    await api('start', {
      stage: Number($('stage').value),
      tick_ms: Number($('tick').value),
      max_ticks: Number($('max').value),
      bots: [...document.querySelectorAll('#bots input:checked')].map((input) => input.value),
      config,
    });
    loadSettings(await api('state'));
    return '已按当前地图部署，点击“开始 / 继续”运行。';
  });
for (const command of ['resume', 'pause', 'reset']) {
  $(command).onclick = () =>
    act(async () => {
      if (command === 'reset' && !confirm('保留当前地图及日志，按已部署配置重新开始？'))
        return '已取消';
      await api(command, {});
      if (command === 'reset') loadSettings(await api('state'));
    });
}
// 配置导出与导入只作用于本页待部署内容，导入后仍需手动部署。
$('save').onclick = () =>
  act(async () => {
    const text = JSON.stringify(JSON.parse($('config').value), null, 2);
    const url = URL.createObjectURL(new Blob([text], { type: 'application/json' }));
    const link = document.createElement('a');
    link.href = url;
    link.download = 'practice-stage' + $('stage').value + '.json';
    link.click();
    setTimeout(() => URL.revokeObjectURL(url), 1000);
  });
$('import').onchange = () =>
  act(async () => {
    const file = $('import').files[0];
    if (!file) return;
    if (file.size > 1024 * 1024) throw Error('配置不得超过1 MiB');
    const config = JSON.parse(await file.text());
    if (![1, 2].includes(config.stage)) throw Error('stage须为1或2');
    $('stage').value = config.stage;
    $('config').value = JSON.stringify(config, null, 2);
    syncBots(config);
    dirty = true;
    $('pending').textContent = '待部署：导入配置（直接使用文件中的地图点位）。';
  });

/** @brief 绘制官方公开给观赛网页的地图，仅显示用途，不向参赛程序发送真值。 */
function draw(board) {
  const view = board.map_view;
  if (!view) return;
  const canvas = $('map'),
    context = canvas.getContext('2d');
  const dx = canvas.width / view.map.width,
    dy = canvas.height / view.map.height;
  context.fillStyle = '#0d1721';
  context.fillRect(0, 0, canvas.width, canvas.height);
  // 用比赛的左上原点坐标绘制单格，格间留出一像素便于区分边界。
  const square = (position, color) => {
    context.fillStyle = color;
    context.fillRect(position[0] * dx + 1, position[1] * dy + 1, dx - 2, dy - 2);
  };
  for (const position of view.obstacles || []) square(position, '#657787');
  for (const resource of view.resources || [])
    if (resource.available !== false)
      square(resource.position, resource.value >= 20 ? '#e1be50' : '#55a98c');
  for (const [index, robot] of (view.robots || []).entries()) {
    square(
      robot.position,
      robot.alive ? ['#ef7979', '#6eb6ff', '#b4cf68', '#d79be8'][index % 4] : '#343b43',
    );
    context.fillStyle = '#fff';
    context.font = `bold ${Math.max(10, dx * 0.5)}px sans-serif`;
    context.fillText(
      robot.team,
      robot.position[0] * dx + dx * 0.25,
      robot.position[1] * dy + dy * 0.65,
    );
  }
}

/** @brief 每500ms更新观赛信息；只在初始化或外部换场且表单未编辑时同步配置。 */
async function poll() {
  try {
    const state = await api('state'),
      board = state.board || {},
      status = state.status || {};
    if ((!initialized || (session !== state.session_id && !dirty)) && !busy) loadSettings(state);
    $('map-seed').textContent = '当前地图：' + describe(state.map_seed);
    $('status').textContent =
      `Tick ${status.tick ?? 0}/${status.max_ticks ?? '?'} · ${state.stopped ? '已停止' : state.starting ? '启动中' : status.finished ? '已结束' : status.paused ? '暂停' : '运行'} · ${state.session_id}`;
    if (state.fault || status.fault) {
      $('notice').textContent = state.fault || status.fault;
      $('notice').className = 'bad';
    }
    $('connection').textContent =
      `ROS_DOMAIN_ID=${state.public_domain}；AI地址 ${state.gateway}；模式 ${state.ai_mode}；practice-席位 为本场令牌的本地兼容别名`;
    $('logs').textContent = '本场日志：' + state.run_dir;
    $('checks').replaceChildren();
    for (const seat of 'ABCD') {
      const row = document.createElement('tr'),
        check = state.checks[seat] || {},
        ai = state.ai[seat];
      for (const value of [
        seat,
        check.perception_tick ?? '—',
        check.session_error || (check.action_tick ?? '—'),
        check.executed_tick ?? '—',
        `${state.ai_mode === 'mock' ? '模拟' : 'LLM'}成功 ${ai?.success_count ?? 0} 次${ai && !ai.success ? ' · 最近失败' : ''}`,
      ]) {
        const cell = document.createElement('td');
        cell.textContent = value;
        row.appendChild(cell);
      }
      $('checks').appendChild(row);
    }
    $('scores').textContent = (board.teams || [])
      .map(
        (team) =>
          `${team.tied ? '并列' : ''}${team.rank ?? '—'} ${team.team} ${team.team_name} · 得分 ${Number(team.score).toFixed(3)} · 资源 ${team.resource_value ?? 0} · 有效伤害 ${team.effective_damage ?? 0} · 任务 原始${team.mission_completed_raw ?? team.mission_completed ?? 0}/计分${team.mission_completed_eligible ?? 0} · AI资格 ${team.ai_eligible ? '本地真实调用通过' : '未满足'} · HP ${team.hp} · 无效动作 ${team.invalid_actions ?? 0}`,
      )
      .join('\n');
    $('actions').textContent = JSON.stringify(
      Object.fromEntries(
        Object.entries(state.checks).map(([seat, check]) => [
          seat,
          {
            last_action: check.last_action ?? null,
            error: check.session_error || check.parse_error || null,
          },
        ]),
      ),
      null,
      2,
    );
    $('events').textContent =
      [
        ...new Set(
          Object.values(state.checks)
            .map((check) => check.event_text)
            .filter(Boolean),
        ),
      ].join('\n') || '暂无公告';
    draw(board);
  } catch (error) {
    $('notice').textContent = '连接失败：' + error.message;
  } finally {
    setTimeout(poll, 500);
  }
}
poll();
