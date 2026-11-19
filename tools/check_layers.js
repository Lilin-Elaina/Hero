/**
 * 分层自检：只读扫描 include，检查 application/modules/bsp 之间的依赖方向。
 *
 * 硬规则（违反即报错）：
 *   modules/*.c  不许 include application/（下层反调上层 = 分层塌了）
 *   bsp/*.c      不许 include modules/ 或 application/
 * 软规则（只提示，属于"系统服务"）：
 *   application/*.c 直接 include bsp/ 的时间基准与 CAN 恢复接口——
 *   这两个是全局服务，参考工程里 robot.c 也直接调 BSPInit()，先记为欠账。
 *
 * 用法：node tools/check_layers.js
 */
const fs = require('fs');
const path = require('path');

const ROOT = path.resolve(__dirname, '..');
const LAYERS = ['application', 'modules', 'bsp'];

function walk(dir, out = []) {
  if (!fs.existsSync(dir)) return out;
  for (const e of fs.readdirSync(dir, { withFileTypes: true })) {
    const p = path.join(dir, e.name);
    if (e.isDirectory()) walk(p, out);
    else if (e.name.endsWith('.c')) out.push(path.dirname(p));
    else if (!e.name.endsWith('.h')) out.push(path.dirname(p));
  }
  return out;
}

/** 头文件所属层 */
const headerOwner = {};
for (const layer of LAYERS) {
  for (const dir of [...new Set(walk(path.join(ROOT, layer)))]) {
    for (const f of fs.readdirSync(dir)) {
      if (f.endsWith('.h')) headerOwner[f] = layer;
    }
  }
}

const RANK = { application: 3, modules: 2, bsp: 1 };

const errors = [];
const warn = [];
const halCalls = [];

for (const layer of LAYERS) {
  for (const dir of [...new Set(walk(path.join(ROOT, layer)))]) {
    for (const f of fs.readdirSync(dir)) {
      if (!f.endsWith('.c')) continue;
      const p = path.join(dir, f);
      const rel = path.relative(ROOT, p);
      const src = fs.readFileSync(p, 'utf8');

      for (const m of src.matchAll(/#include\s+"([^"]+)"/g)) {
        const owner = headerOwner[path.basename(m[1])];
        if (!owner || owner === layer) continue;
        if (RANK[owner] > RANK[layer]) {
          errors.push(`${rel}  ->  ${m[1]}  (${owner}：下层反向依赖上层)`);
        } else if (layer === 'application' && owner === 'bsp') {
          warn.push(`${rel}  ->  ${m[1]}`);
        }
      }
      if (layer !== 'bsp') {
        halCalls.push(...[...src.matchAll(/\bHAL_[A-Za-z_]+\(/g)].map(m => `${rel}: ${m[0]}`));
      }
    }
  }
}

console.log(`硬规则违规：${errors.length} 处`);
errors.forEach(r => console.log('  ✗ ' + r));
console.log(`\n软规则提示（APP 直接用 bsp 的系统服务）：${warn.length} 处`);
warn.forEach(r => console.log('  · ' + r));
console.log(`\n上层（application/modules）直接调用 HAL_*()：${halCalls.length} 处`);
halCalls.slice(0, 10).forEach(c => console.log('  ✗ ' + c));

process.exitCode = errors.length ? 1 : 0;
