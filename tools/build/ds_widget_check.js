// Copyright (c) Probot Studio. Hand-written; see LICENSE.
//
// Cross-check of the Field2d, Mechanism2d and Scheduler widget layouts (design/core-requests.md,
// "Widget layouts for Field2d, Mechanism2d, Scheduler"): decodes the golden values of
// test/prosdk/commands2/WidgetLayoutTest.cpp with probot-core's own Driver Station functions,
// extracted from its ui/index.html, and encodes the DS write-backs (pose write, cancel) to compare
// them with the bytes prosdk's tests push. No browser: the canvas is a stub that records strokes.
// The goldens are copied by hand from the C++ test; update both together.
//
//   node tools/build/ds_widget_check.js .upstream/probot-core/ui/index.html
'use strict';
const fs = require('fs');
const vm = require('vm');
const html = fs.readFileSync(process.argv[2], 'utf8');

function extract(name) {
  const at = html.indexOf('function ' + name + '(');
  if (at < 0) throw new Error('missing ' + name);
  let i = html.indexOf('{', at), depth = 0;
  for (; i < html.length; ++i) {
    if (html[i] === '{') depth++;
    else if (html[i] === '}' && --depth === 0) return html.slice(at, i + 1);
  }
  throw new Error('unbalanced ' + name);
}
const names = ['dkI64', 'dkW64', 'dkPrim', 'dkDec', 'dkEnc', 'dkPoses', 'dkPoseBytes', 'dkDeg',
               'dkKid', 'dkFin', 'dkHex', 'dkNum', 'dkMech'];
const src = 'var DK_ES=[0,1,8,4,8];var textDecoder=new TextDecoder();\n' +
            'function dkCanvas(aspect,draw){return {aspect:aspect,draw:draw,attrs:{},' +
            'setAttribute:function(k,v){this.attrs[k]=v;},dkPaint:function(){}};}\n' +
            names.map(extract).join('\n');
const ctx = {TextDecoder, TextEncoder, BigInt, Math, Object, Uint8Array, DataView, String, isFinite};
vm.createContext(ctx);
vm.runInContext(src, ctx);

let fail = 0;
function check(label, got, want) {
  const g = JSON.stringify(got), w = JSON.stringify(want);
  console.log((g === w ? 'OK   ' : 'FAIL ') + label + ': ' + g + (g === w ? '' : ' (want ' + w + ')'));
  if (g !== w) fail++;
}
function f64le(...xs) {
  const b = new Uint8Array(8 * xs.length), dv = new DataView(b.buffer);
  xs.forEach((x, i) => dv.setFloat64(8 * i, x, true));
  return b;
}
function i64le(...xs) {
  const b = new Uint8Array(8 * xs.length), dv = new DataView(b.buffer);
  xs.forEach((x, i) => dv.setBigInt64(8 * i, BigInt(x), true));
  return b;
}
function strArr(...xs) {  // prosdk/core string[]: u16 LE length + UTF-8 per element
  const parts = xs.map(s => new TextEncoder().encode(s));
  const b = new Uint8Array(parts.reduce((n, p) => n + 2 + p.length, 0));
  let o = 0;
  parts.forEach(p => { b[o] = p.length & 255; b[o + 1] = p.length >> 8; b.set(p, o + 2); o += 2 + p.length; });
  return b;
}
const hex = b => Buffer.from(b).toString('hex');

// ---- Field2d: struct:Pose2d[] (raw, vt 6), 24 B per pose {x m, y m, rot rad} LE ----
const robot = f64le(1.5, 2.25, Math.PI / 2);
const targets = f64le(4, 0.5, Math.PI, 6.25, 3, -Math.PI / 4);
check('Field2d Robot', ctx.dkPoses({val: ctx.dkDec(6, robot)}).map(q => q.slice(0, 3)),
      [[1.5, 2.25, Math.PI / 2]]);
check('Field2d Targets', ctx.dkPoses({val: ctx.dkDec(6, targets)}).map(q => q.slice(0, 3)),
      [[4, 0.5, Math.PI], [6.25, 3, -Math.PI / 4]]);
check('Field2d Robot deg', ctx.dkDeg(ctx.dkPoses({val: robot})[0][2]), 90);
check('Field2d 25 bytes refused', ctx.dkPoses({val: new Uint8Array(25)}), null);
check('Field2d empty array', ctx.dkPoses({val: new Uint8Array(0)}), []);
// core's pose encoder, used for every write of the whole array (the typed pose list; a drag keeps
// each pose's rotation and moves only x and y), equals prosdk's PosesBytes({{3, 1, 0.5}})
check('Field2d pose write bytes', hex(ctx.dkPoseBytes([[3, 1, 0.5]])), hex(f64le(3, 1, 0.5)));

// ---- Scheduler: Names string[] (vt 11), Ids int[] (vt 8), Cancel int[] write ----
check('Scheduler Names', ctx.dkDec(11, strArr('Drive', 'Intake')), ['Drive', 'Intake']);
check('Scheduler Ids', ctx.dkDec(8, i64le(1070000000, 1070000064)).map(String),
      ['1070000000', '1070000064']);
// a cancel write of two ids (as text, the DS keeps them exact) equals prosdk's EncodeIntArray
check('Scheduler Cancel write', hex(ctx.dkEnc(8, ['1070000064', '12345'])),
      hex(i64le(1070000064, 12345)));
// ids past 2^53 (a 64-bit host pointer) stay exact
check('Scheduler big id', String(ctx.dkDec(8, i64le('140737488355328123'))[0]),
      '140737488355328123');
check('Scheduler big id write', hex(ctx.dkEnc(8, ['140737488355328123'])),
      hex(i64le('140737488355328123')));
check('Scheduler Cancel echo []', ctx.dkDec(8, new Uint8Array(0)), []);

// ---- Mechanism2d: the golden tree of WidgetLayoutTest ----
function leaf(vt, bytes) { return {k: {}, t: {vt: vt, val: ctx.dkDec(vt, bytes)}}; }
function dir(kids) { return {k: kids}; }
const str = s => leaf(5, new TextEncoder().encode(s));
const dbl = x => leaf(4, f64le(x));
const darr = (...xs) => leaf(10, f64le(...xs));
function lig(angle, color, length, weight, kids = {}) {
  return dir(Object.assign({'.type': str('line'), angle: dbl(angle), color: str(color),
                            length: dbl(length), weight: dbl(weight)}, kids));
}
const mech = dir({
  '.type': str('Mechanism2d'),
  backgroundColor: str('#000020'),
  base: dir({position: darr(2.5, 0), post: lig(90, '#EB8934', 1.5, 6)}),
  dims: darr(3, 2),
  shoulder: dir({position: darr(1, 0.5),
                 arm: lig(45, '#FF0000', 1.25, 4, {wrist: lig(-30, '#0080FF', 0.5, 2.5)})}),
});
const cv = ctx.dkMech(mech);
check('Mechanism2d decoded', cv !== null, true);
// ui(): the aria-label lists every ligament with its absolute angle and length
mech.k.dims.t.ui();
check('Mechanism2d summary', cv.attrs['aria-label'],
      'Mekanizma: base/post: 90°, 1.5; shoulder/arm: 45°, 1.25; shoulder/arm/wrist: 15°, 0.5');
// draw on a 300 x 200 canvas (dims 3 x 2 m: 100 px per m, origin bottom left)
const segs = [];
let cur = {};
const g = {
  set fillStyle(v) {}, fillRect() {}, set lineCap(v) {},
  set strokeStyle(v) { cur.color = v; }, set lineWidth(v) { cur.width = v; },
  beginPath() {}, moveTo(x, y) { cur.from = [x, y]; },
  lineTo(x, y) { cur.to = [x, y]; }, stroke() { segs.push(cur); cur = {}; },
};
cv.draw(g, 300, 200);
const r = v => Math.round(v * 1000) / 1000;
check('Mechanism2d segments', segs.map(s => [s.color, s.width, s.from.map(r), s.to.map(r)]), [
  ['#EB8934', 6, [250, 200], [250, 50]],
  ['#FF0000', 4, [100, 150], [r(100 + 125 * Math.SQRT1_2), r(150 - 125 * Math.SQRT1_2)]],
  ['#0080FF', 2.5, [r(100 + 125 * Math.SQRT1_2), r(150 - 125 * Math.SQRT1_2)],
   [r(100 + 125 * Math.SQRT1_2 + 50 * Math.cos(Math.PI / 12)),
    r(150 - 125 * Math.SQRT1_2 - 50 * Math.sin(Math.PI / 12))]],
]);

console.log(fail ? fail + ' FAILED' : 'all golden layouts decode as prosdk means them');
process.exit(fail ? 1 : 0);
