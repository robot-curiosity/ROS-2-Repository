"""The single web page of the tuning dashboard (no external files)."""

PAGE = """<!doctype html>
<html lang="en"><head><meta charset="utf-8">
<meta name="viewport" content="width=device-width, initial-scale=1">
<title>Curiosity tuning</title>
<style>
body { font-family: sans-serif; margin: 16px; }
table { border-collapse: collapse; margin-bottom: 16px; }
td, th { border: 1px solid #999; padding: 4px 8px; text-align: left; }
.old { color: #999; }
input[type=number] { width: 6em; }
pre { background: #eee; color: #000; padding: 8px; overflow-x: auto; }
</style></head><body>
<h1>Curiosity tuning</h1>
<p id="mode">connecting...</p>
<h2>Status</h2><table id="status"></table>
<h2>Sonars (m)</h2><table id="sonar"></table>
<h2>Wheels (m/s)</h2><table id="wheels"></table>
<h2>Detection</h2><pre id="det">none</pre>
<h2>Recent safety stops</h2><table id="stops"></table>
<h2>Parameters</h2><p id="msg"></p><table id="params"></table>
<p>Changes are lost when the behavior node restarts. To keep them, copy these
lines into the behavior section of config/robot.yaml:</p>
<pre id="yaml"></pre>
<script>
const el = id => document.getElementById(id);
function row(table, cells, old) {
  const tr = table.insertRow();
  if (old) tr.className = 'old';
  for (const c of cells) tr.insertCell().textContent = c;
  return tr;
}
function fill(id, rows) {
  el(id).textContent = '';
  for (const r of rows) row(el(id), r[0], r[1]);
}
const num = v => typeof v === 'number' ? v.toFixed(2) : String(v);
const edits = {};
function buildParams(s) {
  for (const [name, t] of Object.entries(s.tunables)) {
    const tr = row(el('params'), [t.label + ' (' + t.unit + ')', '-']);
    const input = document.createElement('input');
    input.type = 'number';
    input.min = t.min; input.max = t.max; input.step = t.step;
    const button = document.createElement('button');
    button.textContent = 'Set';
    button.onclick = () => send(name, input.value);
    input.disabled = button.disabled = !s.allow_write;
    tr.insertCell().appendChild(input);
    tr.insertCell().appendChild(button);
    edits[name] = tr.cells[1];
  }
}
async function send(name, text) {
  if (text === '') return;
  try {
    const r = await fetch('/api/params', {method: 'POST',
      headers: {'Content-Type': 'application/json'},
      body: JSON.stringify({[name]: Number(text)})});
    const reply = await r.json();
    el('msg').textContent = name + ': ' + (reply.message || reply.error);
  } catch (e) { el('msg').textContent = 'no answer from the robot'; }
}
function show(s) {
  el('mode').textContent = s.allow_write ?
    'Writing is ON: anyone on this network can change parameters.' :
    'Read-only.';
  const one = (key, label) => s[key] ?
    [[label, String(s[key].value), s[key].age + ' s ago'], s[key].age > 2] :
    [[label, '-', 'no data'], true];
  fill('status', [one('state', 'Behavior state'), one('safety', 'Safety'),
                  one('fps', 'Detector FPS')]);
  fill('sonar', Object.keys(s).filter(k => k.startsWith('sonar/')).sort()
    .map(k => [[k.slice(6), num(s[k].value), s[k].age + ' s ago'],
               s[k].age > 2]));
  fill('wheels', Object.entries(s.wheels).map(([k, w]) =>
    [[k, 'L ' + num(w.left), 'R ' + num(w.right)], w.age > 1]));
  const d = s.detection;
  el('det').textContent = d ? 'base_x: ' + d.value.base_x + '  base_y: ' +
    d.value.base_y + '  (' + d.age + ' s ago)\\n' +
    JSON.stringify(d.value, null, 1) : 'none';
  fill('stops', s.stops.map(x => [[x.reason, x.age + ' s ago']]));
  if (!el('params').rows.length) buildParams(s);
  const p = s.params && s.params.age < 5 ? s.params.value : {};
  for (const name in edits) {
    edits[name].textContent = name in p ? String(p[name]) : '-';
  }
  el('yaml').textContent = Object.keys(s.tunables).filter(n => n in p)
    .map(n => '    ' + n + ': ' +
         (Number.isInteger(p[n]) ? p[n].toFixed(1) : p[n])).join('\\n');
}
async function poll() {
  try {
    show(await (await fetch('/api/state')).json());
  } catch (e) { el('mode').textContent = 'no answer from the robot'; }
  setTimeout(poll, 500);
}
poll();
</script></body></html>
"""
