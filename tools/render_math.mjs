// Usage: node tools/render_math.mjs <katex package folder> [--vendor]
import fs from 'fs';
import path from 'path';
import { fileURLToPath, pathToFileURL } from 'url';

const [katexDir, flag] = process.argv.slice(2);
if (!katexDir) {
  console.error('usage: node tools/render_math.mjs <katex package folder> [--vendor]');
  process.exit(2);
}
const katex = (await import(pathToFileURL(path.resolve(katexDir, 'dist/katex.mjs')).href)).default;
const root = path.join(path.dirname(fileURLToPath(import.meta.url)), '..');

const unescape = (s) => s.replace(/&quot;/g, '"').replace(/&#x27;/g, "'").replace(/&lt;/g, '<').replace(/&gt;/g, '>').replace(/&amp;/g, '&');

function closingTag(html, from, tag) {
  const re = new RegExp(`<(/?)${tag}\\b[^>]*>`, 'g');
  re.lastIndex = from;
  for (let depth = 1, m; (m = re.exec(html));) {
    depth += m[1] ? -1 : 1;
    if (!depth) return m.index;
  }
  throw new Error(`unclosed <${tag}> at ${from}`);
}

const file = path.join(root, 'index.html');
let html = fs.readFileSync(file, 'utf8'), out = '', at = 0, count = 0;
const open = /<(div|span) class="[^"]*\btex\b[^"]*" data-tex="([^"]*)"( data-display="true")?>/g;
for (let m; (m = open.exec(html));) {
  const start = m.index + m[0].length, end = closingTag(html, start, m[1]);
  out += html.slice(at, start) + katex.renderToString(unescape(m[2]), { displayMode: !!m[3], throwOnError: true, strict: 'error', output: 'htmlAndMathml' });
  at = end;
  open.lastIndex = end;
  count++;
}
fs.writeFileSync(file, out + html.slice(at));
console.log(`rendered ${count} formulas with KaTeX ${katex.version}`);

if (flag === '--vendor') {
  const dir = path.join(root, 'vendor', 'katex');
  fs.mkdirSync(path.join(dir, 'fonts'), { recursive: true });
  const css = fs.readFileSync(path.resolve(katexDir, 'dist/katex.min.css'), 'utf8')
    .replace(/src:url\((fonts\/[^)]+\.woff2)\) format\("woff2"\)(,url\([^)]+\) format\("[a-z]+"\))*/g, 'src:url($1) format("woff2")');
  fs.writeFileSync(path.join(dir, 'katex.min.css'), css);
  for (const f of fs.readdirSync(path.resolve(katexDir, 'dist/fonts'))) {
    if (f.endsWith('.woff2')) fs.copyFileSync(path.resolve(katexDir, 'dist/fonts', f), path.join(dir, 'fonts', f));
  }
  fs.copyFileSync(path.resolve(katexDir, 'LICENSE'), path.join(dir, 'LICENSE'));
}
