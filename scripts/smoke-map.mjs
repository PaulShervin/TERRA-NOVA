// Headless check of live mapping in the browser: fast-forwards the sim, then screenshots + prints map stats.
import puppeteer from 'puppeteer-core';
const browser = await puppeteer.launch({
  executablePath: 'C:/Program Files/Google/Chrome/Application/chrome.exe', headless: 'new',
  args: ['--use-angle=swiftshader', '--enable-unsafe-swiftshader', '--ignore-gpu-blocklist'],
});
const page = await browser.newPage();
await page.setViewport({ width: 1400, height: 850 });
const errors = [];
page.on('pageerror', (e) => errors.push(e.message));
page.on('console', (m) => { if (m.type() === 'error') errors.push(m.text()); });
await page.goto('http://localhost:5173/', { waitUntil: 'load' });
await new Promise((r) => setTimeout(r, 2500));
for (const [name, secs] of [['map_t0', 0], ['map_t60', 60], ['map_t200', 140], ['map_t_done', 700]]) {
  if (secs) await page.evaluate((s) => window.__env.mapper.advance(s), secs);
  await new Promise((r) => setTimeout(r, 1800));
  const st = await page.evaluate(() => window.__env.mapper.stats());
  const pz = await page.evaluate(() => { const p = window.__env.driver.pose; return [+p.x.toFixed(1), +p.y.toFixed(1)]; });
  console.log(name, JSON.stringify({ pose: pz, ...st, coverage: +st.coverage.toFixed(3), travelled: Math.round(st.travelled) }));
  await page.screenshot({ path: `screenshots/${name}.png` });
}
console.log(errors.length ? errors.join('\n') : 'no errors');
await browser.close();
