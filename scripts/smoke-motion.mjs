// Headless check that the robot actually moves along the patrol loop (needs dev server on :5173).
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
const read = () => page.evaluate(() => { const p = window.__env.driver.pose; return { x: +p.x.toFixed(2), y: +p.y.toFixed(2), yaw: +p.yaw.toFixed(2), v: +p.v.toFixed(2) }; });
const samples = [];
for (let i = 0; i < 6; i++) { samples.push(await read()); await new Promise((r) => setTimeout(r, 2500)); }
console.log(samples.map((s) => JSON.stringify(s)).join('\n'));
await page.screenshot({ path: 'screenshots/moving.png' });
console.log(errors.length ? errors.join('\n') : 'no errors');
await browser.close();
