// Opens the algorithm guide, cycles every tab (live drawings), screenshots a few. Needs dev server on :5173.
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
await page.evaluate(() => window.__env.mapper.advance(90));
await page.click('#algo-btn');
for (const id of ['overview', 'lidar', 'occ', 'frontier', 'plan', 'control', 'ros']) {
  await page.click(`nav button[data-tab="${id}"]`);
  await new Promise((r) => setTimeout(r, 900));
  if (['lidar', 'frontier', 'plan', 'control'].includes(id)) await page.screenshot({ path: `screenshots/guide_${id}.png` });
}
console.log(errors.length ? errors.join('\n') : 'no errors');
await browser.close();
