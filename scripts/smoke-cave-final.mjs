// Runs the cave exploration to completion in the browser, then screenshots roof on/off + reports loop candidates.
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
await page.goto('http://localhost:5173/?env=cave', { waitUntil: 'load' });
await page.waitForFunction(() => window.__cave, { timeout: 90000 });
await new Promise((r) => setTimeout(r, 2000));
for (let i = 0; i < 4; i++) { await page.evaluate(() => window.__cave.mapper.advance(150)); }
await new Promise((r) => setTimeout(r, 2500));
console.log(JSON.stringify(await page.evaluate(() => ({ ...window.__cave.mapper.stats(), loopMarkers: window.__cave.slam.parts.loops.children.length / 2, trail: window.__cave.mapper.trajectory.length }))));
await page.screenshot({ path: 'screenshots/cave_final_noroof.png' });
await page.evaluate(() => { const c = document.getElementById('cave-roof'); c.checked = true; c.dispatchEvent(new Event('change', { bubbles: true })); });
await new Promise((r) => setTimeout(r, 2500));
await page.screenshot({ path: 'screenshots/cave_final_roof.png' });
console.log(errors.length ? errors.join('\n') : 'no errors');
await browser.close();
