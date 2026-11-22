// Headless browser check of the cave: loads ?env=cave, fast-forwards the sim, screenshots every view. Needs dev server on :5173.
import puppeteer from 'puppeteer-core';
const browser = await puppeteer.launch({
  executablePath: 'C:/Program Files/Google/Chrome/Application/chrome.exe', headless: 'new',
  args: ['--use-angle=swiftshader', '--enable-unsafe-swiftshader', '--ignore-gpu-blocklist'],
});
const page = await browser.newPage();
await page.setViewport({ width: 1400, height: 850 });
const errors = [];
page.on('pageerror', (e) => errors.push(e.message));
page.on('console', (m) => { if (m.type() === 'error') errors.push(m.text()); if (m.text().startsWith('[cave]')) console.log(m.text()); });
await page.goto('http://localhost:5173/?env=cave', { waitUntil: 'load' });
await page.waitForFunction(() => window.__cave, { timeout: 90000 });
await new Promise((r) => setTimeout(r, 2500));
const shot = async (name, wait = 1800) => { await new Promise((r) => setTimeout(r, wait)); await page.screenshot({ path: `screenshots/cave_${name}.png` }); };
const click = (sel) => page.click(sel);
await shot('iso_roof');
await page.evaluate(() => { const c = document.getElementById('cave-roof'); c.checked = false; c.dispatchEvent(new Event('change', { bubbles: true })); });
await shot('iso_noroof');
await page.evaluate(() => window.__cave.setView('top')); await shot('top');
await page.evaluate(() => window.__cave.mapper.advance(120));
await page.evaluate(() => window.__cave.setView('isometric')); await shot('iso_mapping');
await page.evaluate(() => { for (const k of ['sensors', 'boxes', 'axes']) { const c = document.querySelector(`[data-debug=${k}]`); c.checked = true; c.dispatchEvent(new Event('change', { bubbles: true })); } const t = document.getElementById('cave-tint'); t.checked = true; t.dispatchEvent(new Event('change', { bubbles: true })); });
await shot('debug_tint');
await page.evaluate(() => window.__cave.setView('chase')); await shot('chase');
await page.evaluate(() => window.__cave.setView('fpv')); await shot('fpv');
console.log(JSON.stringify(await page.evaluate(() => window.__cave.mapper.stats())));
console.log(errors.length ? errors.join('\n') : 'no errors');
await browser.close();
