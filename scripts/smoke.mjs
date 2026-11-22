// Headless smoke test: loads the dev server, collects console errors, saves screenshots of several views.
import puppeteer from 'puppeteer-core';
import { mkdirSync } from 'node:fs';
const url = process.argv[2] ?? 'http://localhost:5173/';
const out = process.argv[3] ?? 'screenshots';
mkdirSync(out, { recursive: true });
const browser = await puppeteer.launch({
  executablePath: 'C:/Program Files/Google/Chrome/Application/chrome.exe',
  headless: 'new', args: ['--use-angle=swiftshader', '--enable-unsafe-swiftshader', '--ignore-gpu-blocklist'],
});
const page = await browser.newPage();
await page.setViewport({ width: 1400, height: 850 });
const errors = [];
page.on('console', (m) => { if (['error', 'warning'].includes(m.type())) errors.push(`${m.type()}: ${m.text()}`); });
page.on('pageerror', (e) => errors.push(`pageerror: ${e.message}`));
await page.goto(url, { waitUntil: 'load' });
await new Promise((r) => setTimeout(r, 3000));
const shot = async (name) => { await new Promise((r) => setTimeout(r, 1200)); await page.screenshot({ path: `${out}/${name}.png` }); };
await shot('iso');
await page.evaluate(() => window.__env.cameraRig.setView('top')); await shot('top');
await page.evaluate(() => window.__env.cameraRig.setView('robot')); await shot('robot');
await page.evaluate(() => { window.__env.cameraRig.reset(); ['axes','grid','boxes','labels','robot','path'].forEach((k) => window.__env.debug.setPart(k, true)); }); await shot('debug');
await page.evaluate(() => { ['axes','grid','boxes','labels','robot','path'].forEach((k) => window.__env.debug.setPart(k, false)); window.__env.setSemantic(true); }); await shot('semantic');
await page.evaluate(() => { window.__env.setSemantic(false); window.__env.setAO(true); }); await shot('ao');
console.log('stats', JSON.stringify(await page.evaluate(() => window.__env.stats)));
console.log(errors.length ? errors.join('\n') : 'no console errors');
await browser.close();
