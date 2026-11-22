// Screenshot of the LiDAR visualisation close to the robot (needs dev server on :5173).
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
await page.evaluate(() => { window.__env.mapper.advance(25.03); });
await page.evaluate(() => {
  const e = window.__env, p = e.driver.pose;
  const back = [p.x - 0.5, p.y - 3.0, 15];
  e.camera.fov = 45; e.camera.updateProjectionMatrix();
  e.camera.position.set(back[0], back[2], -back[1]);
  e.cameraRig.controls.target.set(p.x, 0.3, -p.y);
  e.cameraRig.controls.update();
});
await new Promise((r) => setTimeout(r, 2500));
await page.screenshot({ path: 'screenshots/lidar_viz.png' });
console.log(errors.length ? errors.join('\n') : 'no errors');
await browser.close();
