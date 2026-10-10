// Usage: node render.js <html> <outDir> <fps> [onlyTimes...]
const { chromium } = require('playwright');
const path = require('path');
const fs = require('fs');

(async () => {
  const [html, outDir, fpsArg, ...only] = process.argv.slice(2);
  const fps = Number(fpsArg);
  fs.mkdirSync(outDir, { recursive: true });
  const browser = await chromium.launch();
  const page = await browser.newPage({ viewport: { width: 1920, height: 1080 } });
  await page.goto('file://' + path.resolve(html));
  await page.evaluate(() => document.fonts.ready);
  const total = await page.evaluate(() => window.TOTAL);
  const times = only.length ? only.map(Number) : [...Array(Math.round(total * fps)).keys()].map(i => i / fps);
  for (let i = 0; i < times.length; i++) {
    await page.evaluate(t => window.render(t), times[i]);
    await page.screenshot({ path: path.join(outDir, `f_${String(i).padStart(4, '0')}.png`) });
  }
  await browser.close();
  console.log(`rendered ${times.length} frames`);
})();
