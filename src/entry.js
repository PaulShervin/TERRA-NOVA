// Application entry. Picks the environment from the URL (?env=cave) and loads it. The existing industrial
// facility (src/main.js) is imported UNCHANGED when ?env is absent / "industrial"; the cave lives in src/cave/.
import './envSwitch.css';

const params = new URLSearchParams(window.location.search);
const env = params.get('env') === 'cave' ? 'cave' : 'industrial';

const sw = document.createElement('div');
sw.id = 'env-switch';
sw.innerHTML = `<label>Environment
  <select id="env-select">
    <option value="industrial">Industrial facility</option>
    <option value="cave">Underground cave (GPS-denied SLAM)</option>
  </select></label>`;
document.body.appendChild(sw);
const select = sw.querySelector('select');
select.value = env;
select.addEventListener('change', () => {
  const url = new URL(window.location.href);
  if (select.value === 'cave') url.searchParams.set('env', 'cave'); else url.searchParams.delete('env');
  window.location.href = url.toString(); // full reload keeps the two environments completely independent
});

if (env === 'cave') await import('./cave/caveMain.js');
else await import('./main.js');
