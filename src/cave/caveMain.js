// Entry for the cave environment (loaded only when ?env=cave). The industrial main.js is never imported here.
import '../ui.css';
import './cave.css';
import { createCaveEnvironment } from './CaveEnvironment.js';

document.body.classList.add('cave-mode');
const env = await createCaveEnvironment(document.getElementById('app'));
window.__cave = env; // handy for console inspection and the headless smoke tests
