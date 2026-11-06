import { AR } from './vendor/aruco.js';

const dict = new AR.Dictionary('ARUCO_MIP_36h12');
const idIn = document.getElementById('id');
const sizeIn = document.getElementById('size');

function render() {
  const id = Math.max(0, Math.min(249, parseInt(idIn.value, 10) || 0));
  const cm = Math.max(3, parseFloat(sizeIn.value) || 10);
  const cells = dict.markSize; // black square incl. border, in cells
  const margin = 2; // white quiet zone, in cells (a wide one helps detection)
  const total = cells + margin * 2;
  const code = dict.codeList[id];
  const inner = cells - 2;
  let rects = '';
  for (let y = 0; y < inner; y++)
    for (let x = 0; x < inner; x++)
      if (code[y * inner + x] === '1') rects += `<rect x="${x + margin + 1}" y="${y + margin + 1}" width="1" height="1" fill="#fff"/>`;
  const physical = (cm * total) / cells; // printed size including quiet zone
  document.getElementById('marker').innerHTML =
    `<svg xmlns="http://www.w3.org/2000/svg" viewBox="0 0 ${total} ${total}" width="${physical}cm" height="${physical}cm">` +
    `<rect width="${total}" height="${total}" fill="#fff" stroke="#ccc" stroke-width="0.02"/>` +
    `<rect x="${margin}" y="${margin}" width="${cells}" height="${cells}" fill="#000"/>${rects}</svg>`;
  document.getElementById('caption').textContent =
    `ARUCO_MIP_36h12 #${id} · black square ${cm} cm — measure after printing; this edge goes towards you`;
  document.getElementById('open').href = `./?id=${id}&size=${(cm / 100).toFixed(3)}`;
}

idIn.addEventListener('input', render);
sizeIn.addEventListener('input', render);
render();
