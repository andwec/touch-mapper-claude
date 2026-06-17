'use strict';

const express = require('express');
const path = require('path');
const fs = require('fs');
const { spawn } = require('child_process');

const app = express();
const PORT = process.env.PORT || 3000;

// All generated map files live here
const DATA_DIR = path.join(__dirname, 'local-data', 'maps');
fs.mkdirSync(path.join(DATA_DIR, 'info'), { recursive: true });
fs.mkdirSync(path.join(DATA_DIR, 'data'), { recursive: true });

app.use(express.json({ limit: '1mb' }));

// Serve generated map files (replaces S3/CloudFront).
// Files are written by run-local.py into local-data/maps/info/ and local-data/maps/data/.
app.use('/map', express.static(DATA_DIR, {
  setHeaders(res, filePath) {
    if (filePath.endsWith('.json')) {
      res.setHeader('Cache-Control', 'no-cache');
    }
  },
}));

// Map creation endpoint — replaces the AWS SQS SendMessage call.
app.post('/api/create-map', (req, res) => {
  const requestBody = req.body;
  const requestId = requestBody && requestBody.requestId;
  if (!requestId) {
    return res.status(400).json({ error: 'missing requestId' });
  }

  const idStart = requestId.split('/')[0];
  const infoPath = path.join(DATA_DIR, 'info', idStart + '.json');

  fs.writeFileSync(infoPath, JSON.stringify({ requestId, status: { progress: 0 } }));
  res.json({ ok: true });

  const converterScript = path.join(__dirname, 'converter', 'run-local.py');
  const python = process.platform === 'win32' ? 'python' : 'python3';

  const child = spawn(python, [
    converterScript,
    '--request', JSON.stringify(requestBody),
    '--data-root', DATA_DIR,
  ], {
    stdio: ['ignore', 'pipe', 'pipe'],
    cwd: path.join(__dirname, 'converter'),
  });

  child.stdout.on('data', d => process.stdout.write('[converter] ' + d));
  child.stderr.on('data', d => process.stderr.write('[converter] ' + d));

  child.on('close', code => {
    if (code === 0) return;
    try {
      const info = JSON.parse(fs.readFileSync(infoPath, 'utf8'));
      if (!info.status || !info.status.errorCode) {
        info.status = { progress: 0, errorCode: 'unknown', errorDescription: 'Converter exited with code ' + code };
        fs.writeFileSync(infoPath, JSON.stringify(info));
      }
    } catch (_) {
      fs.writeFileSync(infoPath, JSON.stringify({
        requestId,
        status: { progress: 0, errorCode: 'unknown', errorDescription: 'Converter failed (exit ' + code + ')' },
      }));
    }
  });
});

// Serve the web UI (must come after API routes).
// extensions: ['html'] lets /en/map resolve to en/map.html
app.use(express.static(path.join(__dirname, 'web', 'build'), { extensions: ['html'] }));

app.listen(PORT, () => {
  console.log('Touch Mapper local server: http://localhost:' + PORT);
  console.log('');
  console.log('Build the web UI first if you have not already:');
  console.log('  cd web && python pre2src.py && node build.js');
});
