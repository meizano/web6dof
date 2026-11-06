import { defineConfig } from 'vite';
import basicSsl from '@vitejs/plugin-basic-ssl';

// HTTPS=1 npm run dev  -> self-signed HTTPS on your LAN IP (phone must accept the cert)
// npm run dev          -> plain HTTP; use `adb reverse tcp:5173 tcp:5173` and open
//                         http://localhost:5173 on the phone (localhost counts as secure)
const https = !!process.env.HTTPS;

export default defineConfig(({ command }) => ({
  // Relative asset paths in builds, so dist/ works under any sub-path
  // (e.g. https://<user>.github.io/web6dof/).
  base: command === 'build' ? './' : '/',
  plugins: https ? [basicSsl()] : [],
  server: { host: true, port: 5173 },
  worker: { format: 'es' },
  build: {
    target: 'es2022',
    rollupOptions: { input: { main: 'index.html', marker: 'marker.html' } },
  },
}));
