import { defineConfig } from 'vite';

export default defineConfig({
  base: './',
  publicDir: 'webinteractive/model',
  build: { chunkSizeWarningLimit: 800 },
});
