/**
 * Development server for Tagcheck
 * Serves static files and provides API endpoint for tags.toml data
 */

import { readFileSync } from 'fs';
import { parse as parseToml } from '@iarna/toml';
import { join, dirname } from 'path';

const TAGS_TOML_PATH = join(import.meta.dir, '..', 'interchange.toml.d', 'tags.toml');
const PUBLIC_DIR = join(import.meta.dir, 'public');

// Load and parse tags.toml
function loadTags() {
  try {
    const content = readFileSync(TAGS_TOML_PATH, 'utf-8');
    return parseToml(content);
  } catch (error) {
    console.error('Failed to load tags.toml:', error);
    return null;
  }
}

const tagsData = loadTags();

const server = Bun.serve({
  port: 3000,
  async fetch(req) {
    const url = new URL(req.url);
    const path = url.pathname;
    
    // API endpoint for tags data
    if (path === '/api/tags') {
      if (!tagsData) {
        return new Response(JSON.stringify({ error: 'Failed to load tags' }), {
          status: 500,
          headers: { 'Content-Type': 'application/json' }
        });
      }
      return new Response(JSON.stringify(tagsData), {
        headers: { 'Content-Type': 'application/json' }
      });
    }
    
    // Serve static files
    let filePath = path === '/' ? '/index.html' : path;
    
    // Handle /dist/ paths for bundled JS
    if (filePath.startsWith('/dist/')) {
      const distPath = join(import.meta.dir, filePath);
      try {
        const file = Bun.file(distPath);
        if (await file.exists()) {
          return new Response(file, {
            headers: { 'Content-Type': 'application/javascript' }
          });
        }
      } catch (e) {
        // Fall through to 404
      }
    }
    
    // Serve from public directory
    const publicPath = join(PUBLIC_DIR, filePath);
    try {
      const file = Bun.file(publicPath);
      if (await file.exists()) {
        const ext = filePath.split('.').pop();
        const contentTypes: Record<string, string> = {
          'html': 'text/html',
          'css': 'text/css',
          'js': 'application/javascript',
          'json': 'application/json',
          'png': 'image/png',
          'svg': 'image/svg+xml',
        };
        return new Response(file, {
          headers: { 'Content-Type': contentTypes[ext || ''] || 'text/plain' }
        });
      }
    } catch (e) {
      // Fall through to 404
    }
    
    return new Response('Not Found', { status: 404 });
  }
});

console.log(`🏷️  Tagcheck server running at http://localhost:${server.port}`);
console.log(`📁 Serving from: ${PUBLIC_DIR}`);
console.log(`📄 Tags from: ${TAGS_TOML_PATH}`);
