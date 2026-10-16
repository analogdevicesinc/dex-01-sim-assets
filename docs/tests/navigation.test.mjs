// Copyright (c) 2026 Analog Devices, Inc. All Rights Reserved.
// SPDX-License-Identifier: Apache-2.0

import assert from 'node:assert/strict';
import {readFileSync, writeFileSync, mkdtempSync, rmSync} from 'node:fs';
import os from 'node:os';
import path from 'node:path';
import {pathToFileURL} from 'node:url';
import {test, after} from 'node:test';
import {JSDOM} from 'jsdom';

const site = process.env.DOCS_SITE_ROOT;
assert.ok(site, 'DOCS_SITE_ROOT must point to a freshly built documentation site');
// Use the source map shipped by the pinned ADI bundle, not a reimplementation
// of its navigation. Extract ES modules into a disposable Node-only directory.
const sourceMap = JSON.parse(readFileSync(path.join(site, '_static/app.umd.js.map'), 'utf8'));
const modules = mkdtempSync(path.join(os.tmpdir(), 'adi-navigation-test-'));
writeFileSync(path.join(modules, 'package.json'), '{"type":"module"}');
for (let index = 0; index < sourceMap.sources.length; index++) {
  const filename = path.basename(sourceMap.sources[index]);
  writeFileSync(path.join(modules, filename), sourceMap.sourcesContent[index]);
}
const {HotReload} = await import(pathToFileURL(path.join(modules, 'hot_reload.js')).href);
after(() => rmSync(modules, {recursive: true, force: true}));
const navigationSource = readFileSync(new URL('../_static/environment-navigation.js', import.meta.url), 'utf8');

function page(file, prefix = '/main/') {
  const url = new URL(prefix + (file === 'index' ? '' : file.replace(/\/index$/, '') + '/'), 'https://docs.example/');
  return {url, html: readFileSync(path.join(site, file === 'index' ? 'index.html' : file.endsWith('/index') ? file + '.html' : file + '/index.html'), 'utf8')};
}

function harness(start) {
  const dom = new JSDOM(start.html, {url: start.url, runScripts: 'outside-only'});
  const {window} = dom;
  for (const name of ['window', 'document', 'location', 'history', 'CustomEvent', 'HTMLElement']) globalThis[name] = window[name];
  globalThis.DOMParser = window.DOMParser;
  window.HTMLElement.prototype.scrollIntoView = () => {};
  window.scrollTo = () => {};
  const html = window.document;
  const navigation = Object.create(HotReload.prototype);
  navigation.href = String(start.url);
  navigation.toctree = new Map();
  navigation.js_script_memory = new Map();
  navigation.body_classes = new Set();
  navigation.reduced_motion = true;
  navigation.load_failed = false;
  navigation.state_helper = {init_state(state, target) {
    state.content_root = target.documentElement.dataset.content_root;
    state.metadata = {repotoc: {}};
  }};
  navigation.parent = {
    state: {content_root: html.documentElement.dataset.content_root, metadata: {repotoc: {}}},
    versioned: {reset() {}, construct() {}},
    links: {update_repotoc() {}},
  };
  navigation.$ = {
    toctree: html.querySelector('.sphinxsidebar .toc-tree'),
    sidebarwrapper: html.querySelector('.sphinxsidebarwrapper'),
    sidebar_logo: html.querySelector('.sphinxsidebarwrapper > a'),
    header_logo: html.querySelector('header a#logo'),
    content: html.querySelector('.documentwrapper .body'),
    localtoc: html.querySelector('.localtoc nav'),
    related: html.querySelector('.documentwrapper .related'),
    breadcrumb: html.querySelector('.bodywrapper .body-header .breadcrumb'),
    title: html.querySelector('head title'),
  };
  navigation.sync_styles = () => {};
  navigation.sync_scripts = () => {};
  navigation.clear_fetch = () => {};
  navigation.push_history = (url, state) => window.history.replaceState(state, '', url);
  navigation.is_known_repository = () => true;
  navigation.init_toctree();
  window.app = {hot_reload: navigation};
  window.eval(navigationSource);
  html.dispatchEvent(new window.Event('DOMContentLoaded'));
  return {dom, window, navigation};
}

async function navigate(context, target, state = false) {
  const document = new context.window.DOMParser().parseFromString(target.html, 'text/html');
  assert.equal(await context.navigation.replace(target.url, document, false, state), true);
  await new Promise(resolve => setImmediate(resolve));
}

function assertEnvironment(context, environment) {
  const current = context.window.document.querySelector('.environment-choice [aria-current]');
  assert.equal(current?.dataset.docTarget || '', environment);
  const tree = context.window.document.querySelector('.sphinxsidebar .toc-tree');
  const urls = [...tree.querySelectorAll('a[href]')].map(link => new URL(link.href));
  if (environment) {
    const other = environment === 'newton' ? 'physx' : 'newton';
    assert.ok(urls.some(url => url.pathname.endsWith('/' + environment + '/installation/')));
    assert.ok(urls.some(url => url.pathname.endsWith('/' + environment + '/validation/')));
    assert.ok(!urls.some(url => url.pathname.includes('/' + other + '/')));
  } else {
    assert.ok(urls.some(url => url.pathname.endsWith('/newton/')));
    assert.ok(urls.some(url => url.pathname.endsWith('/physx/')));
  }
}

test('ADI same-build replacement follows chooser, backend, shared pages and history', async () => {
  const context = harness(page('index'));
  await navigate(context, page('newton/index'));
  assertEnvironment(context, 'newton');
  await navigate(context, page('physx/api'));
  assertEnvironment(context, 'physx');
  await navigate(context, page('reference/data'));
  assertEnvironment(context, '');
  await navigate(context, page('newton/examples/hand'), {scrollY: 0});
  assertEnvironment(context, 'newton');
  assert.ok(context.navigation.toc_get(page('newton/installation').url));
  context.dom.window.close();
});

test('ADI version replacement restores destination scope after the global tree loads', async () => {
  const context = harness(page('newton/api'));
  globalThis.fetch = async () => ({ok: true, text: async () => readFileSync(path.join(site, '_toctree.html'), 'utf8')});
  await navigate(context, page('physx/examples/hand', '/v0.2.0/'));
  assertEnvironment(context, 'physx');
  assert.equal(context.window.document.querySelector('[data-doc-target="newton"]').href, 'https://docs.example/v0.2.0/newton/');
  context.dom.window.close();
});

test('development tree refresh retains version widget and active environment', async () => {
  const context = harness(page('newton/api'));
  const sidebar = context.window.document.querySelector('.toc-tree');
  const widget = context.window.document.createElement('div');
  widget.className = 'version-dropdown';
  widget.textContent = 'main';
  sidebar.prepend(widget);
  await new Promise(resolve => setImmediate(resolve));
  context.navigation.set_toctree('<ul><li><a href="physx/">Global refreshed tree</a></li></ul>', 'https://docs.example/main/');
  await new Promise(resolve => setImmediate(resolve));
  assertEnvironment(context, 'newton');
  assert.equal(context.window.document.querySelectorAll('.version-dropdown').length, 1);
  context.dom.window.close();
});


function historicalPage() {
  const source = process.env.DOCS_HISTORICAL_ROOT;
  assert.ok(source, 'DOCS_HISTORICAL_ROOT must point to the built immutable prerelease README');
  return {url: new URL('https://docs.example/v0.1.0-a.1/'), html: readFileSync(path.join(source, 'index.html'), 'utf8')};
}

test('current environment to actual historical prerelease removes modern choices', async () => {
  const context = harness(page('newton/api'));
  globalThis.fetch = async () => ({ok: true, text: async () => readFileSync(path.join(process.env.DOCS_HISTORICAL_ROOT, '_toctree.html'), 'utf8')});
  await navigate(context, historicalPage());
  assert.equal(context.window.document.querySelector('.environment-choice'), null);
  assert.equal(context.window.document.querySelector('meta[name="docs-environment"]'), null);
  context.dom.window.close();
});

test('actual historical prerelease to current environment creates usable choices', async () => {
  const context = harness(historicalPage());
  assert.equal(context.window.document.querySelector('.environment-choice'), null);
  globalThis.fetch = async () => ({ok: true, text: async () => readFileSync(path.join(site, '_toctree.html'), 'utf8')});
  await navigate(context, page('newton/api'));
  assertEnvironment(context, 'newton');
  assert.equal(context.window.document.querySelector('[data-doc-target="physx"]').href, 'https://docs.example/main/physx/');
  context.dom.window.close();
});
