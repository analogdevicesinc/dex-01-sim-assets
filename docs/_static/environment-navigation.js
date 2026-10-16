// Copyright (c) 2026 Analog Devices, Inc. All Rights Reserved.
// SPDX-License-Identifier: Apache-2.0

// ADI hot navigation replaces the document body and metadata, but keeps the
// original sidebar for same-build links. Restore the destination page's own
// generated tree after page, history and version navigation.
(() => {
  let versionWidget;
  const restoreNavigation = () => {
    const metadata = document.querySelector('meta[name="docs-navigation-html"]');
    const sidebar = document.querySelector('.sphinxsidebar .toc-tree');
    if (!sidebar) return;
    if (!metadata) {
      document.querySelectorAll('.environment-choice').forEach(chooser => chooser.remove());
      versionWidget = sidebar.querySelector('.version-dropdown');
      return;
    }
    const content = metadata.content;
    const current = sidebar.querySelector('.environment-navigation-content');
    if (current?.dataset.navigationSource !== content) {
      const version = sidebar.querySelector('.version-dropdown') || versionWidget;
      const navigation = window.app?.hot_reload;
      if (navigation?.set_toctree) {
        navigation.set_toctree(content, location.href);
      } else {
        sidebar.innerHTML = content;
        sidebar.querySelectorAll('a[href]').forEach(link => {
          link.href = new URL(link.getAttribute('href'), location.href).href;
        });
      }
      sidebar.querySelector('.environment-navigation-content').dataset.navigationSource = content;
      if (version) sidebar.prepend(version);
    }
    versionWidget = sidebar.querySelector('.version-dropdown') || versionWidget;
    let chooser = document.querySelector('.environment-choice');
    if (!chooser) {
      chooser = document.createElement('nav');
      chooser.className = 'environment-choice';
      chooser.setAttribute('aria-label', 'Simulation environment');
      for (const [target, label] of [['', 'Choose environment'], ['physx', 'PhysX'], ['newton', 'Newton']]) {
        const link = document.createElement('a');
        link.dataset.docTarget = target;
        link.textContent = label;
        chooser.append(link);
      }
      sidebar.parentNode.insertBefore(chooser, sidebar);
    }
    const environment = document.querySelector('meta[name="docs-environment"]')?.content || '';
    const root = new URL(document.documentElement.dataset.content_root || './', location.href);
    document.querySelectorAll('.environment-choice a[data-doc-target]').forEach(link => {
      const target = link.dataset.docTarget;
      link.href = new URL(target ? `${target}/` : './', root).href;
      if (target && target === environment) link.setAttribute('aria-current', 'true');
      else link.removeAttribute('aria-current');
    });
  };
  const observeNavigation = () => {
    restoreNavigation();
    const sidebar = document.querySelector('.sphinxsidebar .toc-tree');
    if (sidebar) new MutationObserver(restoreNavigation).observe(sidebar, {childList: true});
  };
  addEventListener('app:hot_reload:page_loaded', restoreNavigation);
  if (document.readyState === 'loading') {
    addEventListener('DOMContentLoaded', observeNavigation, {once: true});
  } else observeNavigation();
})();
