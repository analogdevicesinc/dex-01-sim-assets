// Copyright (c) 2026 Analog Devices, Inc. All Rights Reserved.
// SPDX-License-Identifier: Apache-2.0

// ADI's theme gets navigation/search metadata from its central registry.
// Supply this repository's entry until it is registered there, preserving
// the theme's existing organization entries and its normal rendering.
(() => {
  const originalFetch = window.fetch.bind(window);
  const repository = "dex-01-sim-assets";
  const sourceRef = () => document.querySelector('meta[name="docs-source-ref"]')?.content || "main";
  window.fetch = async (...args) => {
    const response = await originalFetch(...args);
    const url = new URL(args[0] instanceof Request ? args[0].url : String(args[0]), location.href);
    if (!response.ok || !url.pathname.endsWith("/doctools/metadata.json")) return response;
    const metadata = await response.clone().json();
    metadata.repotoc ||= {};
    metadata.repotoc[repository] = {
      ...metadata.repotoc[repository],
      name: "DEX-01 Tactile Sensor",
      description: "Physical normal-force sensing and firmware-mapped tactile simulation for Isaac Lab.",
      category: "software",
      visibility: "public",
      pathname: "docs",
      branch: sourceRef(),
    };
    // Standard raw URLs accept both branches and immutable commit/tag refs.
    metadata.source_hostname_raw = "https://raw.githubusercontent.com/analogdevicesinc/{repository}/{branch}/{pathname}";
    return new Response(JSON.stringify(metadata), {
      status: response.status,
      headers: { "Content-Type": "application/json" },
    });
  };
  // ADI navigates between versions without a full page reload. Refresh the
  // source ref before its page actions initialize for the newly loaded page.
  addEventListener("app:hot_reload:doc_loaded", () => {
    const entry = window.app?.state?.metadata?.repotoc?.[repository];
    if (entry) entry.branch = sourceRef();
  });
  // This project is hosted on GitHub Pages and has no developer.analog.com
  // mirror. Keep version labels, but omit ADI's generic mirror redirect popup
  // on the canonical project site so it cannot obscure figures or link nowhere.
  const removeMirrorPrompt = () => {
    if (location.hostname !== "analogdevicesinc.github.io" ||
        !location.pathname.startsWith(`/${repository}/`)) return;
    document.querySelectorAll(".domain-popup.domain-dev").forEach(popup => popup.remove());
  };
  addEventListener("app:domain:constructed", removeMirrorPrompt);
  // Metadata may arrive after Domain construction; handle that asynchronous
  // insertion too, without changing warnings for other repositories or hosts.
  const observeMirrorPrompt = () => {
    if (location.hostname !== "analogdevicesinc.github.io" ||
        !location.pathname.startsWith(`/${repository}/`)) return;
    removeMirrorPrompt();
    new MutationObserver(removeMirrorPrompt).observe(document.body, { childList: true, subtree: true });
  };
  if (document.readyState === "loading") {
    addEventListener("DOMContentLoaded", observeMirrorPrompt, { once: true });
  } else {
    observeMirrorPrompt();
  }
})();
