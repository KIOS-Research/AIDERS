/* ==========================================================================
   mobile_drawer.js — turns platform.html's 300px fixed sidebar into an
   off-canvas drawer on phones and tablets.

   Loaded synchronously (no defer) from platform.html immediately after the
   </div> that closes .sidebar and immediately before <section class="main-section">.
   That exact position matters:

     - jQuery and jQuery UI are already loaded (head), so the draggable patch
       below can be installed.
     - .sidebar and .logo-menu already exist, so `close-bar` is applied during
       parse and BEFORE first paint — no flash of a 300px-wide drawer.
     - Every feature script (live_stream.js, detectors.js, ...) loads after
       this point, so the patch is in place before any floating window is made.

   This file adds no toggle logic of its own. The floating action button
   forwards its click to the existing .logo-menu handler in platform.html
   (lines ~1881-1887) so there is exactly one source of truth for the state.
   ========================================================================== */

(function () {
    'use strict';

    var MOBILE_QUERY = '(max-width: 1024px)';
    var mql = window.matchMedia(MOBILE_QUERY);
    var root = document.documentElement;
    var sidebar = document.querySelector('.sidebar');

    /* `close-bar` is the existing class; on mobile the stylesheet reinterprets
       it as "fully off-canvas" instead of "60px icon rail". <html>.drawer-open
       is the inverse, used by the CSS to show the backdrop and hide the FAB. */
    function syncRoot() {
        root.classList.toggle('drawer-open', !!sidebar && !sidebar.classList.contains('close-bar'));
    }

    /* --------------------------------------------------------------------
       1. Start closed on mobile. Runs during parse, before first paint.
       -------------------------------------------------------------------- */
    if (mql.matches && sidebar) {
        root.classList.add('is-mobile');
        sidebar.classList.add('close-bar');
    }
    syncRoot();

    /* --------------------------------------------------------------------
       2. Neutralise jQuery UI drag/resize on touch devices.

       jQuery UI 1.12 has no touch support, and its _mouseDown() calls
       preventDefault() on the emulated mouse events. That swallows taps and
       blocks scrolling inside every floating window — so CSS alone is not
       enough, even though platform-mobile.css already makes the pixel offsets
       visually inert.

       All 14 call sites in this codebase are either argument-less or
       .resizable({handles: 'se'}), and the only use of the return value is
       chaining, e.g. $('#overlay').append($(div).resizable(...)). Returning
       `this` therefore preserves every one of them. No code calls
       .draggable('destroy') or .draggable('option', ...).
       -------------------------------------------------------------------- */
    if (mql.matches && window.jQuery && window.jQuery.fn) {
        ['draggable', 'resizable'].forEach(function (method) {
            if (window.jQuery.fn[method]) {
                window.jQuery.fn[method] = function () {
                    return this;
                };
            }
        });
    }

    /* --------------------------------------------------------------------
       3. Backdrop + floating action button.

       Both get an inline display:none here so that platform-mobile.css can
       contain nothing but @media blocks — the media block re-shows them with
       !important, and above 1024px they simply stay hidden.

       The FAB exists because .logo-menu (the hamburger) lives INSIDE .sidebar,
       so an off-canvas drawer hides its own open button.

       Note: #sidebar-overlay is NOT reused as the backdrop. It is a functional
       300px scrim shown by uav_missions.js and user_defined_area.js to dim the
       sidebar during point-picking.
       -------------------------------------------------------------------- */
    function build() {
        if (document.getElementById('m-drawer-backdrop')) {
            return;
        }

        var backdrop = document.createElement('div');
        backdrop.id = 'm-drawer-backdrop';
        backdrop.setAttribute('aria-hidden', 'true');
        backdrop.style.display = 'none';
        document.body.appendChild(backdrop);

        var fab = document.createElement('button');
        fab.id = 'm-drawer-toggle';
        fab.type = 'button';
        fab.setAttribute('aria-label', 'Open menu');
        fab.setAttribute('aria-expanded', 'false');
        fab.innerHTML = '<i class="fa-solid fa-bars"></i>';
        fab.style.display = 'none';
        document.body.appendChild(fab);

        function toggle() {
            var innerHamburger = document.querySelector('.logo-menu');
            if (innerHamburger) {
                /* Delegate to the existing handler rather than duplicating it,
                   so the sidebar and .overlay-section can never drift apart. */
                innerHamburger.click();
            } else if (sidebar) {
                sidebar.classList.toggle('close-bar');
            }
            syncRoot();
            fab.setAttribute('aria-expanded', root.classList.contains('drawer-open') ? 'true' : 'false');
        }

        fab.addEventListener('click', toggle);
        backdrop.addEventListener('click', toggle);
        document.addEventListener('keydown', function (event) {
            if (event.key === 'Escape' && root.classList.contains('drawer-open')) {
                toggle();
            }
        });

        /* Keep <html>.drawer-open in sync when the in-drawer hamburger is used
           directly. The timeout lets platform.html's own listener run first. */
        var innerHamburger = document.querySelector('.logo-menu');
        if (innerHamburger) {
            innerHamburger.addEventListener('click', function () {
                setTimeout(syncRoot, 0);
            });
        }
    }

    if (document.body) {
        build();
    } else {
        document.addEventListener('DOMContentLoaded', build);
    }

    /* --------------------------------------------------------------------
       4. Breakpoint and orientation changes.

       .overlay-section does not exist yet at parse time, which is fine: on
       mobile the stylesheet makes .overlay-section and .overlay-section.close-bar
       identical, so the two only need to agree when crossing back to desktop.
       -------------------------------------------------------------------- */
    function onBreakpointChange(event) {
        root.classList.toggle('is-mobile', event.matches);

        if (sidebar) {
            sidebar.classList[event.matches ? 'add' : 'remove']('close-bar');
        }
        var overlaySection = document.querySelector('.overlay-section');
        if (overlaySection) {
            overlaySection.classList[event.matches ? 'add' : 'remove']('close-bar');
        }

        syncRoot();
        resizeMapSoon(350);
    }

    function resizeMapSoon(delay) {
        setTimeout(function () {
            if (window.map && typeof window.map.resize === 'function') {
                window.map.resize();
            }
        }, delay);
    }

    if (mql.addEventListener) {
        mql.addEventListener('change', onBreakpointChange);
    } else if (mql.addListener) {
        mql.addListener(onBreakpointChange);
    }

    window.addEventListener('orientationchange', function () {
        resizeMapSoon(300);
    });
})();
