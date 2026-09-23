/*
 * Map Layers State Manager
 *
 * Remembers which toggles of the "Map Layers" side-menu section are switched on and
 * restores them (re-adding their layers on the map) after a page reload.
 * The state is kept in the browser's localStorage, per user/browser.
 */
(function () {
    'use strict';

    const STORAGE_KEY = 'aiders_map_layers_state';
    const TOGGLES_SELECTOR = '#map-layers-list input[type="checkbox"][data-toggle="toggle"]';
    /*Layers are loaded one after the other, so that we don't fire every geojson request at once*/
    const RESTORE_STAGGER_MS = 500;

    function getToggles() {
        return $(TOGGLES_SELECTOR)
            .toArray()
            .filter((toggle) => toggle.id);
    }

    function readSavedState() {
        try {
            return JSON.parse(localStorage.getItem(STORAGE_KEY)) || {};
        } catch (e) {
            console.warn('Saved map layers state is not readable. Ignoring it', e);
            return {};
        }
    }

    /*Stores the state of every map layer toggle, e.g. {"roadLayerToggle": true, "damsLayerToggle": false}*/
    function saveState() {
        let state = {};
        getToggles().forEach((toggle) => {
            state[toggle.id] = toggle.checked;
        });
        try {
            localStorage.setItem(STORAGE_KEY, JSON.stringify(state));
        } catch (e) {
            console.warn('Could not save map layers state', e);
        }
    }

    /*Switches a toggle on/off without letting its inline onchange fire,
     * so that we decide ourselves when the layer will be loaded on the map*/
    function setToggleSilently(toggle, checked) {
        let $toggle = $(toggle);
        let onChange = $toggle.attr('onchange');
        if (onChange !== undefined) {
            $toggle.removeAttr('onchange');
        }
        try {
            $toggle.bootstrapToggle(checked ? 'on' : 'off');
        } catch (e) {
            /*Bootstrap toggle might not be initialized yet. The checkbox itself is what matters*/
            $toggle.prop('checked', checked);
        }
        if (onChange !== undefined) {
            $toggle.attr('onchange', onChange);
        }
    }

    function restoreState() {
        let savedState = readSavedState();
        let togglesToTurnOn = [];

        getToggles().forEach((toggle) => {
            let wasChecked = savedState[toggle.id];
            if (wasChecked === undefined) {
                return;
            }
            /*The toggle might already look checked, because browsers restore form controls
             * on a soft refresh. Its layer is never on the map though, so it always has to be re-added*/
            setToggleSilently(toggle, wasChecked);
            if (wasChecked) {
                togglesToTurnOn.push(toggle);
            }
        });

        togglesToTurnOn.forEach((toggle, index) => {
            setTimeout(() => {
                try {
                    toggleLayerVisibility(toggle.id);
                } catch (e) {
                    console.error('Could not restore map layer of toggle ' + toggle.id, e);
                }
            }, index * RESTORE_STAGGER_MS);
        });
    }

    function clearState() {
        localStorage.removeItem(STORAGE_KEY);
    }

    /*The layers can only be added once the map's style is loaded*/
    function whenMapIsReady(callback) {
        if (typeof map === 'undefined' || map === null) {
            setTimeout(() => whenMapIsReady(callback), 200);
            return;
        }
        if (map.loaded()) {
            callback();
        } else {
            map.once('load', callback);
        }
    }

    window.MapLayersStateManager = {
        save: saveState,
        restore: restoreState,
        clear: clearState,
    };

    $(document).ready(function () {
        if (getToggles().length === 0) {
            return; /*This page has no map layers section*/
        }
        $(document).on('change', TOGGLES_SELECTOR, saveState);
        whenMapIsReady(restoreState);
    });
})();
