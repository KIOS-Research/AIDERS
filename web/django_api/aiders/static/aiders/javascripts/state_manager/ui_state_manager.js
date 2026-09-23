/**
 * UI State Manager
 * Automatically saves and restores UI state across page reloads
 * Uses browser localStorage for persistence
 */

(function() {
    'use strict';

    const STATE_KEY = 'aiders_ui_state';
    const DEBOUNCE_DELAY = 500; // ms to wait before saving after changes

    let saveTimer = null;
    let map = null;
    let stateInitialized = false;
    let pendingLayerStates = {}; // Store layer states that haven't been applied yet

    /**
     * Get current UI state
     */
    function getCurrentState() {
        const state = {
            version: '1.0',
            timestamp: Date.now(),
            map: null,
            panels: {},
            toggles: {},
            windows: {},
            layers: {}
        };

        // Save map state
        if (map && typeof map.getCenter === 'function') {
            try {
                const center = map.getCenter();
                state.map = {
                    center: [center.lng, center.lat],
                    zoom: map.getZoom(),
                    bearing: map.getBearing ? map.getBearing() : 0,
                    pitch: map.getPitch ? map.getPitch() : 0
                };
            } catch (e) {
                console.warn('Could not save map state:', e);
            }
        }

        // Save panel visibility states (only for draggable/modal panels, not main UI panels)
        $('.ui-draggable, .modal, [id*="Box"][id!="searchBox"]').each(function() {
            const $elem = $(this);
            const id = $elem.attr('id');
            if (id) {
                state.panels[id] = {
                    visible: $elem.is(':visible'),
                    display: $elem.css('display')
                };
            }
        });

        // Save toggle button states
        $('input[type="checkbox"], .toggle, input[data-toggle="toggle"]').each(function() {
            const $elem = $(this);
            const id = $elem.attr('id') || $elem.attr('name');
            if (id) {
                state.toggles[id] = {
                    checked: $elem.prop('checked'),
                    disabled: $elem.is(':disabled')
                };
            }
        });

        // Save draggable window positions
        $('.ui-draggable').each(function() {
            const $elem = $(this);
            const id = $elem.attr('id');
            if (id) {
                const position = $elem.position();
                state.windows[id] = {
                    top: position.top,
                    left: position.left,
                    width: $elem.width(),
                    height: $elem.height(),
                    visible: $elem.is(':visible')
                };
            }
        });

        // Save layer visibility (if map layers exist)
        if (map && typeof map.getStyle === 'function') {
            try {
                const style = map.getStyle();
                if (style && style.layers) {
                    style.layers.forEach(layer => {
                        const visibility = map.getLayoutProperty(layer.id, 'visibility');
                        if (visibility !== undefined) {
                            state.layers[layer.id] = visibility;
                        }
                    });
                }
            } catch (e) {
                console.warn('Could not save layer state:', e);
            }
        }

        return state;
    }

    /**
     * Save state to localStorage
     */
    function saveState() {
        try {
            const state = getCurrentState();
            localStorage.setItem(STATE_KEY, JSON.stringify(state));
            console.log('UI state saved');
        } catch (e) {
            console.error('Failed to save UI state:', e);
        }
    }

    /**
     * Debounced save - prevents saving too frequently
     */
    function debouncedSave() {
        if (saveTimer) {
            clearTimeout(saveTimer);
        }
        saveTimer = setTimeout(saveState, DEBOUNCE_DELAY);
    }

    /**
     * Load and restore saved state
     */
    function restoreState() {
        try {
            // Check if we should skip restoration (e.g., after a reset)
            const skipRestore = localStorage.getItem(STATE_KEY + '_skip_restore');
            if (skipRestore) {
                localStorage.removeItem(STATE_KEY + '_skip_restore');
                console.log('Skipping UI state restoration (reset requested)');
                
                // Reset all toggles to their default HTML state without triggering handlers
                setTimeout(() => {
                    $('input[data-toggle="toggle"]').each(function() {
                        const $elem = $(this);
                        const defaultChecked = $elem.prop('defaultChecked'); // Get original HTML state
                        const onchangeAttr = $elem.attr('onchange');
                        
                        // Temporarily remove onchange to prevent unwanted triggers
                        if (onchangeAttr) {
                            $elem.attr('onchange', '');
                        }
                        
                        try {
                            if (defaultChecked) {
                                $elem.bootstrapToggle('on');
                            } else {
                                $elem.bootstrapToggle('off');
                            }
                        } catch (e) {
                            // Bootstrap toggle might not be ready yet
                            $elem.prop('checked', defaultChecked);
                        }
                        
                        // Restore onchange handler
                        if (onchangeAttr) {
                            $elem.attr('onchange', onchangeAttr);
                        }
                    });
                    console.log('Toggles reset to default state');
                }, 1000);
                
                return false;
            }

            const savedData = localStorage.getItem(STATE_KEY);
            if (!savedData) {
                console.log('No saved UI state found');
                return false;
            }

            const state = JSON.parse(savedData);
            console.log('Restoring UI state from:', new Date(state.timestamp));

            // Restore map state
            if (state.map && map) {
                // Wait for map to be fully loaded
                if (map.loaded()) {
                    applyMapState(state.map);
                } else {
                    map.on('load', () => applyMapState(state.map));
                }
            }

            // Restore panel visibility
            Object.keys(state.panels || {}).forEach(id => {
                const $elem = $('#' + id);
                if ($elem.length) {
                    const panelState = state.panels[id];
                    if (panelState.visible) {
                        $elem.show();
                    } else {
                        $elem.hide();
                    }
                }
            });

            // Restore toggle states - delay to ensure Bootstrap Toggle is initialized
            setTimeout(() => {
                Object.keys(state.toggles || {}).forEach(id => {
                    const $elem = $('#' + id + ', [name="' + id + '"]');
                    if ($elem.length) {
                        const toggleState = state.toggles[id];
                        const onchangeAttr = $elem.attr('onchange');
                        
                        // Temporarily disable onchange to prevent unwanted triggers during state setting
                        const originalOnChange = onchangeAttr;
                        if (originalOnChange) {
                            $elem.attr('onchange', '');
                        }
                        
                        // For Bootstrap Toggle elements
                        if ($elem.attr('data-toggle') === 'toggle') {
                            try {
                                if (toggleState.checked) {
                                    $elem.bootstrapToggle('on');
                                } else {
                                    $elem.bootstrapToggle('off');
                                }
                            } catch (e) {
                                console.warn('Bootstrap toggle not ready for:', id, e);
                            }
                        } else {
                            // Regular checkbox
                            $elem.prop('checked', toggleState.checked);
                            if (toggleState.checked) {
                                $elem.addClass('active');
                            } else {
                                $elem.removeClass('active');
                            }
                        }
                        
                        // Restore onchange handler
                        if (originalOnChange) {
                            $elem.attr('onchange', originalOnChange);
                        }
                        
                        // Now manually execute onchange function ONLY if toggle should be ON
                        if (toggleState.checked && onchangeAttr) {
                            try {
                                // Execute the inline onchange handler
                                // This handles cases like onchange="functionName(this.id)"
                                const elem = $elem[0];
                                const onchangeFunc = new Function('event', onchangeAttr);
                                onchangeFunc.call(elem, { target: elem });
                            } catch (e) {
                                console.warn('Could not execute onchange for:', id, e);
                                // Fallback to trigger
                                $elem.trigger('change');
                            }
                        }
                    }
                });
                console.log('Toggle states restored');
                
                // Restore layers AFTER toggles have had time to add them
                if (state.layers && map) {
                    setTimeout(() => {
                        restoreMapLayers(state.layers);
                    }, 1000);
                }
            }, 500);

            // Restore window positions
            Object.keys(state.windows || {}).forEach(id => {
                const $elem = $('#' + id);
                if ($elem.length && $elem.hasClass('ui-draggable')) {
                    const winState = state.windows[id];
                    $elem.css({
                        top: winState.top + 'px',
                        left: winState.left + 'px'
                    });
                    if (winState.width) $elem.width(winState.width);
                    if (winState.height) $elem.height(winState.height);
                    if (winState.visible) {
                        $elem.show();
                    } else {
                        $elem.hide();
                    }
                }
            });

            stateInitialized = true;
            return true;

        } catch (e) {
            console.error('Failed to restore UI state:', e);
            return false;
        }
    }

    /**
     * Restore map layers visibility
     * Separated into its own function so it can be called after toggles create layers
     */
    function restoreMapLayers(layers) {
        if (!layers || !map) return;
        
        // Store pending layer states
        pendingLayerStates = Object.assign({}, layers);
        
        const restoreLayers = () => {
            let restoredCount = 0;
            let pendingCount = 0;
            
            Object.keys(pendingLayerStates).forEach(layerId => {
                try {
                    if (map.getLayer(layerId)) {
                        map.setLayoutProperty(layerId, 'visibility', pendingLayerStates[layerId]);
                        restoredCount++;
                        delete pendingLayerStates[layerId]; // Remove from pending once applied
                    } else {
                        pendingCount++;
                    }
                } catch (e) {
                    // Layer might not exist yet
                    pendingCount++;
                }
            });
            
            if (restoredCount > 0) {
                console.log('Restored visibility for', restoredCount, 'map layers');
            }
            if (pendingCount > 0) {
                console.log('Still waiting for', pendingCount, 'layers to be added');
            }
        };
        
        // Try immediately
        restoreLayers();
        
        // Try again after delays to catch dynamically loaded layers
        setTimeout(restoreLayers, 500);
        setTimeout(restoreLayers, 1000);
        setTimeout(restoreLayers, 2000);
        setTimeout(restoreLayers, 3000);
        setTimeout(restoreLayers, 5000);
        
        // Set up a listener for when new layers are added to the map
        if (map.on) {
            const originalAddLayer = map.addLayer;
            if (originalAddLayer) {
                map.addLayer = function(...args) {
                    const result = originalAddLayer.apply(this, args);
                    // Check if this layer needs state restoration
                    setTimeout(() => {
                        const layerId = args[0]?.id || args[0];
                        if (layerId && pendingLayerStates[layerId]) {
                            try {
                                map.setLayoutProperty(layerId, 'visibility', pendingLayerStates[layerId]);
                                console.log('Restored layer on add:', layerId);
                                delete pendingLayerStates[layerId];
                            } catch (e) {
                                console.warn('Failed to restore layer on add:', layerId, e);
                            }
                        }
                    }, 100);
                    return result;
                };
            }
            
            // Also listen for style.load event which fires when layers are added
            map.on('style.load', () => {
                console.log('Map style loaded, restoring layers...');
                setTimeout(restoreLayers, 500);
            });
            
            map.on('sourcedata', (e) => {
                // When source data loads, layers might be added
                if (e.isSourceLoaded) {
                    setTimeout(restoreLayers, 200);
                }
            });
        }
    }

    /**
     * Apply map state
     */
    function applyMapState(mapState) {
        if (!map || !mapState) return;

        try {
            // Restore map view
            map.jumpTo({
                center: mapState.center,
                zoom: mapState.zoom,
                bearing: mapState.bearing || 0,
                pitch: mapState.pitch || 0
            });
            console.log('Map state restored');
        } catch (e) {
            console.warn('Could not restore map state:', e);
        }
    }

    /**
     * Set up event listeners to track state changes
     */
    function setupListeners() {
        // Track map movements
        if (map) {
            map.on('moveend', debouncedSave);
            map.on('zoomend', debouncedSave);
            if (map.on) {
                map.on('rotateend', debouncedSave);
                map.on('pitchend', debouncedSave);
            }
        }

        // Track panel show/hide
        $(document).on('show.bs.collapse hide.bs.collapse', debouncedSave);
        $(document).on('shown.bs.modal hidden.bs.modal', debouncedSave);

        // Track toggle changes
        $(document).on('change', 'input[type="checkbox"], .toggle, input[data-toggle="toggle"]', debouncedSave);
        
        // Track Bootstrap Toggle specific events
        $(document).on('change.bootstrapToggle', 'input[data-toggle="toggle"]', debouncedSave);

        // Track window drag
        $(document).on('dragstop', '.ui-draggable', debouncedSave);

        // Track layer visibility changes (if applicable)
        if (map) {
            // Monitor when layers change visibility
            map.on('idle', debouncedSave);
            
            // Some implementations might have custom events
            $(document).on('layer:toggle layer:show layer:hide', debouncedSave);
        }

        // Save state before page unload
        $(window).on('beforeunload', function() {
            saveState();
        });

        // Periodic save (backup - every 30 seconds)
        setInterval(saveState, 30000);
    }

    /**
     * Initialize the state manager
     */
    function init(mapInstance) {
        map = mapInstance;

        console.log('UI State Manager initialized');

        // Restore state after a delay to let the page and Bootstrap Toggle initialize
        setTimeout(() => {
            restoreState();
            setupListeners();
        }, 2000);
    }

    /**
     * Clear saved state
     */
    function clearState() {
        try {
            localStorage.removeItem(STATE_KEY);
            console.log('UI state cleared');
            return true;
        } catch (e) {
            console.error('Failed to clear UI state:', e);
            return false;
        }
    }

    /**
     * Clear state and reload page
     */
    function clearAndReload() {
        try {
            // Set a flag to prevent restoration on next load
            localStorage.setItem(STATE_KEY + '_skip_restore', 'true');
            localStorage.removeItem(STATE_KEY);
            console.log('UI state cleared, reloading...');
            // Small delay to ensure localStorage is written
            setTimeout(() => {
                location.reload();
            }, 100);
            return true;
        } catch (e) {
            console.error('Failed to clear UI state:', e);
            return false;
        }
    }

    /**
     * Export state for debugging
     */
    function exportState() {
        const state = getCurrentState();
        console.log('Current UI State:', state);
        return state;
    }

    // Make functions available globally
    window.UIStateManager = {
        init: init,
        save: saveState,
        restore: restoreState,
        clear: clearState,
        clearAndReload: clearAndReload,
        export: exportState,
        getCurrentState: getCurrentState
    };

    // Auto-initialize when document is ready
    $(document).ready(function() {
        // Wait for map to be available
        const checkMap = setInterval(function() {
            // Check for common map variable names
            if (window.map || window.myMap || window.mainMap) {
                const detectedMap = window.map || window.myMap || window.mainMap;
                init(detectedMap);
                clearInterval(checkMap);
            }
        }, 500);

        // Stop checking after 10 seconds
        setTimeout(() => clearInterval(checkMap), 10000);
    });

})();
