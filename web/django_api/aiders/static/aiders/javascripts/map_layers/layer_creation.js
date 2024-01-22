/*An offlne layer*/
const offlineStyle = {
    version: 8,
    sources: {
        'raster-tiles': {
            type: 'raster',
            tiles: ['http://mt0.google.com/vt/lyrs=m&hl=en&x={x}&y={y}&z={z}'],
            tileSize: 256,
            attribution:
                'Map tiles by <a target="_top" rel="noopener" href="http://stamen.com">Stamen Design</a>, under <a target="_top" rel="noopener" href="http://creativecommons.org/licenses/by/3.0">CC BY 3.0</a>. Data by <a target="_top" rel="noopener" href="http://openstreetmap.org">OpenStreetMap</a>, under <a target="_top" rel="noopener" href="http://creativecommons.org/licenses/by-sa/3.0">CC BY SA</a>',
        },
    },
    layers: [
        {
            id: 'simple-tiles',
            type: 'raster',
            source: 'raster-tiles',
            minzoom: 0,
            maxzoom: 22,
        },
    ],
};

//A mapbox layer that adds height to the buildings
const threeDbuildingLayer = {
    id: '3d-buildings',
    source: 'composite',
    'source-layer': 'building',
    filter: ['==', 'extrude', 'true'],
    type: 'fill-extrusion',
    minzoom: 12,
    paint: {
        'fill-extrusion-color': [
            'case',
            ['boolean', ['feature-state', 'select'], false],
            'lightgreen',
            ['boolean', ['feature-state', 'hover'], false],
            'lightblue',
            '#aaa',
        ],

        // use an 'interpolate' expression to add a smooth transition effect to the
        // buildings as the user zooms in
        'fill-extrusion-height': ['interpolate', ['linear'], ['zoom'], 12, 0, 12 + 0.05, ['get', 'height']],
        'fill-extrusion-base': ['interpolate', ['linear'], ['zoom'], 12, 0, 12 + 0.05, ['get', 'min_height']],
        'fill-extrusion-opacity': 0.7,
    },
};

const layerRoads = {
    id: 'roads',
    type: 'line',
    source: 'roads_source',
    layout: {
        //'icon-allow-overlap': true
        visibility: 'visible',
    },
    paint: {
        'line-color': '#ff69b4',
        'line-width': 1,
    },
};

const layerBuildings = {
    id: 'buildings',
    type: 'fill',
    source: 'buildings_source',
    layout: {
        //'icon-allow-overlap': true
        visibility: 'visible',
    },
    paint: {
        'fill-outline-color': '#3c997b',
        'fill-color': '#3cf0b7',
        'fill-opacity': 0.5,
    },
};

let layerDams = {
    id: 'points',
    type: 'symbol',
    source: 'points',
    layout: {
        'icon-size': 0.1,
        visibility: 'visible',
    },
    paint: {
        'icon-color': '#4980e6',
    },
};

const layerHospitals = {
    id: 'hospitals',
    type: 'symbol',
    source: 'hospitals',
    layout: {
        'icon-size': 0.1,
        visibility: 'visible',
    },
    paint: {
        'icon-color': '#ff0000',
    },
};

const layerPoles = {
    id: 'ecaPoles',
    type: 'symbol',
    source: 'ecaPoles',
    layout: {
        'icon-size': 0.1,
        visibility: 'visible',
    },
    paint: {
        'icon-color': '#0702ff',
    },
};

const layerPoleLines = {
    id: 'ecaPoleLines',
    type: 'line',
    source: 'ecaPoleLines',
    layout: {
        'line-join': 'round',
        'line-cap': 'round',
        // 'visibility': 'none'
    },
    paint: {
        'line-color': '#0223fc',
        'line-width': 3,
    },
};

const layerRestrictedAllowedAreasForDrone = {
    id: 'restrictedAllowedAreasForDrone',
    type: 'line',
    source: 'restrictedAllowedAreasForDrone_source',
    layout: {
        //'icon-allow-overlap': true
        visibility: 'visible',
    },
    paint: {
        'line-color': '#fcfc05',
        'line-width': 1,
    },
};

const layercyprusFir = {
    id: 'cyprusFir',
    type: 'line',
    source: 'cyprusFir_source',
    layout: {
        //'icon-allow-overlap': true
        visibility: 'visible',
    },
    paint: {
        'line-color': '#02fc0a',
        'line-width': 1,
    },
};
const layerOperationAreas = {
    id: 'operationAreas',
    type: 'line',
    source: 'operationAreas_source',
    layout: {
        //'icon-allow-overlap': true
        visibility: 'visible',
    },
    paint: {
        'line-color': '#fc2024',
        'line-width': 1,
    },
};
// const layerDams = {
//     'id': 'points',
//     'type': 'symbol',
//     'source': 'points',
//     'layout': {
//         'icon-image': WEBSERVER_URL_CYPRUS_DAM_ICON_PATH + "_symbol",
//         'icon-size': 0.02
//     }
// }
//Reference: https://docs.mapbox.com/vector-tiles/reference/mapbox-terrain-v2/#contour
const layerTerrainLines = {
    id: 'terrain-data',
    type: 'line',
    source: 'terrain-data_source',
    'source-layer': 'contour',
    layout: {
        'line-join': 'round',
        'line-cap': 'round',
        // 'visibility': 'none'
    },
    paint: {
        'line-color': '#4980e6',
        'line-width': 1,
    },
};

// const layerRoadsHighlighted = {
//     'id': 'roads-highligted',
//     'type': 'line',
//     'source': 'roads',
//     'layout': {
//         //'icon-allow-overlap': true
//         'visibility': 'none',
//     },
//     'paint': {
//         'line-color': '#ae1ed6',
//         'line-width': 3
//     },
//     'filter': ['in', 'id', '']
// };

let threeDMarker_origin = [33.4151176797, 35.1452954125, 30];
let label;

// TODO: Try to make label as a 3b object
const thrreeDmarker_Layer = {
    id: 'threeDMarkerLayer',
    type: 'custom',
    renderingMode: '3d',
    onAdd: function (map, mbxContext) {
        label = tb.label(
            (obj = {
                position: threeDMarker_origin,
                htmlElement: createLabel(),
                cssClass: ' label3D',
                alwaysVisible: true,
                bottomMargin: 0,
                feature: null,
            })
        );

        label.setCoords(threeDMarker_origin);
        // label.addEventListener('ObjectDragged', onDraggedObject, true);
        // label.addEventListener('ObjectMouseOver', onObjectMouseOver, true);
        // label.addEventListener('ObjectMouseOut', onObjectMouseOut, true);

        tb.add(label);
    },

    render: function (gl, matrix) {
        // tb.update();
    },
};

function add_symbol_layer(symbol_png, geojson_url, color) {
    map.loadImage(symbol_png, (error, image) => {
        if (error) throw error;
        if (!map.hasImage('custom-marker')) {
            map.addImage('custom-marker', image, { sdf: true });
        }
        // Add a GeoJSON source with 2 points
        if (!map.getSource('points')) {
            map.addSource('points', {
                type: 'geojson',
                data: geojson_url,
            });
        }

        // Add a symbol layer
        map.addLayer({
            id: 'points',
            type: 'symbol',
            source: 'points',
            layout: {
                'icon-image': 'custom-marker',
                'icon-size': 0.1,
                // get the title name from the source's "title" property
                // 'text-field': ['get', 'title'],
                // 'text-font': [
                //     'Open Sans Semibold',
                //     'Arial Unicode MS Bold'
                // ],
                // 'text-offset': [0, 1.25],
                // 'text-anchor': 'top'
            },
            paint: {
                'icon-color': color,
            },
        });
    });
}
function createLabel() {
    var divTooltip = document.createElement('div');
    divTooltip.className = 'marker';

    let color = 'background-color:' + get_selected_drone_marker_color() + ';';
    let test = '<span style=' + color + '><b>';
    divTooltip.innerHTML = test + (markerNumber + 1) + '</b></span>';
    return divTooltip;
}

function add_terrain_lines_layer() {
    if (!map.getSource('terrain-data_source')) {
        map.addSource('terrain-data_source', {
            type: 'vector',
            url: MAPBOX_TERRAIN_LINES_V2_URL,
        });
    }
    map.addLayer(layerTerrainLines);
}

function createLineLayer(objectID) {
    return new MapboxLayer({
        id: objectID + 'line',
        type: LineLayer,
        data: [],
        fp64: false,
        widthScale: 0.1,
        getWidth: 90, //Change getWidth and widthScale to fine-tune the line width
        opacity: 0.1,
        widthUnit: 'meters',
        // getStrokeWidth: 6,
        getSourcePosition: (d) => d.source,
        getTargetPosition: (d) => d.dest,
        getColor: hexToRgb(getRandomColour()), //for example "#257103"
    });
}

function add_circl_layer(source_id, layer_id, radiusMeters, lat, lon) {
    let _center = turf.point([lon, lat]);
    let _radiusKM = radiusMeters / 1000;
    let _options = {
        steps: 80,
        units: 'kilometers', // or "mile"
    };

    let _circle = turf.circle(_center, _radiusKM, _options);
    map.addSource(source_id, {
        type: 'geojson',
        data: _circle,
    });

    map.addLayer({
        id: layer_id,
        type: 'fill',
        source: source_id,
        paint: {
            'fill-color': 'yellow',
            'fill-opacity': 0.2,
        },
    });
}
function add_layer_on_map(
    source_id,
    geojson_url,
    layer_obj,
    type,
    icon_path = 'None',
    icon_color = 'None',
    icon_size = 0.1,
    changeCursorStyleOnHover = false,
    tolerance = 0.375
) {
    if (icon_path === 'None') {
        if (!map.getSource(source_id)) {
            map.addSource(source_id, {
                type: type,
                data: geojson_url,
                //buffer: 0,
                //tolerance: 3.5
            });
        }
        if (!map.getLayer(layer_obj)) {
            map.addLayer(layer_obj);
        }
    } else {
        layer_obj['paint']['icon-color'] = icon_color;
        layer_obj['layout']['icon-size'] = icon_size;
        map.loadImage(icon_path, (error, image) => {
            if (error) throw error;
            if (!map.hasImage(source_id + '_symbol')) {
                map.addImage(source_id + '_symbol', image);
            }
            layer_obj['layout']['icon-image'] = source_id + '_symbol';
            if (!map.getSource(source_id)) {
                map.addSource(source_id, {
                    type: type,
                    data: geojson_url,
                    tolerance: tolerance,
                });
            }
            if (!map.getLayer(layer_obj)) {
                map.addLayer(layer_obj);
            }
        });
    }

    if (changeCursorStyleOnHover) {
        map.on('mouseenter', layer_obj.id, () => {
            tb.defaultCursor = 'pointer';
        });

        map.on('mouseleave', layer_obj.id, () => {
            tb.defaultCursor = '';
        });
    }
}

function create_layers_for_new_drone(allDronesArray, index) {
    allDronesArray[index].droneLineLayer = createLineLayer(allDronesArray[index].droneID);
    return allDronesArray;
}
function create_layers_for_new_device(allDeviceArray, index) {
    allDeviceArray[index].deviceLineLayer = createLineLayer(allDeviceArray[index].deviceID);
    return allDeviceArray;
}
function create_layers_for_new_balora(allBaloraArray, index) {
    allBaloraArray[index].baloraLineLayer = createLineLayer(allBaloraArray[index].baloraID);
    return allBaloraArray;
}
