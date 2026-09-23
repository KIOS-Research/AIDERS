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

/*There are more than a thousand shelters, so they are clustered while zoomed out.
 * The clusters and the shelters themselves are drawn out of a single source*/
const layerShelters = {
    id: 'shelters',
    type: 'symbol',
    source: 'shelters',
    filter: ['!', ['has', 'point_count']],
    layout: {
        'icon-size': 0.5,
        visibility: 'visible',
    },
    paint: {
        'icon-color': '#1a7a7a',
    },
};

const layerShelterClusters = {
    id: 'shelterClusters',
    type: 'circle',
    source: 'shelters',
    filter: ['has', 'point_count'],
    layout: {
        visibility: 'visible',
    },
    paint: {
        'circle-color': '#1a7a7a',
        'circle-opacity': 0.85,
        'circle-stroke-width': 2,
        'circle-stroke-color': '#ffffff',
        'circle-radius': ['step', ['get', 'point_count'], 15, 10, 20, 50, 25, 100, 30],
    },
};

const layerShelterClusterCount = {
    id: 'shelterClusterCount',
    type: 'symbol',
    source: 'shelters',
    filter: ['has', 'point_count'],
    layout: {
        'text-field': '{point_count_abbreviated}',
        'text-font': ['Open Sans Bold', 'Arial Unicode MS Bold'],
        'text-size': 12,
        visibility: 'visible',
    },
    paint: {
        'text-color': '#ffffff',
    },
};

const layerFireStations = {
    id: 'fireStations',
    type: 'symbol',
    source: 'fireStations',
    layout: {
        'icon-size': 0.1,
        visibility: 'visible',
    },
    paint: {
        'icon-color': '#d62828',
    },
};

/*The police stations and the area every station is responsible for come from the same geojson,
 * so they are drawn out of a single source and are switched on/off by a single toggle*/
const layerPoliceStations = {
    id: 'policeStations',
    type: 'symbol',
    source: 'policeStations',
    filter: ['==', ['geometry-type'], 'Point'],
    layout: {
        'icon-size': 0.5,
        visibility: 'visible',
    },
    paint: {
        'icon-color': '#153d8a',
    },
};

const layerPoliceStationBoundaries = {
    id: 'policeStationBoundaries',
    type: 'fill',
    source: 'policeStations',
    filter: ['==', ['geometry-type'], 'Polygon'],
    layout: {
        visibility: 'visible',
    },
    paint: {
        'fill-color': '#153d8a',
        'fill-opacity': 0.12,
    },
};

const layerPoliceStationBoundariesOutline = {
    id: 'policeStationBoundariesOutline',
    type: 'line',
    source: 'policeStations',
    filter: ['==', ['geometry-type'], 'Polygon'],
    layout: {
        'line-join': 'round',
        visibility: 'visible',
    },
    paint: {
        'line-color': '#153d8a',
        'line-width': 1.5,
    },
};

/*The layers below come from the Department of Forests' wildfire dataset*/
const layerFirebreaks = {
    id: 'firebreaks',
    type: 'line',
    source: 'firebreaks',
    layout: {
        'line-join': 'round',
        'line-cap': 'round',
        visibility: 'visible',
    },
    paint: {
        'line-color': '#e42041',
        'line-width': 2,
    },
};

const layerForestStations = {
    id: 'forestStations',
    type: 'symbol',
    source: 'forestStations',
    filter: ['!', ['has', 'point_count']],
    layout: {
        'icon-size': 0.5,
        visibility: 'visible',
    },
    paint: {
        'icon-color': '#2e7d32',
    },
};

const layerFireLookouts = {
    id: 'fireLookouts',
    type: 'symbol',
    source: 'fireLookouts',
    filter: ['!', ['has', 'point_count']],
    layout: {
        'icon-size': 0.5,
        visibility: 'visible',
    },
    paint: {
        'icon-color': '#e65100',
    },
};

const layerHeliports = {
    id: 'heliports',
    type: 'symbol',
    source: 'heliports',
    filter: ['!', ['has', 'point_count']],
    layout: {
        'icon-size': 0.5,
        visibility: 'visible',
    },
    paint: {
        'icon-color': '#1565c0',
    },
};

const layerFireHydrants = {
    id: 'fireHydrants',
    type: 'symbol',
    source: 'fireHydrants',
    filter: ['!', ['has', 'point_count']],
    layout: {
        'icon-size': 0.5,
        visibility: 'visible',
    },
    paint: {
        'icon-color': '#c62828',
    },
};

const layerExplosivesStores = {
    id: 'explosivesStores',
    type: 'symbol',
    source: 'explosivesStores',
    filter: ['!', ['has', 'point_count']],
    layout: {
        'icon-size': 0.5,
        visibility: 'visible',
    },
    paint: {
        'icon-color': '#6a1b9a',
    },
};

/*Builds the two extra layers a clustered point layer needs: a bubble per cluster
 * and the number of features drawn inside it. The features that stayed out of a
 * cluster keep being drawn by layer_obj itself, which filters them in.*/
function create_cluster_layers(layer_obj, color) {
    return {
        circles: {
            id: layer_obj.id + 'Clusters',
            type: 'circle',
            source: layer_obj.source,
            filter: ['has', 'point_count'],
            layout: {
                visibility: 'visible',
            },
            paint: {
                'circle-color': color,
                'circle-opacity': 0.85,
                'circle-stroke-width': 2,
                'circle-stroke-color': '#ffffff',
                'circle-radius': ['step', ['get', 'point_count'], 15, 10, 20, 50, 25, 100, 30],
            },
        },
        counts: {
            id: layer_obj.id + 'ClusterCount',
            type: 'symbol',
            source: layer_obj.source,
            filter: ['has', 'point_count'],
            layout: {
                'text-field': '{point_count_abbreviated}',
                'text-font': ['Open Sans Bold', 'Arial Unicode MS Bold'],
                'text-size': 12,
                visibility: 'visible',
            },
            paint: {
                'text-color': '#ffffff',
            },
        },
    };
}

const forestStationClusterLayers = create_cluster_layers(layerForestStations, '#2e7d32');
const fireLookoutClusterLayers = create_cluster_layers(layerFireLookouts, '#e65100');
const heliportClusterLayers = create_cluster_layers(layerHeliports, '#1565c0');
const fireHydrantClusterLayers = create_cluster_layers(layerFireHydrants, '#c62828');
const explosivesStoreClusterLayers = create_cluster_layers(layerExplosivesStores, '#6a1b9a');

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


const layerGeoZones = {
    id: 'geoZones',
    type: 'fill',
    source: 'geoZones_source',
    layout: {
        //'icon-allow-overlap': true
        visibility: 'visible',
    },
    paint: {
        'fill-outline-color': '#000000',
        'fill-color': ['get', 'fill'],
        'fill-opacity': 0.6,
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
        'line-color': '#fc5e02',
        'line-width': 3,
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

const layerPopulation = {
    id: 'population',
    type: 'fill',
    source: 'population_source',
    layout: {
        visibility: 'visible',
    },
    paint: {
        'fill-outline-color': '#2c3e50',
        'fill-color': [
            'interpolate',
            ['linear'],
            ['get', 'Total_Population'],
            0,
            '#f7fbff',
            100,
            '#deebf7',
            500,
            '#c6dbef',
            1000,
            '#9ecae1',
            1500,
            '#6baed6',
            2000,
            '#4292c6',
            2500,
            '#2171b5',
            3000,
            '#084594'
        ],
        'fill-opacity': 0.5,
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

function add_shelters_on_map(geojson_url, icon_path) {
    const source_id = layerShelters.source;
    if (!map.getSource(source_id)) {
        map.addSource(source_id, {
            type: 'geojson',
            data: geojson_url,
            cluster: true,
            clusterMaxZoom: 14, //From this zoom level and closer, every shelter is shown on its own
            clusterRadius: 50,
        });
    }
    [layerShelterClusters, layerShelterClusterCount].forEach((layer_obj) => {
        if (!map.getLayer(layer_obj.id)) {
            map.addLayer(layer_obj);
        }
    });

    map.loadImage(icon_path, (error, image) => {
        if (error) throw error;
        if (!map.hasImage(source_id + '_symbol')) {
            map.addImage(source_id + '_symbol', image);
        }
        layerShelters['layout']['icon-image'] = source_id + '_symbol';
        if (!map.getLayer(layerShelters.id)) {
            map.addLayer(layerShelters);
        }
    });

    [layerShelters.id, layerShelterClusters.id].forEach((layer_id) => {
        map.on('mouseenter', layer_id, () => {
            tb.defaultCursor = 'pointer';
        });

        map.on('mouseleave', layer_id, () => {
            tb.defaultCursor = '';
        });
    });
}

function remove_shelters_from_map() {
    [layerShelters, layerShelterClusters, layerShelterClusterCount].forEach((layer_obj) => {
        if (map.getLayer(layer_obj.id)) {
            map.removeLayer(layer_obj.id);
        }
    });
}

/*Adds a point layer whose features are grouped into clusters while zoomed out.
 * The clusters and the features are drawn out of a single source, so the three
 * layers are switched on and off together, by a single toggle.*/
function add_clustered_layer_on_map(layer_obj, cluster_layers, geojson_url, icon_path) {
    const source_id = layer_obj.source;
    if (!map.getSource(source_id)) {
        map.addSource(source_id, {
            type: 'geojson',
            data: geojson_url,
            cluster: true,
            clusterMaxZoom: 14, //From this zoom level and closer, every feature is shown on its own
            clusterRadius: 50,
        });
    }
    [cluster_layers.circles, cluster_layers.counts].forEach((cluster_layer) => {
        if (!map.getLayer(cluster_layer.id)) {
            map.addLayer(cluster_layer);
        }
    });

    map.loadImage(icon_path, (error, image) => {
        if (error) throw error;
        if (!map.hasImage(source_id + '_symbol')) {
            map.addImage(source_id + '_symbol', image);
        }
        layer_obj['layout']['icon-image'] = source_id + '_symbol';
        if (!map.getLayer(layer_obj.id)) {
            map.addLayer(layer_obj);
        }
    });

    [layer_obj.id, cluster_layers.circles.id].forEach((layer_id) => {
        map.on('mouseenter', layer_id, () => {
            tb.defaultCursor = 'pointer';
        });

        map.on('mouseleave', layer_id, () => {
            tb.defaultCursor = '';
        });
    });
}

function remove_clustered_layer_from_map(layer_obj, cluster_layers) {
    [layer_obj, cluster_layers.circles, cluster_layers.counts].forEach((layer) => {
        if (map.getLayer(layer.id)) {
            map.removeLayer(layer.id);
        }
    });
}

/*Adds the boundaries first, so that the station icons stay on top of them*/
function add_police_stations_on_map(geojson_url, icon_path) {
    const source_id = layerPoliceStations.source;
    if (!map.getSource(source_id)) {
        map.addSource(source_id, {
            type: 'geojson',
            data: geojson_url,
        });
    }
    [layerPoliceStationBoundaries, layerPoliceStationBoundariesOutline].forEach((layer_obj) => {
        if (!map.getLayer(layer_obj.id)) {
            map.addLayer(layer_obj);
        }
    });

    map.loadImage(icon_path, (error, image) => {
        if (error) throw error;
        if (!map.hasImage(source_id + '_symbol')) {
            map.addImage(source_id + '_symbol', image);
        }
        layerPoliceStations['layout']['icon-image'] = source_id + '_symbol';
        if (!map.getLayer(layerPoliceStations.id)) {
            map.addLayer(layerPoliceStations);
        }
    });

    map.on('mouseenter', layerPoliceStations.id, () => {
        tb.defaultCursor = 'pointer';
    });

    map.on('mouseleave', layerPoliceStations.id, () => {
        tb.defaultCursor = '';
    });
}

function remove_police_stations_from_map() {
    [layerPoliceStations, layerPoliceStationBoundaries, layerPoliceStationBoundariesOutline].forEach((layer_obj) => {
        if (map.getLayer(layer_obj.id)) {
            map.removeLayer(layer_obj.id);
        }
    });
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

function createLineLayer(objectID, color) {
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
        getColor: hexToRgb(color), //for example "#257103"
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

function create_layers_for_new_drone(allDronesArray, index, color) {
    allDronesArray[index].droneLineLayer = createLineLayer(allDronesArray[index].droneID, color);
    return allDronesArray;
}
function create_layers_for_new_device(allDeviceArray, index, color) {
    allDeviceArray[index].deviceLineLayer = createLineLayer(allDeviceArray[index].deviceID, color);
    return allDeviceArray;
}
function create_layers_for_new_balora(allBaloraArray, index, color) {
    allBaloraArray[index].baloraLineLayer = createLineLayer(allBaloraArray[index].baloraID, color);
    return allBaloraArray;
}
