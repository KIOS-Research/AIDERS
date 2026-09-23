// Function to add GeoJSON data to the map
function addGeoJSONToMap(data) {


    let areasLayer = 'kmz-areas-layer-'+Math.floor(Math.random() * 1000); // Generate a unique layer ID
    let areasSource = 'kmz-areas-source-'+Math.floor(Math.random() * 1000); // Generate a unique source ID
    let pinsLayer = 'kmz-pins-layer-'+Math.floor(Math.random() * 1000); // Generate a unique layer ID
    let pinsSource = 'kmz-pins-source-'+Math.floor(Math.random() * 1000); // Generate a unique source ID
    // Remove existing layers and sources if they exist
    if (map.getLayer(areasLayer)) map.removeLayer(areasLayer);
    if (map.getSource(areasSource)) map.removeSource(areasSource);
    if (map.getLayer(pinsLayer)) map.removeLayer(pinsLayer);
    if (map.getSource(pinsSource)) map.removeSource(pinsSource);

    // Add areas (lines/polygons) to the map
    map.addSource(areasSource, {
        type: 'geojson',
        data: data
    });

    // Add a fill layer for the polygons
    map.addLayer({
        id: 'area-fill-' + Math.floor(Math.random() * 1000), // Generate a unique layer ID
        type: 'fill',
        source: areasSource,
        paint: {
            'fill-color': '#FF0000', // Fill color for the polygons
            'fill-opacity': 0.1      // Opacity of the fill
        }
    });    

    map.addLayer({
        id: areasLayer,
        type: 'line',
        source: areasSource,
        paint: {
            'line-color': '#FF0000',
            'line-width': 4
        }
    });

    // Add a symbol layer for the area names
    map.addLayer({
        id: 'area-names-'+Math.floor(Math.random() * 1000), // Generate a unique layer ID
        type: 'symbol',
        source: areasSource,
        layout: {
            'text-field': ['get', 'name'], // Assumes the GeoJSON has a 'name' property
            'text-size': 14,
            'text-anchor': 'top'
        },
        paint: {
            'text-color': '#000000'
        }
    });
    
    
    // Extract pins (points) from the GeoJSON data
    const pins = data.features.filter(feature => feature.geometry.type === 'Point');

    const pinsGeoJSON = {
        type: 'FeatureCollection',
        features: pins
    };

    map.addSource(pinsSource, {
        type: 'geojson',
        data: pinsGeoJSON,
        cluster: true, // Enable clustering
        clusterMaxZoom: 26, // Max zoom level to cluster points
        clusterRadius: 50 // Radius of each cluster in pixels
    });

    map.addLayer({
        id: 'clusters'+Math.floor(Math.random() * 1000), // Generate a unique layer ID
        type: 'circle',
        source: pinsSource,
        filter: ['has', 'point_count'], // Only show clusters
        paint: {
            'circle-color': [
                'step',
                ['get', 'point_count'],
                '#51bbd6', // Color for clusters with fewer points
                10, '#f1f075', // Color for clusters with 10+ points
                30, '#f28cb1' // Color for clusters with 30+ points
            ],
            'circle-radius': [
                'step',
                ['get', 'point_count'],
                15, // Radius for clusters with fewer points
                10, 20, // Radius for clusters with 10+ points
                30, 25 // Radius for clusters with 30+ points
            ]
        }
    });

    map.addLayer({
        id: 'cluster-count-'+Math.floor(Math.random() * 10000000), // Generate a unique layer ID
        type: 'symbol',
        source: pinsSource,
        filter: ['has', 'point_count'], // Only show cluster labels
        layout: {
            'text-field': '{point_count_abbreviated}', // Show the number of points in the cluster
            'text-font': ['DIN Offc Pro Medium', 'Arial Unicode MS Bold'],
            'text-size': 12
        }
    });

    map.addLayer({
        id: 'unclustered-point-'+Math.floor(Math.random() * 10000000), // Generate a unique layer ID
        type: 'symbol',
        source: pinsSource,
        filter: ['!', ['has', 'point_count']], // Only show individual points
        layout: {
            'icon-image': 'custom-marker',
            'icon-size': [
                'interpolate', ['linear'], ['zoom'],
                0, 0.2,
                10, 0.8,
                20, 1.0
            ],
            'text-field': ['get', 'name'],
            'text-offset': [0, 1.5],
            'text-anchor': 'top',
            'text-size': [
                'interpolate', ['linear'], ['zoom'],
                0, 8,
                10, 10,
                20, 12
            ]
        },
        paint: {
            'text-color': '#000000'
        }
    });

}   