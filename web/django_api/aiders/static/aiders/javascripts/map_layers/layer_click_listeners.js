{
    //Register all the click evens for infrustructure layers (e.g Dams, Buildings etc.)
    map.on('click', layerDams.id, (e) => {
        // Copy coordinates array.
        // console.log("EVENT: ")
        // expandedLog(e)
        const coordinates = e.features[0].geometry.coordinates.slice();

        const description = e.features[0].properties.description;
        const name =  e.features[0].properties["NAME"];
        const river =  e.features[0].properties["RIVER"];
        const capacity =  e.features[0].properties["CAPACITY (m3)"];
        let descr = `<strong>Name: ${name}
                     <br>River: ${river}
                     <br>Capacity: ${capacity} m³                 
                     <br>Lon: ${coordinates[0].toFixed(6)} °                 
                     <br>Lat: ${coordinates[1].toFixed(6)} °                 
</strong>`
        new maplibregl.Popup()
            .setLngLat(coordinates)
            .setHTML(descr)
            .addTo(map);
    });



    map.on('click', layerHospitals.id, (e) => {
        const coordinates = e.features[0].geometry.coordinates.slice();
        const name =  e.features[0].properties["Hospital"];
        const tel =  e.features[0].properties["Tel"];
        let descr = `<strong>Name: ${name}
                     <br>Tel: ${tel}
                     <br>Lon: ${coordinates[0].toFixed(6)} °                 
                     <br>Lat: ${coordinates[1].toFixed(6)} °                 
</strong>`
        new maplibregl.Popup()
            .setLngLat(coordinates)
            .setHTML(descr)
            .addTo(map);
    });


    map.on('click', layerPoliceStations.id, (e) => {
        const coordinates = e.features[0].geometry.coordinates.slice();
        const properties = e.features[0].properties;
        let descr = `<strong>Police Station: ${properties["Station_Name"] || 'N/A'}
                     <br>Station No: ${properties["Station_No"]}
                     <br>Address: ${properties["Address"] || 'N/A'}
                     <br>Tel: ${properties["Tel"] || 'N/A'}
                     <br>Division: ${properties["Division"] || 'N/A'}
                     <br>Division Code: ${properties["Division_Code"]}
                     <br>Lon: ${coordinates[0].toFixed(6)} °
                     <br>Lat: ${coordinates[1].toFixed(6)} °
</strong>`
        new maplibregl.Popup()
            .setLngLat(coordinates)
            .setHTML(descr)
            .addTo(map);
    });


    /*Clicking a cluster zooms in, up to the point where the cluster breaks apart*/
    map.on('click', layerShelterClusters.id, (e) => {
        const clusterID = e.features[0].properties.cluster_id;
        map.getSource(layerShelters.source).getClusterExpansionZoom(clusterID, (error, zoom) => {
            if (error) return;
            map.easeTo({
                center: e.features[0].geometry.coordinates,
                zoom: zoom,
            });
        });
    });

    map.on('click', layerShelters.id, (e) => {
        const coordinates = e.features[0].geometry.coordinates.slice();
        const properties = e.features[0].properties;
        const capacity = properties["Capacity"];
        let descr = `<strong>Shelter: ${properties["Shelter_ID"]}
                     <br>Type: ${properties["Type"]}
                     <br>Capacity: ${capacity ? capacity.toLocaleString() + ' people' : 'N/A'}
                     <br>Sign Number: ${properties["Sign_Number"] || 'N/A'}
                     <br>Address: ${properties["Address"]}
                     <br>District: ${properties["District"]}
                     <br>Sub District: ${properties["Sub_District"] || 'N/A'}
                     <br>Lon: ${coordinates[0].toFixed(6)} °
                     <br>Lat: ${coordinates[1].toFixed(6)} °
</strong>`
        new maplibregl.Popup()
            .setLngLat(coordinates)
            .setHTML(descr)
            .addTo(map);
    });


    map.on('click', layerFireStations.id, (e) => {
        const coordinates = e.features[0].geometry.coordinates.slice();
        const name = e.features[0].properties["CSC_Name"];
        const address = e.features[0].properties["Address"];
        const citizenTel = e.features[0].properties["Citizen_Tel_No"];
        const officeTel = e.features[0].properties["Office_Tel"];
        const fax = e.features[0].properties["Office_Fax"];
        const openingHours = e.features[0].properties["Opening_Hours"];
        const email = e.features[0].properties["Email"];
        let descr = `<strong>Name: ${name}
                     <br>Address: ${address}
                     <br>Emergency Tel: ${citizenTel}
                     <br>Office Tel: ${officeTel}
                     <br>Fax: ${fax}
                     <br>Opening Hours: ${openingHours}
                     <br>Email: ${email}
                     <br>Lon: ${coordinates[0].toFixed(6)} °
                     <br>Lat: ${coordinates[1].toFixed(6)} °
</strong>`
        new maplibregl.Popup()
            .setLngLat(coordinates)
            .setHTML(descr)
            .addTo(map);
    });



    map.on('click', layerFirebreaks.id, (e) => {
        const coordinates = e.lngLat;
        const properties = e.features[0].properties;
        const width = properties["FRTRCWidth"];
        const length = properties["FRTRCLength"];
        let descr = `<strong>Firebreak: ${properties["NAME_GR"] || 'N/A'}
                     <br>Forest Division: ${properties["ForestDivision"] || 'N/A'}
                     <br>Forest Sub Division: ${properties["ForestSubDivision"] || 'N/A'}
                     <br>Forest Station: ${properties["ForestStation"] || 'N/A'}
                     <br>State Forest: ${properties["StateForest"] || 'N/A'}
                     <br>Length: ${length ? length + ' km' : 'N/A'}
                     <br>Width: ${width ? width + ' m' : 'N/A'}
                     <br>Accessible: ${properties["Accessible"] || 'N/A'}
                     <br>Lon: ${coordinates.lng.toFixed(6)} °
                     <br>Lat: ${coordinates.lat.toFixed(6)} °
</strong>`
        new maplibregl.Popup()
            .setLngLat(coordinates)
            .setHTML(descr)
            .addTo(map);
    });


    /*Clicking any of these clusters zooms in, up to the point where it breaks apart*/
    [
        forestStationClusterLayers,
        fireLookoutClusterLayers,
        heliportClusterLayers,
        fireHydrantClusterLayers,
        explosivesStoreClusterLayers,
    ].forEach((cluster_layers) => {
        map.on('click', cluster_layers.circles.id, (e) => {
            const clusterID = e.features[0].properties.cluster_id;
            map.getSource(cluster_layers.circles.source).getClusterExpansionZoom(clusterID, (error, zoom) => {
                if (error) return;
                map.easeTo({
                    center: e.features[0].geometry.coordinates,
                    zoom: zoom,
                });
            });
        });
    });


    map.on('click', layerForestStations.id, (e) => {
        const coordinates = e.features[0].geometry.coordinates.slice();
        const properties = e.features[0].properties;
        let descr = `<strong>Forest Station: ${properties["NAME_EN"] || properties["NAME_GR"] || 'N/A'}
                     <br>Name (GR): ${properties["NAME_GR"] || 'N/A'}
                     <br>Forest Division: ${properties["ForestDivision"] || 'N/A'}
                     <br>Forest Sub Division: ${properties["ForestSubDivision"] || 'N/A'}
                     <br>State Forest: ${properties["StateForest"] || 'N/A'}
                     <br>Main Station: ${properties["MainStation"] || 'N/A'}
                     <br>Lon: ${coordinates[0].toFixed(6)} °
                     <br>Lat: ${coordinates[1].toFixed(6)} °
</strong>`
        new maplibregl.Popup()
            .setLngLat(coordinates)
            .setHTML(descr)
            .addTo(map);
    });


    map.on('click', layerFireLookouts.id, (e) => {
        const coordinates = e.features[0].geometry.coordinates.slice();
        const properties = e.features[0].properties;
        let descr = `<strong>Fire Lookout: ${properties["NAME_EN"] || properties["NAME_GR"] || 'N/A'}
                     <br>Name (GR): ${properties["NAME_GR"] || 'N/A'}
                     <br>Type: ${properties["Type"] || 'N/A'}
                     <br>Forest Division: ${properties["ForestDivision"] || 'N/A'}
                     <br>Managed By: ${properties["Management"] || 'N/A'}
                     <br>Personnel: ${properties["Personnel"] || 'N/A'}
                     <br>Night Shift: ${properties["Nightshift"] || 'N/A'}
                     <br>Tel: ${properties["TelNo"] || 'N/A'}
                     <br>Lon: ${coordinates[0].toFixed(6)} °
                     <br>Lat: ${coordinates[1].toFixed(6)} °
</strong>`
        new maplibregl.Popup()
            .setLngLat(coordinates)
            .setHTML(descr)
            .addTo(map);
    });


    /*The heliport geojson misspells its division fields as "Divsion"*/
    map.on('click', layerHeliports.id, (e) => {
        const coordinates = e.features[0].geometry.coordinates.slice();
        const properties = e.features[0].properties;
        let descr = `<strong>Heliport: ${properties["Locality"] || 'N/A'}
                     <br>Heliport ID: ${properties["HellPortID"] || 'N/A'}
                     <br>Forest Division: ${properties["ForestDivsion"] || 'N/A'}
                     <br>Forest Sub Division: ${properties["ForestSubDivsion"] || 'N/A'}
                     <br>Forest Station: ${properties["ForestStation"] || 'N/A'}
                     <br>State Forest: ${properties["StateForest"] || 'N/A'}
                     <br>Compartment: ${properties["Compartment"] || 'N/A'}
                     <br>Details: ${properties["Details"] || 'N/A'}
                     <br>Lon: ${coordinates[0].toFixed(6)} °
                     <br>Lat: ${coordinates[1].toFixed(6)} °
</strong>`
        new maplibregl.Popup()
            .setLngLat(coordinates)
            .setHTML(descr)
            .addTo(map);
    });


    map.on('click', layerFireHydrants.id, (e) => {
        const coordinates = e.features[0].geometry.coordinates.slice();
        const properties = e.features[0].properties;
        let descr = `<strong>Hydrant No: ${properties["HYNumber"] || 'N/A'}
                     <br>Locality: ${properties["Locality"] || 'N/A'}
                     <br>Water Source: ${properties["HYSource"] || 'N/A'}
                     <br>District: ${properties["District"] || 'N/A'}
                     <br>Forest Division: ${properties["ForestDivision"] || 'N/A'}
                     <br>Forest Sub Division: ${properties["ForestSubDivision"] || 'N/A'}
                     <br>Forest Station: ${properties["ForestStation"] || 'N/A'}
                     <br>State Forest: ${properties["StateForest"] || 'N/A'}
                     <br>Lon: ${coordinates[0].toFixed(6)} °
                     <br>Lat: ${coordinates[1].toFixed(6)} °
</strong>`
        new maplibregl.Popup()
            .setLngLat(coordinates)
            .setHTML(descr)
            .addTo(map);
    });


    map.on('click', layerExplosivesStores.id, (e) => {
        const coordinates = e.features[0].geometry.coordinates.slice();
        const properties = e.features[0].properties;
        const seveso = properties["Seveso"];
        let descr = `<strong>Explosives Store: ${properties["Name"] || 'N/A'}
                     <br>Stores: ${properties["Category"] || 'N/A'}
                     <br>District: ${properties["District"] || 'N/A'}
                     <br>Tel: ${properties["Tel"] || 'N/A'}
                     <br>SEVESO: ${seveso ? seveso : 'Not classified'}
                     <br>Details: ${properties["Details"] || 'N/A'}
                     <br>Lon: ${coordinates[0].toFixed(6)} °
                     <br>Lat: ${coordinates[1].toFixed(6)} °
</strong>`
        new maplibregl.Popup()
            .setLngLat(coordinates)
            .setHTML(descr)
            .addTo(map);
    });


    map.on('click', layerPoles.id, (e) => {
        const coordinates = e.features[0].geometry.coordinates.slice();
        const properties = e.features[0].properties;
        const height = properties["structure_height"];
        let descr = `<strong>Line Name: ${properties["line_name"] || 'N/A'}
                     <br>Line ID: ${properties["line_id"] || 'N/A'}
                     <br>Pole Number: ${properties["structure_number"] || 'N/A'}
                     <br>Construction Date: ${properties["construction_date"] || 'N/A'}
                     <br>Height: ${height !== undefined ? height + ' m' : 'N/A'}
                     <br>Authority: ${properties["authority"] || 'N/A'}
                     <br>Last Edited: ${properties["last_edited_date"] || 'N/A'}
                     <br>Lon: ${coordinates[0].toFixed(6)} °
                     <br>Lat: ${coordinates[1].toFixed(6)} °
</strong>`
        new maplibregl.Popup()
            .setLngLat(coordinates)
            .setHTML(descr)
            .addTo(map);
    });


    map.on('click', layerPoleLines.id, (e)=>{
        const coordinates = e.lngLat
        const properties = e.features[0].properties;
        const length =  properties["Shape_Leng"];
        let descr = `<strong>Line Name: ${properties["Line_Name"] || 'N/A'}
                     <br>Line ID: ${properties["Line_ID"] || 'N/A'}
                     <br>Voltage: ${properties["Voltage"] || 'N/A'}
                     <br>Authority: ${properties["Authority"] || 'N/A'}
                     <br>Source Of Data: ${properties["Source_Of_Data"] || 'N/A'}
                     <br>Last Edited: ${properties["Last_Edited_Date"] || 'N/A'}
                     <br>Length: ${length !== undefined ? length.toFixed(0) + ' m' : 'N/A'}
                     <br>Lon: ${parseFloat(coordinates['lng']).toFixed(6)} °
                     <br>Lat: ${parseFloat(coordinates['lat']).toFixed(6)} °
</strong>`
        new maplibregl.Popup()
            .setLngLat(coordinates)
            .setHTML(descr)
            .addTo(map);
    });


    map.on('click', layerBuildings.id, (e)=>{
        const coordinates = e.lngLat
        const area_sqm =  e.features[0].properties["area_sqm"];
        const floor_qnty =  e.features[0].properties["floor_qty"];
        let descr = `<strong>Area: ${area_sqm} m<sup>2</sup>
                     <br>Floors: ${floor_qnty}
</strong>`
        new maplibregl.Popup()
            .setLngLat(coordinates)
            .setHTML(descr)
            .addTo(map);
    });

    map.on('click', layerPopulation.id, (e) => {
        const coordinates = e.lngLat;
        const postcode = e.features[0].properties["post_code"];
        const population = e.features[0].properties["Total_Population"];
        const area = e.features[0].properties["area_km2"];
        const city = e.features[0].properties["vil_name_e"] || "N/A";
        
        let descr = `<strong>City: ${city}
                        <br>Population: ${population ? Math.round(population).toLocaleString() : 'N/A'}
                        <br>Postcode: ${postcode || 'N/A'}
                        <br>Area: ${area ? area.toFixed(2) + ' km²' : 'N/A'}
                        <br>Lat: ${coordinates.lat.toFixed(6)} °                 
                        <br>Lon: ${coordinates.lng.toFixed(6)} °                 
                    </strong>`
        new maplibregl.Popup()
            .setLngLat(coordinates)
            .setHTML(descr)
            .addTo(map);
    });

    /*These properties only describe how a geo zone is drawn, so they are of no use to the user*/
    const GEO_ZONE_STYLE_PROPERTIES = [
        'fill',
        'fill-opacity',
        'stroke',
        'stroke-width',
        'stroke-opacity',
        'styleUrl',
        'styleHash',
        'styleMapHash',
    ];

    map.on('click', layerGeoZones.id, (e) => {
        const coordinates = e.lngLat;
        let alreadyShownZones = [];
        let descr = '';

        e.features.forEach((feature) => {
            const properties = feature.properties;
            const name = properties['name'] || 'Geo Zone';
            /*Zones that span more than one tile are returned once per tile*/
            if (alreadyShownZones.includes(name)) {
                return;
            }
            alreadyShownZones.push(name);

            if (descr !== '') {
                descr += '<hr>';
            }
            descr += `<strong>${name}</strong>`;
            Object.keys(properties).forEach((key) => {
                const value = properties[key];
                if (key === 'name' || GEO_ZONE_STYLE_PROPERTIES.includes(key) || value === null || value === '') {
                    return;
                }
                /*A zone's description (manager, telephone, access terms etc.) already comes as HTML*/
                descr += key === 'description' ? `<br><br>${value}` : `<br><br><strong>${key}:</strong> ${value}`;
            });
        });

        if (descr === '') {
            return;
        }
        descr += `<br><br><strong>Lon: ${coordinates.lng.toFixed(6)} °
                  <br>Lat: ${coordinates.lat.toFixed(6)} °
</strong>`;

        new maplibregl.Popup({ maxWidth: '400px' })
            .setLngLat(coordinates)
            .setHTML(`<div style="max-height: 350px; overflow-y: auto">${descr}</div>`)
            .addTo(map);
    });
}