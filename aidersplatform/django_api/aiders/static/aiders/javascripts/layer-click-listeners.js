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


    map.on('click', layerPoles.id, (e)=>{
        const coordinates = e.features[0].geometry.coordinates.slice();

        const description = e.features[0].properties.description;
        const line_name =  e.features[0].properties["line_name"];
        const date =  e.features[0].properties["construction_date"];
        const height =  e.features[0].properties["structure_height"];
        let descr = `<strong>Line Name: ${line_name}
                     <br>Construction Date: ${date}
                     <br>Height: ${height} m                 
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
        const line_name =  e.features[0].properties["Line_Name"];

        const dataType =  e.features[0].properties["Data_type"];
        const voltage =  e.features[0].properties["Voltage"];
        const operatingVoltage =  e.features[0].properties["OperVolltg"];
        const status =  e.features[0].properties["Status"];
        const region =  e.features[0].properties["Region"];
        const length =  e.features[0].properties["Shape_Leng"];
        let descr = `<strong>Line Name: ${line_name}
                     <br>Data Type: ${dataType}
                     <br>Voltage: ${voltage}
                     <br>Status: ${status}
                     <br>Voltage: ${voltage}
                     <br>Operating Voltage: ${operatingVoltage}
                     <br>Region: ${region}
                     <br>Length: ${length.toFixed(0)} m
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
}