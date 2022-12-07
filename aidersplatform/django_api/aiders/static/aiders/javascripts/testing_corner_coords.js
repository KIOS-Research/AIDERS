{
    let image_url = "http://localhost:8000/IMG_0035_1_ucy.png"
    let center = [ 33.4152074, 35.1448747]
    let width_meters = 44.52286853085
    let height_meters = 33.4595319
    let bearing = -20.218516294
    let geometryModel = create_geometry_model(image_url, width_meters, height_meters, center, bearing)
    let layer_id = "test_layer";
    if (!map.getLayer(layer_id))
    {
        let emptyLayer = create_empty_layer(layer_id)
        map.addLayer(emptyLayer)
    }
    tb.add(geometryModel,layer_id)
    map.setStyle('mapbox://styles/mapbox/light-v9')
    map.addSource('test_test_photo', {
        "type": "image",
        "url": image_url,
        "coordinates": [
            [ -119.73236304904971 , 46.294446917194556 ],
            [ -119.73209678900581 , 46.294803148565926 ],
            [ -119.73171234965834 , 46.2946650818776 ],
            [ -119.73197861152583 , 46.29430885139282 ],

        ]
    })

    map.addLayer({
        "id": 'layer_test_test_photo',
        "source": 'test_test_photo',
        "type": "raster",
        "paint": {"raster-opacity": 0.6}
    })  
}