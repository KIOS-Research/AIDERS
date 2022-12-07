map.on('mousemove', function (e)
{
    // console.log("HERE")
    document.getElementById('info').innerHTML =
// e.point is the x, y coordinates of the mousemove event relative
// to the top-left corner of the map
        JSON.stringify(e.point) +
        '<br />' +
        // e.lngLat is the longitude, latitude geographical position of the event
        JSON.stringify(e.lngLat.wrap());
});