/*
 * Adds tghe functionality where user is able to draw lines or rectangles on the map and determine the length or the area
 * */

// var draw = new MapboxDraw({
//     displayControlsDefault: false,
//     controls: {
//         line_string: true,
//         polygon: true,
//         boxSelect: true,
//         trash: true
//     }

// });

if (USE_ONLINE_MAPS == true) {
    var draw = new MapboxDraw({
        // displayControlsDefault: false,
        // controls: {
        //     line_string: true,
        //     polygon: true,
        //     boxSelect: true,
        //     trash: true,
        // },
    });
}
function updateArea(e) {
    if (USE_ONLINE_MAPS == true) {
        var data = draw.getAll();
        var areaAnswer = document.getElementById('calculated-area1');
        var lengthAnswer = document.getElementById('calculated-area2');
        if (data.features.length > 0) {
            var area = turf.area(data);
            var distance = turf.length(data).toLocaleString();
            var rounded_area = Math.round(area * 100) / 100;
            areaAnswer.innerHTML = '<strong>' + rounded_area + 'm<sup>2</sup></strong>';

            lengthAnswer.innerHTML = '<strong>' + distance + 'km</strong>';
        } else {
            areaAnswer.innerHTML = 'Null';
            lengthAnswer.innerHTML = 'Null';

            if (e.type !== 'draw.delete') {
                alert('Use the draw tools to draw a polygon!');
            }
        }
    }
}

var button;
function toggleDrawingVisibility(toggleID) {
    var pressed = $('#' + toggleID).is(':checked');
    postElementId(toggleID, pressed);
    var boxDiv = document.getElementById('calculation-box-div1');
    if (pressed) {
        boxDiv.style.display = 'block';
        map.addControl(draw);
        var drawPolygon = document.getElementsByClassName('mapbox-gl-draw_polygon');
        drawPolygon[0].onclick = (e) => {};

        map.on('draw.create', updateArea);
        map.on('draw.delete', updateArea);
        map.on('draw.update', updateArea);
    } else {
        map.removeControl(draw);
        boxDiv.style.display = 'none';
    }
}
