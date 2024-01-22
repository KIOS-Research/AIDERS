{
    let clickMarker = new maplibregl.Marker({
        color: '#314ccd',
    });

    let lonMarker;
    let latMarker;
    let fire_ajax_req;

    function placeMarkerOnClick(e) {
        clickMarker.setLngLat(e.lngLat).addTo(map);
        lonMarker = e.lngLat.lng;

        latMarker = e.lngLat.lat;
    }

    function runFireSpreadDetAlgorithm() {
        map.on('click', placeMarkerOnClick); //ALlow user to place a marker as the initial fire position
        let boxDiv = $('#fireSpreadBox');
        boxDiv.toggle();
        postElementId('Fire Spread', 'Click');
    }

    function clearFireDataFromMap() {
        let okButton = 'Clear';
        let cancelButton = 'Cancel';
        let message = 'Are you sure you want to clear all Fire Prediction data from the map? ';
        let dialogTitle = 'Confirm';
        postElementId('Clear All Fire Spread', 'Click');
        create_confirmation_dialog(okButton, cancelButton, message, dialogTitle).then(function (willClear) {
            if (willClear) {
                let all_sources = map.getStyle().sources;
                let fireDataFound = false;
                for (let source_id in all_sources) {
                    //Remove the images from the map that belong to the period that user specified
                    if (!all_sources.hasOwnProperty(source_id)) continue;
                    if (source_id.includes('geojson_fire_data')) {
                        fireDataFound = true;
                        map.removeLayer(source_id);
                        map.removeLayer(source_id + '_outline');
                        map.removeSource(source_id);
                    }
                }

                lonMarker = undefined;
                latMarker = undefined;
                if (fireDataFound) {
                    create_popup_for_a_little(SUCCESS_ALERT, 'Fire Prediction data removed from the map!', 2000);
                } else {
                    create_popup_for_a_little(WARNING_ALERT, 'No Fire Prediction data to clear!', 2000);
                }

                clickMarker.remove();
            }
        });
    }
    function check_for_empty_value(
        timeStepsInput,
        initialFFinput,
        windAngleInput,
        windSpeedInput,
        fireSpeedInput,
        timeStepUnit,
        timeIntervalInput,
        lon
    ) {
        let msg;
        let canProceed = true;

        switch ('') {
            case timeStepsInput:
                msg = 'The field "Time Steps" was left empty!';
                canProceed = false;
                break;
            case initialFFinput:
                msg = 'The field "Initial Fire Fronts" was left empty!';
                canProceed = false;
                break;
            case windAngleInput:
                msg = 'The field "Wind Angle" was left empty!';
                canProceed = false;
                break;
            case windSpeedInput:
                msg = 'The field "Wind Speed" was left empty!';
                canProceed = false;
                break;
            case fireSpeedInput:
                msg = 'The field "Time Steps" was left empty!';
                canProceed = false;
                break;
            case timeIntervalInput:
                msg = 'The field "Time Interval" was left empty!';
                canProceed = false;
                break;
            case timeStepUnit:
                msg = 'No units selected for the "Time Steps" field';
                canProceed = false;
                break;
            default:
                canProceed = true;
            // code block
        }

        if (lon === undefined) {
            msg = 'No location provided! Click anywhere on the map to set the starting location of fire';
            console.log('NO LOCATION PROVIDED!');
            canProceed = false;
        }

        let input_error_div_id;
        if (!canProceed) {
            input_error_div_id = create_popup_for_a_little(WARNING_ALERT, msg, 4000);
        }
        return {
            canProceed: canProceed,
            divID: input_error_div_id,
        };
        // return canProceed
    }
    function submitFireSpreadInput() {
        let timeStepsInput = $('#timeStepsInput').val();
        let initialFFinput = $('#initialFFInput').val();
        let windAngleInput = $('#windAngleInput').val();
        let windSpeedInput = $('#windSpeedInput').val();
        let fireSpeedInput = $('#fireSpeedInput').val();
        let timeStepUnit = $('#inlineFormCustomSelect').val();
        let timeIntervalInput = $('#timeIntervalInput').val();
        let userInput = document.getElementById('userInput').value;

        let obj = check_for_empty_value(
            timeStepsInput,
            initialFFinput,
            windAngleInput,
            windSpeedInput,
            fireSpeedInput,
            timeStepUnit,
            timeIntervalInput,
            lonMarker,
            operationInput,
            userInput
        );

        timeStepsInput = parseInt(timeStepsInput) + 1;
        initialFFinput = parseInt(initialFFinput);
        windAngleInput = parseFloat(windAngleInput);
        windSpeedInput = parseFloat(windSpeedInput);
        fireSpeedInput = parseFloat(fireSpeedInput);
        timeIntervalInput = parseInt(timeIntervalInput);

        let canProceed = obj['canProceed'];
        if (!canProceed) return;
        let error_div_id = obj['divID'];
        windAngleInput = windAngleInput * (Math.PI / 180);
        let intervalArr = [];
        let k = 0;
        let fireSpreadBox = $('#fireSpreadBox');
        fireSpreadBox.toggle();
        // showPopup('#progressDiv','')
        $('#loadingAlgo').show();
        $('#progressDiv').show();
        removePopupByID('#temp_popup'); // Remove the temporary popup that might have been created due to input error
        for (let i = 0; i < timeStepsInput; i++) {
            if (k === timeIntervalInput) {
                intervalArr.push(i);
                k = 0;
            }
            k++;
        }
        // console.log("initialFFinput: " + initialFFinput)
        // console.log("windAngleInput: " + windAngleInput)
        // console.log("windSpeedInput: " + windSpeedInput)
        // console.log("fireSpeedInput: " + fireSpeedInput)
        // console.log("lonMarker: " + lonMarker)
        // console.log("latMarker: " + latMarker)
        // console.log("option: " + timeStepUnit)
        // console.log("intervalArr: " + intervalArr)
        if (timeStepUnit === 'minutes') {
            timeStepsInput = timeStepsInput * 60;
        }
        // let reqJson = {
        //     "fire_speed": 5,
        //     "fire_fronts": 31,
        //     "wind_speed": 5,
        //     "wind_angle": 0.3925,
        //     "time_steps": {
        //         "value": 1000,
        //         "units": "seconds"
        //     },
        //     "location":{
        //         "lon":33.377672,
        //         "lat": 35.161117
        //     },
        //     "time_intervals":[200,300,400,500,600, 800, 1000]
        // };

        let reqJson = {
            fire_speed: fireSpeedInput,
            fire_fronts: initialFFinput,
            wind_speed: windSpeedInput,
            wind_angle: windAngleInput,
            time_steps: {
                value: timeStepsInput,
                units: timeStepUnit,
            },
            location: {
                lon: lonMarker,
                lat: latMarker,
            },
            time_intervals: intervalArr,
            user: userInput,
        };

        var settings = {
            url: dutils.urls.resolve('fire_prediction', { operation_name: CURRENT_OP }),
            method: 'POST',
            timeout: 0,
            headers: {
                'Content-Type': 'application/json',
                'X-CSRFToken': document.getElementById('csrf').querySelector('input').value,
            },
            data: JSON.stringify(reqJson),
        };

        fire_ajax_req = $.ajax(settings).done(function (response) {
            response = JSON.parse(response);
            response = response[0];

            let geojsonSourceID = 'geojson_fire_data_' + new Date().valueOf();
            let onlyOnce = true;
            response.features = response.features.reverse(); //We need to reverse the order of the features because when clicking on the feature we have to get the information for the first. FIrst feature should be placed last so as to be on top
            load_fire_geojson(response, geojsonSourceID, 0.5, response.features.length);
            let firstCoord = response['features'][0]['geometry']['coordinates'][0][0];

            map.on('sourcedata', sourceCallback);

            //This is a callback to be notified when the geojson fire prediction data were loaded
            //This is because they take some time to load the geojson files
            function sourceCallback(e) {
                // console.log("event ")
                // console.log(e)
                if (map.getSource(geojsonSourceID) && map.isSourceLoaded(geojsonSourceID) && onlyOnce) {
                    hidePopup('#progressDiv');
                    let successMsg = 'Fire prediction results were successfully placed on the map!';
                    create_popup_for_a_little(SUCCESS_ALERT, successMsg, 2000);
                    map.flyTo({ center: firstCoord, zoom: 16 });
                    onlyOnce = false;
                    map.off('sourcedata', sourceCallback);
                    map.off('click', placeMarkerOnClick); //Once the fire data are loaded on the map, don't let user place markers on the map anymore

                    map.on('click', geojsonSourceID, (e) => {
                        let popupDisplay = '<strong >';
                        let clickedObj = e.features[0].properties;

                        Object.keys(clickedObj).forEach(function (key) {
                            popupDisplay += `${key}: ${clickedObj[key]}<br>`;
                        });
                        popupDisplay += `Lon: ${e.lngLat.lng.toFixed(6)} °<br>`;
                        popupDisplay += `Lat: ${e.lngLat.lat.toFixed(6)} °<br>`;
                        const div = window.document.createElement('div');
                        div.innerHTML = popupDisplay;
                        div.setAttribute('align', 'left');
                        div.setAttribute('style', 'font-size: 13px;');
                        new maplibregl.Popup().setLngLat(e.lngLat).setDOMContent(div).addTo(map);
                    });
                }
            }
        });
    }
    function RunFireSpreadOutput(output_data) {
        // output_data=JSON.parse(output_data)

        // output_data = output_data[1]
        let geojsonSourceID = 'geojson_fire_data_' + String(output_data.fields.id);
        let onlyOnce = true;
        output_data = output_data.fields.output;

        output_data.features = output_data.features.reverse(); //We need to reverse the order of the features because when clicking on the feature we have to get the information for the first. FIrst feature should be placed last so as to be on top
        load_fire_geojson(output_data, geojsonSourceID, 0.5, output_data.features.length);
        let firstCoord = output_data['features'][0]['geometry']['coordinates'][0][0];

        map.on('sourcedata', sourceCallback);

        //This is a callback to be notified when the geojson fire prediction data were loaded
        //This is because they take some time to load the geojson files
        function sourceCallback(e) {
            // console.log("event ")
            // console.log(e)
            if (map.getSource(geojsonSourceID) && map.isSourceLoaded(geojsonSourceID) && onlyOnce) {
                hidePopup('#progressDiv');
                let fireSpreadBox = $('#fireSpreadBox');
                fireSpreadBox.hide();
                let successMsg = 'Fire prediction results were successfully placed on the map!';
                create_popup_for_a_little(SUCCESS_ALERT, successMsg, 2000);
                map.flyTo({ center: firstCoord, zoom: 16 });
                onlyOnce = false;
                map.off('sourcedata', sourceCallback);
                map.off('click', placeMarkerOnClick); //Once the fire data are loaded on the map, don't let user place markers on the map anymore

                map.on('click', geojsonSourceID, (e) => {
                    let popupDisplay = '<strong >';
                    let clickedObj = e.features[0].properties;

                    Object.keys(clickedObj).forEach(function (key) {
                        popupDisplay += `${key}: ${clickedObj[key]}<br>`;
                    });
                    popupDisplay += `Lon: ${e.lngLat.lng.toFixed(6)} °<br>`;
                    popupDisplay += `Lat: ${e.lngLat.lat.toFixed(6)} °<br>`;
                    const div = window.document.createElement('div');
                    div.innerHTML = popupDisplay;
                    div.setAttribute('align', 'left');
                    div.setAttribute('style', 'font-size: 13px;');
                    new maplibregl.Popup().setLngLat(e.lngLat).setDOMContent(div).addTo(map);
                });
            }
        }
    }

    $.fn.dragIt = function () {
        var $this = this,
            ns = 'draggable_' + (Math.random() + '').replace('.', ''),
            mm = 'mousemove.' + ns,
            mu = 'mouseup.' + ns,
            $w = $(window),
            isFixed = $this.css('position') === 'fixed',
            adjX = 0,
            adjY = 0;

        $this.mousedown(function (ev) {
            var pos = $this.offset();
            if (isFixed) {
                adjX = $w.scrollLeft();
                adjY = $w.scrollTop();
            }
            var ox = ev.pageX - pos.left,
                oy = ev.pageY - pos.top;
            $this.data(ns, { x: ox, y: oy });
            $w.on(mm, function (ev) {
                ev.preventDefault();
                ev.stopPropagation();
                if (isFixed) {
                    adjX = $w.scrollLeft();
                    adjY = $w.scrollTop();
                }
                var offset = $this.data(ns);
                $this.css({ left: ev.pageX - adjX - offset.x, top: ev.pageY - adjY - offset.y });
            });
            $w.on(mu, function () {
                $w.off(mm + ' ' + mu).removeData(ns);
            });
        });

        return this;
    };

    function minimizeFireLoadingBox(el_id) {
        $(el_id).remove();
        let msg = 'Map will zoom to the results, once they are ready!';
        create_popup_for_a_little(WARNING_ALERT, msg, 2000);
        hidePopup('#fireSpreadBox');
    }

    function stopFireAlgorithm() {
        let okButton = 'Yes';
        let cancelButton = 'No';
        let msg = 'Are you sure you want to stop the process?';
        create_confirmation_dialog(okButton, cancelButton, msg, 'Confirm').then(function (willStop) {
            if (willStop) {
                if (fire_ajax_req !== undefined) {
                    fire_ajax_req.abort();
                    msg = 'Process Stopped!';
                    create_popup_for_a_little(WARNING_ALERT, msg, 2000);
                    hidePopup('#fireSpreadBox');
                    hidePopup('#progressDiv');
                }
            }
        });
    }
    function RemoveFireSpreadOutput(fireSpreadId) {
        let geojsonSourceID = 'geojson_fire_data_' + String(fireSpreadId);
        if (map.getSource(geojsonSourceID)) {
            if (map.getLayer(geojsonSourceID)) {
                map.removeLayer(geojsonSourceID);
            }
            map.removeLayer(geojsonSourceID + '_outline');
            map.removeSource(geojsonSourceID);
        }
    }
}
