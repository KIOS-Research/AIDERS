
{
    let allStaticCameras = {};
    let allStaticCameraMarkers = {};

    function handleStaticCamerasUpdate(_staticCameras) {

        if (Object.keys(allStaticCameras).length === 0 && _staticCameras.length === 0) {
            document.getElementById("static-cameras-menu").style.display = "none";  // hide sidebar section for static-cameras
            return;
        }

        // Check if the message contains static cameras data
        if (_staticCameras) {
            // Call the function to update static cameras on the map
            _staticCameras.forEach(staticCamera => {
                // check if the static camera already exists in the allStaticCameras array
                if (!allStaticCameras[staticCamera.name]) {
                    allStaticCameras[staticCamera.name] = staticCamera;
                    // place the static camera information on the map and panel
                    placeStaticCameraOnMap(staticCamera);
                    // create video elements
                    create_video_elements(staticCamera.name);
                    placeStaticCameraOnPanel(staticCamera.name);
                }
            });
        }

        // check for any cameras that are not in the _staticCameras array
        Object.keys(allStaticCameras).forEach(staticCameraName => {
            if (!_staticCameras.some(staticCamera => staticCamera.name === staticCameraName)) {
                
                console.log(`Static camera ${staticCameraName} not found in the update, removing from map and panel.`);

                // delete video elements
                remove_video_element(staticCameraName);

                // remove the static camera from the panel
                let cameraPanelEntry = document.getElementById('static-camera-' + staticCameraName);
                if (cameraPanelEntry) {
                    cameraPanelEntry.remove();
                }

                // remove the static camera from the map
                allStaticCameraMarkers[staticCameraName].remove();
                
                // remove static camera from the global arrays
                delete allStaticCameraMarkers[staticCameraName];
                delete allStaticCameras[staticCameraName];
            }
        });

        if(Object.keys(allStaticCameras).length > 0) {
            document.getElementById("connected-static-cameras").innerHTML = Object.keys(allStaticCameras).length; // show number of static-cameras on sidebar
            document.getElementById("static-cameras-menu").style.display = "block";             // show sidebar section for static-cameras
        }
        else {
            document.getElementById("static-cameras-menu").style.display = "none";
        }

        // console.log(`All static cameras: ${JSON.stringify(allStaticCameras)}`);
    }



    function placeStaticCameraOnMap(staticCamera) {
        // Create a popup for the static camera
        const popup = new maplibregl.Popup({ offset: 0 }).setHTML(
            `<b>${staticCamera.name}</b><br>Model: ${staticCamera.model}`
        );

        // Create a DOM element for the marker
        const el = document.createElement('div');
        el.className = 'static-camera-marker';
        el.style.backgroundImage = 'url(/static/aiders/imgs/static_camera_icon.png)';
        el.style.backgroundSize = '32px 32px';
        el.style.width = '32px';
        el.style.height = '32px';
        el.style.cursor = 'pointer';

        // Create a marker for the static camera
        const marker = new maplibregl.Marker(el)
            .setLngLat([staticCamera.longitude, staticCamera.latitude])
            .setPopup(popup)
            .addTo(map);

        // Store the marker in the allStaticCameraMarkers object
        allStaticCameraMarkers[staticCamera.name] = marker;

        // // Add a click event listener to the marker
        // el.addEventListener('click', () => {
        //     // Open the popup when the marker is clicked
        //     marker.togglePopup();
        // });

        console.log(`Static camera ${staticCamera.name} placed on map.`);
    }



    // add staticCamera to the side-panel
    function placeStaticCameraOnPanel(_staticCameraName) {
    
        let ul = document.getElementById('static-camera-selection-list');
    
        let li = document.createElement('li');
        li.id = 'static-camera-' + _staticCameraName;
        ul.append(li);
    
        // create the row element for this staticCamera
        let staticCameraWrapperDiv = document.createElement('div');
        // $(staticCameraWrapperDiv).addClass('row');
        staticCameraWrapperDiv.style.fontSize = '13px';
        staticCameraWrapperDiv.style.marginBottom = '15px';
        staticCameraWrapperDiv.style.color = '#ffffff';
        staticCameraWrapperDiv.style.lineHeight = '2';
        staticCameraWrapperDiv.style.background = 'inherit';
        li.append(staticCameraWrapperDiv);
    
        // create and add the staticCamera's name
        let staticCameraHeader = document.createElement('div');
        $(staticCameraHeader).addClass('row client-header');
        let staticCameraNameHeader = document.createElement('div');
        staticCameraNameHeader.id = 'static-camera-name-' + _staticCameraName;
        $(staticCameraNameHeader).addClass('col-md-6');
        staticCameraNameHeader.innerHTML = "<a href='#' style='padding: 0px; opacity: 1;' onclick='zoomToClient(\"staticCamera\", \"" + _staticCameraName + "\")'>" + _staticCameraName + "</a>";
    
    
    
        staticCameraHeader.appendChild(staticCameraNameHeader);
        staticCameraWrapperDiv.appendChild(staticCameraHeader);
    
        // create the divs that will contain the toggle buttons
        let togglesRow = document.createElement('div');
        togglesRow.style.marginBottom = '4px';
    
        // create and append the checkboxes
        let selectCheckbox = createCheckbox('static-camera-select-toggle-' + _staticCameraName);
        let videoCheckbox = createCheckbox('drone-video-toggle-' + _staticCameraName);
        togglesRow.appendChild(selectCheckbox);
        togglesRow.appendChild(videoCheckbox);
    
        // append the div holding the checkboxes to the wrapper div
        // this must be done before converting checkboxes to toggles
        staticCameraWrapperDiv.appendChild(togglesRow);
    
        convertCheckboxToToggleButton(selectCheckbox, _staticCameraName, "check", "SELECT", toggleSelectStaticCamera, "off");
        convertCheckboxToToggleButton(videoCheckbox, _staticCameraName, "video", "LIVE", toggleVideoVisibility, "off");
    }



    function toggleSelectStaticCamera(_elementId, _staticCameraName) {
        let staticCameraMarker = allStaticCameraMarkers[_staticCameraName];
        console.log(allStaticCameraMarkers);
        
        if (staticCameraMarker) {
            // toggle the popup when the static camera is selected
            staticCameraMarker.togglePopup();
        } else {
            console.warn(`Static camera marker for ${_staticCameraName} not found.`);
        }
    }

    function getStaticCameraCoordinates(_name) {
        let staticCamera = allStaticCameras[_name];
        if (staticCamera) {
            return [staticCamera.longitude, staticCamera.latitude];
        }
        return null;
    }

    // Make the function globally accessible
    window.toggleSelectStaticCamera = toggleSelectStaticCamera;

}


