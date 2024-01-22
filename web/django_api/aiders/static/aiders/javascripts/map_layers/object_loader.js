{
    function loadExternalODMAPI() {
        postElementId('Construct 3D Object', 'Click');
        let msg =
            'Generate orthophotos and 3D models from aerial images using <b>NodeODM</b>. <br><br>' +
            'As soon as the NodeODM task is completed, you will be able to visualize the output on the map through the Algorithms page.<br><br>' +
            ' - You can skip the 3D model generation if you only need an orthophoto. <br /><br />' +
            ' - <b>Please don\'t skip the orthophoto generation as it\'s required in order to extract the location metadata.</b>';
        create_confirmation_dialog('Launch NodeODM', 'Cancel', msg, 'Aerial Image Processing').then(function (canProceed) {
            if (canProceed) {
                let params = `scrollbars=no,resizable=no,status=no,location=no,toolbar=no,menubar=no,
                                width=1000,height=700,left=100,top=100`;
                let win = open('http://' + document.location.hostname + ':4000', 'test', params);
                // var timer = setInterval(function() {
                //     if(win.closed) {
                //         clearInterval(timer);
                //         let msg = "Open the same window to chec!"
                //         create_popup_for_a_little(WARNING_ALERT,msg,4000)
                //     }
                // }, 500);
            }
        });
    }

    function onClick(event) {
        event.target.value = '';
    }

    function load3DObject(objUri, objectCoordinates) {
        objUri.replace('*IP_HERE*', location.hostname);
        let mtlUri = objUri.substr(0, objUri.lastIndexOf('.')) + '.mtl';
        map.addLayer({
            id: 'object_' + objUri,
            type: 'custom',
            renderingMode: '3d',
            onAdd: function (map, mbxContext) {
                let options = {
                    type: 'mtl',
                    obj: objUri,
                    mtl: mtlUri,
                    scale: 1,
                    units: 'meters',
                    rotation: { x: 0, y: 0, z: 180 }, //default rotation
                    anchor: 'top-left',
                    uri: objUri,
                    layerId: 'object_' + objUri,
                    clone: false,
                };

                tb.loadObj(options, function (model) {
                    let mesh = model.setCoords(objectCoordinates);
                    tb.add(mesh);
                    let msg = 'The 3D object was loaded successfully!';

                    create_popup_for_a_little(SUCCESS_ALERT, msg, 2000);
                    remove_all_build_map_periods();
                    $(`.${WARNING_ALERT}`).hide();
                });
            },

            render: function (gl, matrix) {},
        });

        setTimeout(function () {
            map.flyTo({ center: objectCoordinates, zoom: 15 });
        }, 1500);
    }


    function loadOrthophoto(orthophotoUri, topLeft, topRight, bottomRight, bottomLeft) {
        orthophotoUri.replace('*IP_HERE*', location.hostname);
        let jpgUri = orthophotoUri.substr(0, orthophotoUri.lastIndexOf('.')) + '.png';

        map.addSource('custom-image', {
            type: 'image',
            url: jpgUri,
            coordinates: [
                 bottomRight, topRight, topLeft, bottomLeft,
            ],
        });

        map.addLayer({
            id: 'custom-image-layer',
            source: 'custom-image',
            type: 'raster',
            paint: {
                'raster-opacity': 1,
            },
        });
        
        setTimeout(function () {
            map.flyTo({ center: topLeft, zoom: 15 });
        }, 1500);        
    }



    function clearObjectFromMap() {
        let okButton = 'Clear';
        let cancelButton = 'Cancel';
        let message = 'Are you sure you want to clear all 3d objects from the map? ';
        let dialogTitle = 'Confirm';
        postElementId('Clear Construct 3D Object', 'Click');
        create_confirmation_dialog(okButton, cancelButton, message, dialogTitle).then(function (willClear) {
            if (willClear) {
                let all_threebox_layers = tb.world.children;
                let object_found = false;
                for (let i = 0; i < all_threebox_layers.length; i++) {
                    let obj = all_threebox_layers[i];
                    if (obj.userData.uri !== undefined && obj.userData.uri.includes('object_3d_results_')) {
                        object_found = true;
                        tb.remove(obj);
                        tb.removeLayer(obj.userData.layerId);
                    }
                    if (object_found) {
                        create_popup_for_a_little(SUCCESS_ALERT, 'Object(s) removed from the map!', 2000);
                    } else {
                        create_popup_for_a_little(WARNING_ALERT, 'No objects found!', 2000);
                    }
                }
            }
        });
    }
}
