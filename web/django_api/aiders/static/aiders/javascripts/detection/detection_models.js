{
    let prototypeModels = [];

    function DetectedModel(type, geometry, material, color, mesh, model) {
        this.type = type;
        this.color = color;
        this.checkboxID = 'det' + type + 'Checkbox';
        this.geometry = geometry;
        this.mesh = mesh;
        this.model = model;
        this.counter = 0;
    }

    function getMaterial(color) {
        return new THREE.MeshPhongMaterial({
            color: color,
            opacity: 1,
            transparent: true,
            side: THREE.DoubleSide,
        });
    }

    /*
     * The detection algorithm detects certain objects.
     * If user decides to start detection, a prototype model is created for each object
     * The aim is to create once these objects at the beginning to avoid creating them every single time
     * Instead, they will be duplicated.
     * */
    function create_prototype_models(availableTypes) {
        prototypeModels = [];

        for (let i = 0; i < availableTypes.length; i++) {
            let geometry;
            let color = 'rgb(255,0,0)';
            if (availableTypes[i] === CAR) {
                geometry = new THREE.BoxGeometry(1.47, 1.5, 1.6); //(length,height,depth)
                color = CAR_COLOR;
            } else if (availableTypes[i] === PERSON) {
                geometry = new THREE.BoxGeometry(0.5, 1.75, 0.5); //(length,height, depth)
                color = PERSON_COLOR;
            } else if (availableTypes[i] === BICYCLE) {
                geometry = new THREE.BoxGeometry(0.51, 1.7, 1.1); //(width,length,depth)
                color = BICYCLE_COLOR;
            }
            let material = getMaterial(color);
            let mesh = new THREE.Mesh(geometry, material);
            let options = {
                obj: mesh,
                adjustment: { x: 0.5, y: 0.5, z: 0 },
                units: 'meters',
                rotation: { x: 90, y: 0, z: 0 },
                anchor: 'center',
            };
            let model = tb.Object3D(options);
            let detectedModel = new DetectedModel(availableTypes[i], geometry, material, color, mesh, model);
            prototypeModels.push(detectedModel);
        }
    }

    function get_prototype_models() {
        return prototypeModels;
    }

    function duplicate_model(modelType) {
        for (let i = 0; i < prototypeModels.length; i++) {
            if (prototypeModels[i].type === modelType) {
                return prototypeModels[i].model.duplicate();
            }
        }

        return NOT_FOUND_STRING;
    }

    function initialize_detected_objects_counter(modelType) {
        for (let i = 0; i < prototypeModels.length; i++) {
            prototypeModels[i].counter = 0;
        }
    }

    function get_updated_chart_data() {
        let updatedData = [];
        for (let i = 0; i < prototypeModels.length; i++) {
            updatedData.push(prototypeModels[i].counter);
        }
        return updatedData;
    }

    function increase_counter(modelType) {
        for (let i = 0; i < prototypeModels.length; i++) {
            if (prototypeModels[i].type === modelType) {
                prototypeModels[i].counter++;
                // console.log("COUNTER FOR " + prototypeModels[i] + " :" + prototypeModels[i].counter)
                return;
            }
        }

        return NOT_FOUND_STRING;
    }

    function get_number_of_detected_objects(objType) {
        for (let i = 0; i < prototypeModels.length; i++) {
            if (prototypeModels[i].type === objType) {
                return prototypeModels[i].counter;
            }
        }

        return 0;
    }

    function print_all_prototypes() {
        for (let i = 0; i < prototypeModels.length; i++) {
            console.log(prototypeModels[i]);
        }
    }

    /*
     * Returns the layer that will host the objects detected by drone
     * */
    function get_prot_models_layer(availableTypes) {
        return {
            id: 'det_objects_layer',
            type: 'custom',
            renderingMode: '3d',
            onAdd: function () {
                for (let i = 0; i < availableTypes.length; i++) {
                    tb.add(prototypeModels[i].model);
                }
            },
            render: function (gl, matrix) {
                // tb.update();
            },
        };
    }

    function http_get_available_types() {
        console.log('AAAAAAAAAAAAAAAAAAAAAAA http_get_available_types');

        var settings = {
            url: BASE_URL_COCO_NAMES,
            method: 'GET',
            timeout: 0,
            headers: {
                'Content-Type': 'text/plain',
            },
        };

        return $.ajax(settings).done(function (response) {
            console.log(response);
            return response;
        });
    }

    function get_available_classes() {
        let available_classes = [];
        for (let i = 0; i < prototypeModels.length; i++) {
            available_classes.push(prototypeModels[i].type);
        }
        return available_classes;
    }
}
