{
    cpu_core_csv = [];
    gpu_csv = [];
    ram_csv = [];
    temp_csv = [];
    Colors = [
        '#00ffff',
        '#0000ff',
        '#a52a2a',
        '#008b8b',
        '#006400',
        '#bdb76b',
        '#8b008b',
        '#556b2f',
        '#ff8c00',
        '#00008b',
        '#9932cc',
        '#8b0000',
        '#e9967a',
        '#9400d3',
        '#ff00ff',
        '#ffd700',
        '#008000',
        '#4b0082',
        '#00ff00',
        '#ff00ff',
        '#800000',
        '#000080',
        '#808000',
        '#ffa500',
        '#ffc0cb',
        '#800080',
        '#800080',
        '#ff0000',
        '#ffff00',
    ];
    var max_timer = 61;

    function add_data_to_array(list, item) {
        if (list.length >= max_timer) {
            list.shift();
            add_data_to_array(list, item);
        } else {
            list.push(item);
        }
    }

    function data_to_objects(data) {
        object_list = [];
        object = {};
        for (let i = 0; i <= 60; i++) {
            if (data[data.length - 1 - i] !== undefined) {
                object_list.push((object = { x: i, y: data[data.length - 1 - i] }));
            } else {
                object_list.push((object = { x: i, y: null }));
            }
        }
        return object_list;
    }

    function configure_table_add(config, name, color) {
        config.data.datasets.push({ data: '', label: name, showLine: true, backgroundColor: color, borderColor: color, fill: false });
        return config;
    }

    function configure_table_delete(config) {
        config.data.datasets.pop();
        return config;
    }

    function visualize_data(array, csv, chart, names, colors) {
        if (csv.length < array.length) {
            for (let i = csv.length; i < array.length; i++) {
                configure_table_add(chart, names[i], colors[i]);
                csv.push([]);
            }
        } else if (csv.length > array.length) {
            for (let i = csv.length; i < array.length; i--) {
                configure_table_delete(chart);
                csv.shift();
            }
        }
        object_data = [];
        for (let i = 0; i < array.length; i++) {
            add_data_to_array(csv[i], array[i]);
            object_data[i] = data_to_objects(csv[i]);
            chart.data.datasets[i].data = object_data[i];
        }
        chart.update();
    }

    function create_canvas_charts() {
        // CPU
        ctx = document.querySelector('#line-chart-cpu').getContext('2d');
        labels = [];
        for (let i = 0; i <= 60; i++) {
            labels.push(i);
        }
        config = {
            type: 'line',
            data: {
                labels: labels,
                datasets: [],
            },
            options: {
                animation: {
                    duration: 0,
                },
                scales: {
                    yAxes: [{ ticks: { min: 0, max: 100 } }],
                },
            },
        };
        cpu_cores_chart = new Chart(ctx, config);

        // GPU
        ctx = document.querySelector('#line-chart-gpu').getContext('2d');
        labels = [];
        for (let i = 0; i <= 60; i++) {
            labels.push(i);
        }
        config = {
            type: 'line',
            data: {
                labels: labels,
                datasets: [],
            },
            options: {
                animation: {
                    duration: 0,
                },
                scales: {
                    yAxes: [{ ticks: { min: 0, max: 100 } }],
                },
            },
        };
        gpu_chart = new Chart(ctx, config);

        // RAM
        ctx = document.querySelector('#line-chart-ram').getContext('2d');
        labels = [];
        for (let i = 0; i <= 60; i++) {
            labels.push(i);
        }
        config = {
            type: 'line',
            data: {
                labels: labels,
                datasets: [],
            },
            options: {
                animation: {
                    duration: 0,
                },
                scales: {
                    yAxes: [{ ticks: { min: 0, max: 100 } }],
                },
            },
        };
        ram_chart = new Chart(ctx, config);

        // Temperature
        ctx = document.querySelector('#line-chart-temp').getContext('2d');
        labels = [];
        for (let i = 0; i <= 60; i++) {
            labels.push(i);
        }
        config = {
            type: 'line',
            data: {
                labels: labels,
                datasets: [],
            },
            options: {
                animation: {
                    duration: 0,
                },
                scales: {
                    yAxes: [{ ticks: { min: 0, max: 100 } }],
                },
            },
        };
        temp_chart = new Chart(ctx, config);
    }

    create_canvas_charts();
    function connect_to_ws(wsUrl) {
        console.log('ws://' + window.location.host + '/' + dutils.urls.resolve(wsUrl));
        var ws = new WebSocket('ws://' + window.location.host + '/' + dutils.urls.resolve(wsUrl));
        ws.onopen = function () {
            // subscribe to some channels
            ws.send('1');
        };

        ws.onclose = function (e) {
            console.log('Socket is closed. Reconnect will be attempted in 1 second.');
            setTimeout(function () {
                connect_to_ws(wsUrl);
            }, 1000);
        };

        ws.onerror = function (err) {
            console.error('Socket encountered error: ', err.message, 'Closing socket');
            ws.close();
        };
        return ws;
    }
    var socket = connect_to_ws('ws_monitoring');
    socket.onmessage = function (e) {
        wsData = JSON.parse(e.data);
        names = [];
        colors = [];
        cpu_array = wsData.cpu_core_usage.toString().split(', ');
        for (let i = 1; i <= cpu_array.length + 1; i++) {
            names.push('Core ' + i.toString());
        }
        for (let i = 1; i <= cpu_array.length + 1; i++) {
            names.push('Core ' + i.toString());
        }
        colors = Colors;
        visualize_data(cpu_array, cpu_core_csv, cpu_cores_chart, names, colors);
        gpu_array = [wsData.gpu_usage];
        names = ['Usage'];
        colors = ['#008000'];
        visualize_data(gpu_array, gpu_csv, gpu_chart, names, colors);
        ram_array = [wsData.ram_usage, wsData.swap_usage];
        names = ['Usage', 'Swap Usage'];
        colors = ['#ff00ff', '#008000'];
        visualize_data(ram_array, ram_csv, ram_chart, names, colors);
        temp_array = [wsData.cpu_temp, wsData.gpu_temp];
        names = ['CPU', 'GPU'];
        colors = ['#ffc0cb', '#4b0082', '#ffa500'];
        visualize_data(temp_array, temp_csv, temp_chart, names, colors);
        disk_array = [wsData.disk_read / 1000000, wsData.disk_write / 1000000];
        names = ['Read', 'Write'];
        colors = ['#ffff00', '#00ffff'];
    };

    function updateDataWs() {
        socket.send(drone_name);
    }
    droneTimer = setInterval(updateDataWs, 1000);
}
