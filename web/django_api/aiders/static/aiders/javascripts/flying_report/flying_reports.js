{
    function go_pdf(element) {
        $.ajax({
            url: dutils.urls.resolve('media', { path: element.value }),
            type: 'GET',
            success: function () {
                window.location = dutils.urls.resolve('media', { path: element.value });
            },
        });
    }
}
