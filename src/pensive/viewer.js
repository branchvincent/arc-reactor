var updating = false;
var dirty = false;
var request = null;

var last_path = null;
var last_store = null;

var keys = function(obj) {
    var ks = [];
    for(var k in obj) {
        if(obj.hasOwnProperty(k)) {
            ks.push(k);
        }
    }

    return ks;
}

var load = function(store, orig_path) {
    var path = orig_path || '';

    var url = '';
    if(store) {
        url += '/i/' + store;
    } else {
        url += '/d/';
    }

    var dfd = new $.Deferred();

    request = $.ajax({
        type: 'POST',
        url: url + '?depth=1',
        data: JSON.stringify({operation: 'index', keys: [path]}),
        processData: false
    })
    .always(function() {
        request = null;
    })
    .done(function(response) {
        if(path != last_path) {
            // update breadcrumbs
            $('#path').empty();
            var levels = path.split('/');
            var subpath = '';last_path
            for(var i = 0; i < levels.length; i++) {
                subpath = levels.slice(0, i + 1).join('/');

                if(i == levels.length - 1) {
                    $('<li>')
                        .text(i == 0 ? 'Root' : levels[i])
                        .addClass('active')
                    .appendTo('#path');
                } else {
                    $('<li>').append(
                        $('<a>')
                            .text(i == 0 ? 'Root' : levels[i])
                            .attr('href', '#')
                            .data('store', store)
                            .data('path', subpath)
                            .click(function() {
                                clear();
                                load($(this).data('store'), $(this).data('path'));
                            })
                    ).appendTo('#path');
                }
            }
        }

        last_path = path;
        last_store = store;

        var index = response[path];
        var subkeys = keys(index);

        if(subkeys.length) {
            // update the index view
            $('#index').empty();

            subkeys.sort();
            for(var i = 0; i < subkeys.length; i++) {
                $('<li>').append(
                    $('<a>')
                        .text(subkeys[i])
                        .attr('href', '#')
                        .data('path', path + '/' + subkeys[i])
                        .data('store', store)
                        .click(function() {
                            clear();
                            load($(this).data('store'), $(this).data('path'));
                        })
                ).appendTo('#index');
            }

            // clear the value view
            $('#value').empty();
        } else {
            // keep the index view

            // update the value view
            request = $.getJSON(url + path, function(response) {
                var value = JSON.stringify(response.value, undefined, 4);

                if(dirty && $('#value').val() != value) {
                } else {
                    $('#value').val(value).removeClass('dirty');
                    dirty = false;
                }
            });
        }

        dfd.resolve();
    })
    .fail(function(xhr, status, error) {
        dfd.reject(status, error);
    });

    return dfd;
}

var clear = function() {
    // cancel any active request
    if(request) {
        request.abort();
    }

    updating = false;
 }

var update = function(force) {
    // prevent concurrent updates
    if(updating) {
        return;
    }
    updating = true;

    load(last_store, last_path)
    .always(function() {
        updating = false;
    })
    .done(function() {
        $('#reload').removeClass('btn-warning btn-danger').addClass('btn-primary');
    })
    .fail(function(status, error) {
        if(status === 'abort') {
            // ignore
        } else if(status === 'timeout') {
            $('#reload').removeClass('btn-primary btn-danger').addClass('btn-warning');
        } else {
            $('#reload').removeClass('btn-primary btn-warning').addClass('btn-danger');
        }
    });
}

var reload = function() {
    // reset any current operations
    clear();

    // trigger an update
    update();
}

var setup = function() {
    $('#value').on('change keydown paste cut', function() {
        dirty = true;
        $('#value').addClass('dirty');
    });

    $('#revert').click(function() {
        dirty = false;
    });

    $('#save').click(function() {
        var url = '';
        if(last_store) {
            url += '/i/' + last_store;
        } else {
            url += '/d/';
        }
        url += last_path;

        $.ajax({
            type: 'PUT',
            url: url,
            data: '{"value": ' + $('#value').val() + '}',
            processData: false
        }).fail(function(xhr, status, error) {
            alert('Updating "' + last_store + ':' + last_path + '" failed!\n\n' + status + ' ' + error);
        });
    });

    $('#reload').click(reload);
}

$(document).ready(function() {
    $.ajaxSetup({timeout: 1000});

    setup();

    reload();

    // periodically trigger an update
    setInterval(function() {
        //if(!$('#auto_off').is(':checked')) {
            reload();
        //}
    }, 500);
});