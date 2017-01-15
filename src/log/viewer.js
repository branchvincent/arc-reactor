var level2name = [];
level2name[0] = "All";
level2name[10] = "Debug",
level2name[20] = "Info";
level2name[30] = "Warn";
level2name[40] = "Error";
level2name[50] = "None";

var level2class = [];
level2class[10] = "",
level2class[20] = "info";
level2class[30] = "warning";
level2class[40] = "danger";

var populate_filter_dropdown = function(id) {
    var filter = $(id);
    for(var i = 0; i <= 50; i += 10) {
        filter.append($("<option>").attr("value", i).text(level2name[i]));
    }

}

var pad = function(v, n) {
    var s = v.toString();
    while(s.length < n) {
        s = "0" + s;
    }
    return s;
}

var break_on = function(char, str) {
    return str.split(char).join(char + '\u200B');
}

var format_date = function(ms, ampm) {
    var date = new Date(ms);

    // form the date
    var s = (date.getMonth() + 1) + '/' + pad(date.getDate(), 2) + '/' + date.getFullYear() + ' ';

    // form the time
    if(ampm) {
        s += (date.getHours() % 12) || 12;
    } else {
        s += date.getHours();
    }
    s += ':' + pad(date.getMinutes(), 2) + ':' + pad(date.getSeconds(), 2) + '\u200B.' + pad(date.getMilliseconds(), 3);

    // add the suffix
    if(ampm) {
        s += ' ' + (date.getHours() < 12 ? 'am' : 'pm');
    }

    return s;
}

var build_record_row = function(record, options) {
    var icon = $("<span>");
    if(record.levelno >= 40) {
        icon.addClass("glyphicon glyphicon-exclamation-sign");
    //} else if(record.levelno >= 30) {
    //    icon.addClass("glyphicon glyphicon-alert");
    }

    var lines = record.message.split(/(?:\n|\r)+/g);
    var message = $('<td>').append($('<div>').text(lines[0].trim()));
    for(var j = 1; j < lines.length; j++) {
        message.append($('<div>').text(lines[j].trim()));
    }

    if(record.exception) {
        // build exception expand link and panel
        message.children('div').last().append(
            ' ',
            $('<a>')
                .attr('role', 'button')
                .attr('href', '#')
                .text('>>>')
                .click(function() { $('#e' + record.id).toggle(); }),
            $('<pre>')
                .attr('id', 'e' + record.id)
                .text(record.exception)
                .hide()
        );
    }

    var row = $('<tr>').attr('id', 'r' + record.id).append(
        $('<td>').text(format_date(1000 * record.created, options.ampm)),
        $('<td>').text((options.hostname ? record.hostname : record.ip) + '\u200B:' + record.port),
        $('<td>').text(break_on('.', record.name)),
        $('<td>').text(break_on('/', record.pathname) + '\u200B:' + record.lineno),
        $('<td>').append(icon),
        message
    ).addClass(level2class[record.levelno]);

    return row;
}

var update_source_filters = function(sources) {
    var sort = false;

    for(var i = 0; i < sources.length; i++) {
        var id = 'filter-' + sources[i].replace('.', '-');

        if($('#' + id).length) {
            continue;
        }
        sort = true;

        $('<div>').data('name', sources[i]).addClass('form-group col-md-2').append(
            $('<label>')
                .addClass('control-label')
                .attr('for', id)
                .text(break_on('.', sources[i]))
                .append('&nbsp;'),
            $('<select>')
                .attr('id', id)
                .data('name', sources[i])
                .addClass('form-control')
        ).appendTo('#source_filters');

        populate_filter_dropdown('#' + id);
    }

    if(sort) {
        $("#source_filters").children("div").sort(function(a, b) {
            return $(a).data("name") > $(b).data("name");
        }).appendTo("#source_filters");
    }
}

var options;

var load = function(table, skip, count) {
    var filter = '';
    // build the filter query argument
    for(var key in options.filters) {
        if(options.filters.hasOwnProperty(key)) {
            filter += key + ':' + options.filters[key] + '!';
        }
    }
    // drop the trailing delimiter
    filter = filter.substring(0, filter.length - 1);

    // assemble payload
    var payload = {
        skip: skip,
        count: count,

        level: options.level,
        filter: filter,
        search: options.search,
    };

    var dfd = new $.Deferred();

    request = $.getJSON('records', payload)
    .always(function() {
        request = null;
    })
    .done(function(response) {
        update_source_filters(response.names);

        for(var i = 0; i < response.records.length; i++) {
            var row = build_record_row(response.records[i], options);
            if(options.reverse) {
                row.prependTo("#records tbody");
            } else {
                row.appendTo("#records tbody");
            }
        }

        var percent = 100.0 * (skip + response.records.length) / response.available;
        $("#progress").css("width", percent + "%");

        dfd.resolve(response.records.length);
    })
    .fail(function(xhr, status, error) {
        dfd.fail(status);
    });

    return dfd;
}

var clear = function() {
    // cancel any active request
    if(request) {
        request.abort();
        updating = false;
    }

    $("#records tbody").empty();
}

var updating = false;
var request = null;

var update = function(force) {
    // prevent concurrent updates
    if(updating) {
        return;
    }
    updating = true;

    load('#records', $('#records tr').length, 50, options)
    .always(function() {
        updating = false;
    })
    .done(function(count) {
        if(count) {
            // load more
            update();
        } else {
            // done
        }
    });
}

var reload = function() {
    // reset any current operations
    clear();

    // generate and cache display options for timer updates
    options = {
        level: $('#filter option:selected').val(),
        filters: {},

        ampm: $("#time_12hr").is(":checked"),
        hostname: $("#host_name").is(":checked"),
        reverse: !$("#order_down").is(":checked"),

        search: $('#search').val()
    }

    $("#source_filters").find("select").each(function(i, e) {
        options.filters[$(e).data("name")] = $(e).children(":selected").val();
    });

    // trigger an update
    update();
}

var setup = function() {
    // setup filter dropdown
    populate_filter_dropdown("#filter");

    $("#reload").click(reload);
    $("#do_search").click(reload);

    $("#options_button").click(function() { $("#options_panel").toggle(); });
    $("#source_button").click(function() { $("#source_filters").toggle(); });

    // handy shortcuts for the various panels
    $(document).bind("keypress", function(e) {
        // pass input to search box
        if($(e.target).attr('id') === 'search') {
            return;
        }

        var code = e.keyCode || e.which;

        switch(code) {
        case 114: // r
            $("#reload").click();
            e.preventDefault();
            break;

        case 111: // o
            $("#options_button").click();
            e.preventDefault();
            break;

        case 102: // f
            $("#source_button").click();
            e.preventDefault();
            break;

        case 115: // s
            $("#search").select();
            break;

        default:
            break;
        }
    });

    // handy shortcuts for enter and escape
    $("#search").bind("keypress", function(e) {
        var code = e.keyCode || e.which;

        if(code == 13) {
            $("#reload").click();
            e.preventDefault();
        } else if(code == 27) {
            $("#search").blur();
            e.preventDefault();
        }
    })
}

$(document).ready(function() {
    setup();
    reload();

    // periodically trigger an update
    setInterval(function() {
        if(!$("#auto_off").is(":checked")) {
            update();
        }
    }, 500);
});