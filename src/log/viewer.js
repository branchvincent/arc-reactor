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

var populate_filter = function(id) {
    var filter = $(id);
    for(var i = 0; i <= 50; i += 10) {
        filter.append($("<option>").attr("value", i).text(level2name[i]));
    }

}

var setup = function() {
    // setup filter dropdown
    populate_filter("#filter");

    $("#reload").click(function() {
        var level = $("#filter option:selected").val();
        clear_all_records("#records");

        var filters = [];
        $("#source_filters").find("select").each(function(i, e) {
            filters[$(e).data("name")] = $(e).children(":selected").val();
        });

        load_all_records("#records", 50, level, filters, $("#search").val());
    });

    $("#do_search").click(function() {
        $("#reload").click();
    })

    $("#options_button").click(function() {
        $("#options_panel").toggle();
    });

    $("#source_button").click(function() {
        $("#source_filters").toggle();
    });

    $(document).bind("keypress", function(e) {
        if(e.target.tagName.toLowerCase() == "input") {
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

var pad = function(v, n) {
    var s = v.toString();
    while(s.length < n) {
        s = "0" + s;
    }
    return s;
}

var format_date = function(millis, ampm) {
    var date = new Date(millis);

    var s = (date.getMonth() + 1) + "/" + pad(date.getDate(), 2) + "/" + date.getFullYear() + " ";
    if(ampm) {
        s += (date.getHours() % 12) || 12;
    } else {
        s += date.getHours();
    }
    s += ":" + pad(date.getMinutes(), 2) + ":" + pad(date.getSeconds(), 2) + "." + pad(date.getMilliseconds(), 3);
    if(ampm) {
        s += " " + (date.getHours() < 12 ? "am" : "pm");
    }

    return s;
}

var load_records = function(table, skip, count, level, filters, search, next) {
    s = '';
    for(var key in filters) {
        if(filters.hasOwnProperty(key)) {
            s += key + ":" + filters[key] + "!";
        }
    }
    s = s.substring(0, s.length - 1);

    $.ajax({ url: "records", type: "GET", data: {level: level, skip: skip, count: count, filter: s, search: search}, dataType: "json"})
    .done(function(obj){
        var ampm = $("#time_12hr").is(":checked");
        var hostname = $("#host_name").is(":checked");
        var reverse = $("#order_up").is(":checked");

        var name_update = false;

        var sources = obj.names;
        for(var i = 0; i < sources.length; i++) {
            var id = "filter-" + sources[i].replace('.', '-');

            if($("#" + id).length) {
                continue;
            }
            name_update = true;

            var row = $("<div>").data("name", sources[i]).addClass("form-group col-md-2").append(
                $("<label>").addClass("control-label").attr("for", id).text(sources[i].replace(".", ".\u200B")).append("&nbsp;"),
                $("<select>").attr("id", id).data("name", sources[i]).addClass("form-control")
            ).appendTo("#source_filters");

            populate_filter("#" + id);
        }

        // sort divs after adding filter entries
        if(name_update) {
            $("#source_filters").children("div").sort(function(a, b) {
                return $(a).data("name") > $(b).data("name");
            }).appendTo("#source_filters");
        }

        for(var i = 0; i < obj.records.length; i++) {
            var record = obj.records[i];

            var icon = $("<span>");
            if(record.levelno >= 40) {
                icon.addClass("glyphicon glyphicon-exclamation-sign");
            //} else if(record.levelno >= 30) {
            //    icon.addClass("glyphicon glyphicon-alert");
            }

            var lines = record.message.split(/(?:\n|\r)+/g);
            var message = $("<td>").append(lines[0].trim());
            for(var j = 1; j < lines.length; j++) {
                message.append("<br />").append(lines[j].trim());
            }

            if(record.exception) {
                message
                .append("&nbsp;")
                .append($("<a>")
                        .attr("role", "button")
                        .attr("data-toggle", "collapse")
                        .attr("data-target", "#extra" + record.id)
                        .attr("href", "#record" + record.id)
                        .text(">>>"))
                .append($("<pre>")
                    .attr("id", "extra" + record.id)
                    .addClass("collapse")
                    .text(record.exception));
            }

            var row = $("<tr>").attr("id", "record" + record.id).append(
                $("<td>").append(format_date(1000 * record.created, ampm)),
                $("<td>").text((hostname ? record.hostname : record.ip) + "\u200B:" + record.port),
                $("<td>").text(record.name.replace(".", ".\u200B")),
                $("<td>").text(record.pathname.replace("/", "/\u200B") + "\u200B:" + record.lineno),
                $("<td>").append(icon),
                message
            ).addClass(level2class[record.levelno]);

            if(reverse) {
                row.prependTo("#records tbody");
            } else {
                row.appendTo("#records tbody");
            }
        }

        var percent = 100.0 * (skip + obj.records.length) / obj.available;

        $("#progress").css("width", percent + "%");

        next(table, skip, obj.records.length);
    });
}

var load_all_records = function(table, rate, level, filters, search) {
    var next = function(table, skip, count) {
        if(count < rate) {
            // done
        } else {
            load_records(table, skip + rate, rate, level, filters, search, next)
        }
    };

    load_records("#records", 0, rate, level, filters, search, next);
}

var clear_all_records = function(table) {
    $(table + " > tbody").empty();
}

$(document).ready(function() {
    setup();
    load_all_records("#records", 50);
});